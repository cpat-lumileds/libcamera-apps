/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * adaptive_face_detect_cv_stage.cpp - Adaptive Face Detector implementation, using OpenCV
 * 
 * Based on face_detect_cv_stage.cpp
 */

/* 
Some helpful hints on writing your own stages:

*	Generally, the Process method should not take too long as it will block the imaging pipeline 
	and may cause stuttering.
	When time-consuming algorithms need to be run, it may be helpful to delegate them to 
	another asynchronous thread.

*	When delegating work to another thread, the way image buffers are handled currently 
	means that they will need to be copied.
	For some applications, such as image analysis, 
	it may be viable to use the "low resolution" image stream rather than full resolution images.

*	The post-processing framework adds multi-threading parallelism on a per-frame basis. 
	This is helpful in improving throughput if you want to run on every single frame. 
	Some functions may supply parallelism within each frame (such as OpenCV and TFLite). 
	In these cases it would probably be better to serialise the calls so as to suppress 
	the per-frame parallelism.

*	Most streams, and in particular the low resolution stream, have YUV420 format. 
	These formats are sometimes not ideal for OpenCV or TFLite so there may sometimes 
	need to be a conversion step.

*	When images need to be altered, doing so in place is much the easiest strategy.
*/

#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <libcamera/stream.h>
#include <memory>
#include <vector>
#include <set>
#include <map>

#include <libcamera/geometry.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

#include "lumileds/lumileds_i2c.hpp"
#include "lumileds/leds.hpp"

using namespace cv;

using Stream = libcamera::Stream;


class AdaptiveFaceDetectCvStage : public PostProcessingStage
{
public:
	AdaptiveFaceDetectCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	void Stop() override;

	void Teardown() override;

private:
	void detectFeatures(cv::CascadeClassifier &cascade);
	void drawFeatures(cv::Mat &img);

	void drawLines(cv::Mat &img);
	std::vector <std::map<std::string, pair_k>> calculate_face_corners(std::vector<cv::Rect>&);
	std::set<int> find_matching_segments(std::map<std::string, pair_k>&, 
										 std::map<int, std::unique_ptr<SegmentCoordinates>>&); 
	void check_borders(std::vector <std::map<std::string, pair_k>>&  face_coords, int width, int height);

	Stream *stream_;
	StreamInfo low_res_info_;
	Stream *full_stream_;
	StreamInfo full_stream_info_;
	std::unique_ptr<std::future<void>> future_ptr_;
	std::mutex face_mutex_;
	std::mutex future_ptr_mutex_;
	Mat image_;
	std::vector<cv::Rect> faces_;
	CascadeClassifier cascade_;
	std::string cascadeName_;
	double scaling_factor_;
	int min_neighbors_;
	int min_size_;
	int max_size_;
	int refresh_rate_;
	int draw_features_;

	bool adaptive_;
	std::mutex adaptive_mutex_;
	int seg_x_;
	int seg_y_;
	bool draw_lines_;
	//TODO: pointer to coords really necessary ?
	std::map<int, std::unique_ptr<SegmentCoordinates>> segment_coords_;

	std::string text_;
	int fg_;
	int bg_;
	double scale_;
	int thickness_;
	double alpha_;
	double adjusted_scale_;
	int adjusted_thickness_;

	LumiledsI2C i2c_;
};


#define NAME "adaptive_face_detect_cv"

/*
Return the name of the stage. This is used to match against stages listed in the 
JSON post-processing configuration file.
*/
char const *AdaptiveFaceDetectCvStage::Name() const
{
	return NAME;
}

/*
This method will read any of the stage’s configuration parameters from the JSON file.
*/
void AdaptiveFaceDetectCvStage::Read(boost::property_tree::ptree const &params)
{
	cascadeName_ =
		params.get<char>("cascade_name", "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml");
	if (!cascade_.load(cascadeName_))
		throw std::runtime_error("FaceDetectCvStage: failed to load haar classifier");
	scaling_factor_ = params.get<double>("scaling_factor", 1.1);
	min_neighbors_ = params.get<int>("min_neighbors", 3);
	min_size_ = params.get<int>("min_size", 32);
	max_size_ = params.get<int>("max_size", 256);
	refresh_rate_ = params.get<int>("refresh_rate", 5);
	draw_features_ = params.get<int>("draw_features", 1);

	adaptive_ = params.get<bool>("adaptive", false);
	draw_lines_ = params.get<bool>("draw_lines", false);
	seg_x_ = params.get<int>("seg_x", 3);
	seg_y_ = params.get<int>("seg_y", 3);

	text_ = params.get<std::string>("text");
	fg_ = params.get<int>("fg", 255);
	bg_ = params.get<int>("bg", 0);
	scale_ = params.get<double>("scale", 1.0);
	thickness_ = params.get<int>("thickness", 2);
	alpha_ = params.get<double>("alpha", 0.5);

}

/*
This is called just after the camera has been configured. 
It is a good moment to check that the stage has access to the streams it needs, 
and it can also allocate any resources that it may require.
*/
void AdaptiveFaceDetectCvStage::Configure()
{
	stream_ = nullptr;
	full_stream_ = nullptr;

	if (app_->StillStream()) // for stills capture, do nothing
		return;

	// Otherwise we expect there to be a lo res stream that we will use.
	stream_ = app_->LoresStream();
	if (!stream_)
		throw std::runtime_error("FaceDetectCvStage: no low resolution stream");
	// (the lo res stream can only be YUV420)
	low_res_info_ = app_->GetStreamInfo(stream_);

	// We also expect there to be a "full resolution" stream which defines the output coordinate
	// system, and we can optionally draw the faces there too.
	full_stream_ = app_->GetMainStream();
	if (!full_stream_)
		throw std::runtime_error("FaceDetectCvStage: no full resolution stream available");
	full_stream_info_ = app_->GetStreamInfo(full_stream_);
	if (draw_features_ && full_stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("FaceDetectCvStage: drawing only supported for YUV420 images");

	int stride_x = full_stream_info_.width / seg_x_;
    int stride_y = full_stream_info_.height / seg_y_;

    for (int j = 0; j < seg_y_; j++){
		 for (int i = 0; i < seg_x_; i++)
            segment_coords_[j * seg_x_ + i] = std::make_unique<SegmentCoordinates>(i * stride_x,  
                                                                                   j * stride_y, 
                                                                                   i * stride_x + stride_x, 
                                                                                   j * stride_y + stride_y);
	}
       
	// Adjust the scale and thickness according to the image size, so that the relative
	// size is preserved across different camera modes. Note that the thickness can get
	// rather harshly quantised, not much we can do about that.
	adjusted_scale_ = scale_ * full_stream_info_.width  / 1200;
	adjusted_thickness_ = std::max(thickness_ * full_stream_info_.width  / 700, 1u);

}

/*
This method presents completed camera requests for post-processing and is where the
necessary pixel manipulations or image analysis will happen. 

The function returns true if the post-processing framework is not 
to deliver this request on to the application.
*/
bool AdaptiveFaceDetectCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	{
		std::unique_lock<std::mutex> lck(future_ptr_mutex_);
		if (completed_request->sequence % refresh_rate_ == 0 &&
			(!future_ptr_ || future_ptr_->wait_for(std::chrono::seconds(0)) == std::future_status::ready))
		{
			libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
			uint8_t *ptr = (uint8_t *)buffer.data();
			Mat image(low_res_info_.height, low_res_info_.width, CV_8U, ptr, low_res_info_.stride);
			image_ = image.clone();

			future_ptr_ = std::make_unique<std::future<void>>();
			*future_ptr_ = std::async(std::launch::async, [this] { detectFeatures(cascade_); });
		}
	}

	std::unique_lock<std::mutex> lock(face_mutex_);

	std::vector<libcamera::Rectangle> temprect;
	std::transform(faces_.begin(), faces_.end(), std::back_inserter(temprect),
				   [](Rect &r) { return libcamera::Rectangle(r.x, r.y, r.width, r.height); });
	completed_request->post_process_metadata.Set("detected_faces", temprect);

	if (draw_features_)
	{
		libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[full_stream_])[0];
		uint8_t *ptr = (uint8_t *)buffer.data();
		Mat image(full_stream_info_.height, full_stream_info_.width, CV_8U, ptr, full_stream_info_.stride);
		drawFeatures(image);
		if (draw_lines_) drawLines(image);

		int font = FONT_HERSHEY_SIMPLEX;
		int baseline = 0;
		Size size = getTextSize(text_, font, adjusted_scale_, adjusted_thickness_, &baseline);

		// Can't find a handy "draw rectangle with alpha" function...
		for (int y = 0; y < size.height + baseline; y++, ptr += full_stream_info_.stride)
		{
			for (int x = 0; x < size.width; x++)
				ptr[x] = bg_ * alpha_ + (1 - alpha_) * ptr[x];
		}
		putText(image, text_, Point(0, size.height), font, adjusted_scale_, fg_, adjusted_thickness_, 0);
	}


	if (adaptive_)
	{
		std::unique_lock<std::mutex> lock(adaptive_mutex_);   

		// for (auto &face : faces_)
		// 	std::cout << face.x << " " << face.y << " " << face.width  << " " <<  face.height << std::endl;

		std::vector <std::map<std::string, pair_k>> face_coords;
		face_coords = calculate_face_corners(faces_);
		check_borders(face_coords, full_stream_info_.width, full_stream_info_.height);

		// for (auto& fc: face_coords)
		// 	for (auto const& [key, val]: fc)
		// 		std::cout << key << " " << val.first << " " << val.second << " " << std::endl;

		text_ = "";
		std::set<int> segments;
		for (auto& face: face_coords){
			segments = find_matching_segments(face, segment_coords_);
			for (auto& s: segments)
				text_ += std::to_string(s) + " "; 
		}
	}

	i2c_.makeFlash();

	return false;
}

void AdaptiveFaceDetectCvStage::detectFeatures(CascadeClassifier &cascade)
{
	equalizeHist(image_, image_);

	std::vector<Rect> temp_faces;
	cascade.detectMultiScale(image_, temp_faces, scaling_factor_, min_neighbors_, CASCADE_SCALE_IMAGE,
							 Size(min_size_, min_size_), Size(max_size_, max_size_));

	// Scale faces back to the size and location in the full res image.
	double scale_x = full_stream_info_.width / (double)low_res_info_.width;
	double scale_y = full_stream_info_.height / (double)low_res_info_.height;
	for (auto &face : temp_faces)
	{
		face.x *= scale_x;
		face.y *= scale_y;
		face.width *= scale_x;
		face.height *= scale_y;
	}
	std::unique_lock<std::mutex> lock(face_mutex_);
	faces_ = std::move(temp_faces);
}

std::vector <std::map<std::string, pair_k>> AdaptiveFaceDetectCvStage::calculate_face_corners(std::vector<cv::Rect>& faces)
{
    std::map <std::string, pair_k> coords_map; 
    std::vector <std::map<std::string, pair_k>> face_coords;
    for (auto &face : faces){
        coords_map["c0"] = std::make_pair(face.x, face.y);
        coords_map["c1"] = std::make_pair(face.x + face.width, face.y);
        coords_map["c2"] = std::make_pair(face.x, face.y + face.height);
        coords_map["c3"] = std::make_pair(face.x + face.width, face.y +  face.height);
        face_coords.push_back(coords_map);
    }
    return face_coords;
}


void AdaptiveFaceDetectCvStage::check_borders(std::vector <std::map<std::string, pair_k>>&  face_coords, int width, int height){
	for (auto& f: face_coords){
        for (auto& [key, val]: f){

            if (val.first >= width){
                 val.first = width;
                 std::cout << val.first << std::endl;
            }
               
            if (val.second >= height){
                val.second = height;
                std::cout << val.second << std::endl;
            }
        }
    }
}


std::set<int> AdaptiveFaceDetectCvStage::find_matching_segments(std::map<std::string, pair_k>& face,  
																std::map<int, std::unique_ptr<SegmentCoordinates>>& coords){
    std::set<int> s;
    for (auto const& [key, val]: face)
    {
        // std::cout << key << " : " << val.first << " " << val.second << std::endl;

        if (val.first < coords[0] -> x1_ && val.second < coords[0] -> y1_) //0
        {
            s.insert(0); 
        }
        else if (val.first > coords[0] -> x1_ && val.second < coords[0] -> y1_) { // 1 -> 2
            if (val.first < coords[1] -> x1_)
                s.insert(1);
            else
                s.insert(2);
        }
        else if(val.first < coords[0] -> x1_ &&  val.second > coords[0] -> y1_) { // 3 -> 6
            if (val.second < coords[3] -> y1_)
                s.insert(3);       
            else
                s.insert(6);         
        }
        else if (val.first < coords[4] -> x1_ && val.second < coords[4] -> y1_){ // 4
                s.insert(4);
        }
        else if (val.first > coords[4] -> x1_ && val.second < coords[4] -> y1_){ // 5
                s.insert(5);
        }
        else if (val.first < coords[4] -> x1_ && val.second > coords[4] -> y1_){ // 7
                s.insert(7);
        }
        else
                s.insert(8); // 8
        
    }
    return s;
} 


void AdaptiveFaceDetectCvStage::drawLines(Mat &img){
	int stride_x = full_stream_info_.width / seg_x_;
	int stride_y = full_stream_info_.height / seg_y_;

	line(img, Point(stride_x, 0), Point(stride_x, 3 * stride_y), Scalar(255, 0, 0), 2, LINE_8);
	line(img, Point(2 * stride_x, 0), Point(2 * stride_x, 3 * stride_y), Scalar(255, 0, 0), 2, LINE_8);
	line(img, Point(0, stride_y), Point(3 * stride_x, stride_y), Scalar(255, 0, 0), 2, LINE_8);
	line(img, Point(0, 2 * stride_y), Point(3 * stride_x, 2 * stride_y), Scalar(255, 0, 0), 2, LINE_8);

}

void AdaptiveFaceDetectCvStage::drawFeatures(Mat &img)
{
	const static Scalar colors[] = {
		Scalar(255, 0, 0),	 Scalar(255, 128, 0), Scalar(255, 255, 0), Scalar(0, 255, 0),
		Scalar(0, 128, 255), Scalar(0, 255, 255), Scalar(0, 0, 255),   Scalar(255, 0, 255)
	};

	for (size_t i = 0; i < faces_.size(); i++)
	{
		Rect r = faces_[i];
		Point center;
		Scalar color = colors[i % 8];
		//int radius;
		//double aspect_ratio = (double)r.width / r.height;

		rectangle(img, Point(cvRound(r.x), cvRound(r.y)),
				Point(cvRound(r.x + r.width - 1), cvRound(r.y + r.height - 1)), color, 3, 8, 0);

		/*
		if (0.75 < aspect_ratio && aspect_ratio < 1.3)
		{
			center.x = cvRound(r.x + r.width * 0.5);
			center.y = cvRound(r.y + r.height * 0.5);
			radius = cvRound((r.width + r.height) * 0.25);
			circle(img, center, radius, color, 3, 8, 0);
		}
		else
			rectangle(img, Point(cvRound(r.x), cvRound(r.y)),
					  Point(cvRound(r.x + r.width - 1), cvRound(r.y + r.height - 1)), color, 3, 8, 0);
		*/
	}
}

void AdaptiveFaceDetectCvStage::Stop()
{	
	if (future_ptr_)
		future_ptr_->wait();
	
}

void AdaptiveFaceDetectCvStage::Teardown(){
	i2c_.clearDacs();
}


static PostProcessingStage *Create(LibcameraApp *app)
{
	return new AdaptiveFaceDetectCvStage(app);
}

static RegisterStage reg(NAME, &Create);
