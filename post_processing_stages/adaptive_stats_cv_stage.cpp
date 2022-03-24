/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Lumileds Netherlands B.V.
 *
 * adaptive_stats_cv_stage.cpp 
 * 

Some helpful hints on writing your own stages:

Generally, the Process method should not take too long as it will block the imaging pipeline 
and may cause stuttering.
When time-consuming algorithms need to be run, it may be helpful to delegate them to 
another asynchronous thread.

When delegating work to another thread, the way image buffers are handled currently 
means that they will need to be copied.
For some applications, such as image analysis, 
it may be viable to use the "low resolution" image stream rather than full resolution images.

The post-processing framework adds multi-threading parallelism on a per-frame basis. 
This is helpful in improving throughput if you want to run on every single frame. 
Some functions may supply parallelism within each frame (such as OpenCV and TFLite). 
In these cases it would probably be better to serialise the calls so as to suppress 
the per-frame parallelism.

Most streams, and in particular the low resolution stream, have YUV420 format. 
These formats are sometimes not ideal for OpenCV or TFLite so there may sometimes 
need to be a conversion step.

When images need to be altered, doing so in place is much the easiest strategy.

*/

#include <libcamera/stream.h>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"

#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class AdaptiveStatsCvStage : public PostProcessingStage
{
public:
	AdaptiveStatsCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override; 

	void AdjustConfig(std::string const &use_case, StreamConfiguration *config);

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

	void Start();

	void Stop();

	void Teardown();

private:
	float adaptive_results_ = 10;
	Stream *stream_;
	int seg_x_, seg_y_;
};

#define NAME "adaptive_stats_cv"

/*
Return the name of the stage. This is used to match against stages listed in the 
JSON post-processing configuration file.
*/
char const *AdaptiveStatsCvStage::Name() const
{
	return NAME;
}	

/*Called when the camera starts. This method is often not required.*/
void AdaptiveStatsCvStage::Start() {}

/*
This method will read any of the stage’s configuration parameters from the JSON file.
*/
void AdaptiveStatsCvStage::Read(boost::property_tree::ptree const &params) 
{
	seg_x_ = params.get<uint8_t>("seg_x", 3);
	seg_y_ = params.get<uint8_t>("seg_y", 3);
}

/*
This method gives stages a chance to influence the configuration of the camera, 
though it is not often necessary to implement it.
*/
void AdaptiveStatsCvStage::AdjustConfig(std::string const &use_case, StreamConfiguration *config) {}

/*
This is called just after the camera has been configured. 
It is a good moment to check that the stage has access to the streams it needs, 
and it can also allocate any resources that it may require.
*/
void AdaptiveStatsCvStage::Configure()
{
	stream_ = app_->GetMainStream();
}

/*
This method presents completed camera requests for post-processing and is where the
necessary pixel manipulations or image analysis will happen. 

The function returns true if the post-processing framework is not 
to deliver this request on to the application.
*/
bool AdaptiveStatsCvStage::Process(CompletedRequestPtr &completed_request)
{
	//StreamInfo info = app_->GetStreamInfo(stream_);
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	uint32_t *ptr = (uint32_t *)buffer.data();

	/* image processing */

	// Constraints on the stride mean we always have multiple-of-4 bytes.
	float sum = 0.0, avg = 0.0;
	for (unsigned int i = 0; i < buffer.size(); i += 4){
		//*(ptr++) ^= 0xffffffff;
		sum += *(ptr++);
	}
	avg = sum / (buffer.size() / 4);
	adaptive_results_ = avg;
	completed_request->post_process_metadata.Set("adaptive.results", adaptive_results_);

	return false;
}

/*
Called when the camera is stopped. Normally a stage would need to shut down 
any processing that might be running (for example, if it started any asynchronous threads).
*/
void AdaptiveStatsCvStage::Stop() {}

/*Called when the camera configuration is torn down. 
This would typically be used to de-allocate any 
resources that were set up in the Configure method.
*/
void AdaptiveStatsCvStage::Teardown() {}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new AdaptiveStatsCvStage(app);
}

static RegisterStage reg(NAME, &Create);	
