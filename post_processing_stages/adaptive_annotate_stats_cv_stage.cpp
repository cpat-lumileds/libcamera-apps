/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Lumileds Netherlands B.V.
 *
 * adaptive_annotate_stats_cv_stage.cpp - add text annotation to image
 */

// The text string can include the % directives supported by FrameInfo.

#include <libcamera/stream.h>

#include "core/frame_info.hpp"
#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class AdaptiveAnnotateStatsCvStage : public PostProcessingStage
{
public:
	AdaptiveAnnotateStatsCvStage(LibcameraApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	StreamInfo info_;
	std::string text_;
	int fg_;
	int bg_;
	double scale_;
	int thickness_;
	double alpha_;
	double adjusted_scale_;
	int adjusted_thickness_;
};

#define NAME "adaptive_annotate_stats_cv"

char const *AdaptiveAnnotateStatsCvStage::Name() const
{
	return NAME;
}

void AdaptiveAnnotateStatsCvStage::Read(boost::property_tree::ptree const &params)
{
	text_ = params.get<std::string>("text");
	fg_ = params.get<int>("fg", 255);
	bg_ = params.get<int>("bg", 0);
	scale_ = params.get<double>("scale", 1.0);
	thickness_ = params.get<int>("thickness", 2);
	alpha_ = params.get<double>("alpha", 0.5);
}

void AdaptiveAnnotateStatsCvStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("AnnotateCvStage: only YUV420 format supported");
	info_ = app_->GetStreamInfo(stream_);

	// Adjust the scale and thickness according to the image size, so that the relative
	// size is preserved across different camera modes. Note that the thickness can get
	// rather harshly quantised, not much we can do about that.
	adjusted_scale_ = scale_ * info_.width / 1200;
	adjusted_thickness_ = std::max(thickness_ * info_.width / 700, 1u);
}

bool AdaptiveAnnotateStatsCvStage::Process(CompletedRequestPtr &completed_request)
{
	libcamera::Span<uint8_t> buffer = app_->Mmap(completed_request->buffers[stream_])[0];
	FrameInfo info(completed_request->metadata);
	info.sequence = completed_request->sequence;

	// Other post-processing stages can supply metadata to update the text.
	completed_request->post_process_metadata.Get("annotate.text", text_);
	std::string text = info.ToString(text_);

	float avg = 0.0;
	completed_request->post_process_metadata.Get("adaptive.results", avg);
	std::cout << avg << std::endl;

	uint8_t *ptr = (uint8_t *)buffer.data();
	Mat im(info_.height, info_.width, CV_8U, ptr, info_.stride);
	int font = FONT_HERSHEY_SIMPLEX;

	int baseline = 0;
	Size size = getTextSize(text, font, adjusted_scale_, adjusted_thickness_, &baseline);

	// Can't find a handy "draw rectangle with alpha" function...
	for (int y = 0; y < size.height + baseline; y++, ptr += info_.stride)
	{
		for (int x = 0; x < size.width; x++)
			ptr[x] = bg_ * alpha_ + (1 - alpha_) * ptr[x];
	}
	putText(im, text, Point(0, size.height), font, adjusted_scale_, fg_, adjusted_thickness_, 0);

	return false;
}

static PostProcessingStage *Create(LibcameraApp *app)
{
	return new AdaptiveAnnotateStatsCvStage(app);
}

static RegisterStage reg(NAME, &Create);
