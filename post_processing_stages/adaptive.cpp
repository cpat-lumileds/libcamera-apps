/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Lumileds Netherlands B.V.
 *
 * adaptive.cpp 
 * 
 *  Read stats  
 *  Compute currents setting of LED 
 *  Set currents
 * 
 */


#include <libcamera/stream.h>
#include <vector>
#include <iterator>

#include "core/libcamera_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using Stream = libcamera::Stream;

using std::cout;
using std::endl;

class Adaptive : public PostProcessingStage
{
    public:
        Adaptive(LibcameraApp *app) : PostProcessingStage(app) {}
        char const *Name() const override;
        void Read(boost::property_tree::ptree const &params) override;
        void Configure() override;
        bool Process(CompletedRequestPtr &completed_request) override;
    private:
        std::vector<libcamera::Rectangle> faces_;
};

#define NAME "adaptive"

char const *Adaptive::Name() const
{
    return NAME;
}

void Adaptive::Read(boost::property_tree::ptree const &params) 
{
}

void Adaptive::Configure() {}

bool Adaptive::Process(CompletedRequestPtr &completed_request)
{
    completed_request->post_process_metadata.Get("detected_faces", faces_);
    //TODO: do it in another asynch process
    //data needs to be passed efficiently 
    // std::vector<libcamera::Rectangle>::iterator it;
    // for (it = faces_.begin(); it != faces_.end(); it++)
    // {   
    //    cout << it -> x << " " << it -> y << endl;
    // }
    return false;
} 


static PostProcessingStage *Create(LibcameraApp *app)
{
    return new Adaptive(app);
}

static RegisterStage reg(NAME, &Create);	
