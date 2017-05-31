#include "stdafx.h"
#include "RealSenseCollection.h"
#include <iostream>
#include <algorithm>
#include "FrameSync.h"
#include <windows.h>
using namespace std;
using namespace cv;

RealSenseCollection::RealSenseCollection()
{

}


RealSenseCollection::~RealSenseCollection()
{
}

bool RealSenseCollection::init(int idx)
{
	device_idx_ = idx;
	is_collction_ = false;
	depth_width_ = 640;
	depth_hight_ = 480;

	color_width_ = 1920;
	color_hight_ = 1080;
	
	depth_mat_ = cv::Mat::zeros(depth_hight_, depth_width_, CV_8UC1);
	color_mat_ = cv::Mat::zeros(color_hight_, color_width_, CV_8UC3);
	c2d_map_mat_ = cv::Mat::zeros(color_hight_, color_width_, CV_16UC2);

	if (ctx.get_device_count() == 0)
	{
		std::cout<< "No device detected. Is it plugged in?"<<std::endl;
		return false;
	}
    
    // Enumerate all devices
    std::vector<rs::device *> devices;
    for(int i=0; i<ctx.get_device_count(); ++i)
    {
        devices.push_back(ctx.get_device(i));
    }
	sort(devices.begin(), devices.end(), [](rs::device * l, rs::device *r) {return strcmp(l->get_serial() , r->get_serial()) < 0; });
	dev = nullptr;
	if (device_idx_ >=0 && device_idx_ < devices.size())
	{
		dev = devices[device_idx_];
        std::cout << "Starting " << dev->get_serial() << "... ";
        dev->enable_stream(rs::stream::depth, depth_width_,depth_hight_,rs::format::z16,30);
        dev->enable_stream(rs::stream::color, color_width_,color_hight_,rs::format::bgr8,30);
        dev->start();
		return true;
	}

	return false;
}

void RealSenseCollection::setCallBack(CollctionCallBack callback)
{
	collction_cb_ = callback;
}

bool RealSenseCollection::start()
{
	if (!is_collction_)
	{
		is_collction_ = true;
		std::thread collction_thread(&RealSenseCollection::collctionThread,this);
		collction_thread.detach();
		return true;
	}
	return false;
}

void RealSenseCollection::stop()
{
	is_collction_ = false;
	std::lock_guard<std::mutex> lock(on_collction_mutex_);
}

void RealSenseCollection::collctionThread()
{
	std::lock_guard<std::mutex> lock(on_collction_mutex_);

	__int64 last_tick = GetTickCount();
	while (is_collction_)
	{

		__int64 cur_time = GetTickCount();
		if (cur_time- last_tick >=fps && getFrameSync().isNeedSend())
		{
			dev->poll_for_frames();
			__int64 time = GetTickCount();
			const auto c = dev->get_stream_intrinsics(rs::stream::color), d = dev->get_stream_intrinsics(rs::stream::depth);
			const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth));
			const cv::Vec3b * color_frame = reinterpret_cast<const cv::Vec3b *>(dev->get_frame_data(rs::stream::color));
			FrameData frame;
			frame.color = cv::Mat(color_hight_, color_width_, CV_8UC3, (void*)color_frame).clone();
			frame.depth = cv::Mat(depth_hight_, depth_width_, CV_16UC1, (void*)depth_frame).clone();
			frame.timestamp = time;
			collction_cb_(frame);
			last_tick = frame.timestamp;
		}
	}
}

