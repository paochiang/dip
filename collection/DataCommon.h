/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 14:12
Description	: 一些共用数据定义
**************************************************************************/
#ifndef DATACOMMON_H_
#define DATACOMMON_H_
#include "opencv2/opencv.hpp"
#include <functional>

const int fps = 3;
const int sync_max_interval = 60;

struct FrameData
{
	cv::Mat depth;//深度图
	cv::Mat color;//彩色图
	cv::Mat c2d_map;//彩色图到深度图映射
	__int64 timestamp;//帧时间戳
};

struct SyncFrameData
{
	FrameData kinect_frame;
	FrameData realsense1_frame;
	FrameData realsense2_frame;
};

typedef std::function<void(FrameData&)> CollctionCallBack;//设备采集数据的回调函数
typedef std::function<void(SyncFrameData&)> FrameSyncCallBack;//获取帧同步数据的回调函数
typedef std::function<void()> CompleteMsgCallBack;//重构完成后的回调函数
#endif //DATACOMMON_H_
