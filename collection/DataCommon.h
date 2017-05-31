/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 14:12
Description	: һЩ�������ݶ���
**************************************************************************/
#ifndef DATACOMMON_H_
#define DATACOMMON_H_
#include "opencv2/opencv.hpp"
#include <functional>

const int fps = 3;
const int sync_max_interval = 60;

struct FrameData
{
	cv::Mat depth;//���ͼ
	cv::Mat color;//��ɫͼ
	cv::Mat c2d_map;//��ɫͼ�����ͼӳ��
	__int64 timestamp;//֡ʱ���
};

struct SyncFrameData
{
	FrameData kinect_frame;
	FrameData realsense1_frame;
	FrameData realsense2_frame;
};

typedef std::function<void(FrameData&)> CollctionCallBack;//�豸�ɼ����ݵĻص�����
typedef std::function<void(SyncFrameData&)> FrameSyncCallBack;//��ȡ֡ͬ�����ݵĻص�����
typedef std::function<void()> CompleteMsgCallBack;//�ع���ɺ�Ļص�����
#endif //DATACOMMON_H_
