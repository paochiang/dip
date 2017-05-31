/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 11:45
Description	: realsense数据采集
**************************************************************************/
#ifndef REALSENSECOLLECTION_H_
#define REALSENSECOLLECTION_H_
#include <thread>
#include <mutex>
#include "collction.h"
#include "librealsense/rs.hpp"
class RealSenseCollection:public ICollction
{
public:
	RealSenseCollection();
	~RealSenseCollection();

	//**********************************************************
	// Method   : init
	// Remark   : 初始化采集设备
	// Date     : 2017/05/19 13:09
	// Returns  : bool
	//**********************************************************
	virtual bool init(int idx);

	//**********************************************************
	// Method   : setCallBack
	// Remark   : 设置接收采集数据的回调函数
	// Date     : 2017/05/19 13:09
	// Returns  : void
	//**********************************************************
	virtual void setCallBack(CollctionCallBack callback);

	//**********************************************************
	// Method   : start
	// Remark   : 开始采集
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual bool start();

	//**********************************************************
	// Method   : stop
	// Remark   : 停止采集
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual void stop();
private:
	void collctionThread();
private:
	CollctionCallBack collction_cb_;
	cv::Mat depth_mat_;
	cv::Mat color_mat_;
	cv::Mat c2d_map_mat_;
	bool is_collction_;
	std::mutex on_collction_mutex_;
	int depth_width_;
	int depth_hight_;

	int color_width_;
	int color_hight_;

	int device_idx_;
	FrameData framedata_;
	rs::device *dev;
    rs::context ctx;
};
#endif //REALSENSECOLLECTION_H_

