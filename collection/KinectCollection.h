/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 13:16
Description	: kinect数据采集
**************************************************************************/
#ifndef KINECTCOLLECTION_H_
#define KINECTCOLLECTION_H_
#include <Kinect.h>
#include <mutex>
#include "collction.h"
class KinectCollection :public ICollction
{
public:
	KinectCollection();
	~KinectCollection();

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
	bool initSensor();
	bool uninitSensor();
	void updateDepth();
	void updateColor();
	void updateMap();
	template<class T>
	HRESULT getFrameSize(T frame,IFrameDescription** ppFrameDescription, int& width, int& height)
	{
		HRESULT hr = frame->get_FrameDescription(ppFrameDescription); 
        if (SUCCEEDED(hr)) { hr = (*ppFrameDescription)->get_Width(&width); }
        if (SUCCEEDED(hr)) { hr = (*ppFrameDescription)->get_Height(&height); }
		return hr;
	}

private:
	CollctionCallBack collction_cb_;
	std::mutex on_collction_mutex_;
    IKinectSensor*                kinect_sensor_;
    ICoordinateMapper*            coordinate_mapper_;
    IColorFrameReader*            color_frame_reader_;
    IDepthFrameReader*            depth_frame_reader_;

    int                           depth_width_;			//深度图像宽
    int                           depth_height_;		//深度图像高

	static const int              color_width_  = 1920;
    static const int              color_height_ = 1080;
    // sensor打开标志位
    bool                          is_collction_;

    std::vector<unsigned char>    color_buffer_; //输出彩色图像的缓存
	cv::Mat                       color_mat_;
	cv::Mat                       depth_mat_;
	cv::Mat                       map_mat_;
	ColorSpacePoint*              colorSpace;


};


#endif //KINECTCOLLECTION_H_
