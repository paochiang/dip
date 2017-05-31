/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 13:16
Description	: kinect���ݲɼ�
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
	// Remark   : ��ʼ���ɼ��豸
	// Date     : 2017/05/19 13:09
	// Returns  : bool
	//**********************************************************
	virtual bool init(int idx);

	//**********************************************************
	// Method   : setCallBack
	// Remark   : ���ý��ղɼ����ݵĻص�����
	// Date     : 2017/05/19 13:09
	// Returns  : void
	//**********************************************************
	virtual void setCallBack(CollctionCallBack callback);

	//**********************************************************
	// Method   : start
	// Remark   : ��ʼ�ɼ�
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual bool start();

	//**********************************************************
	// Method   : stop
	// Remark   : ֹͣ�ɼ�
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

    int                           depth_width_;			//���ͼ���
    int                           depth_height_;		//���ͼ���

	static const int              color_width_  = 1920;
    static const int              color_height_ = 1080;
    // sensor�򿪱�־λ
    bool                          is_collction_;

    std::vector<unsigned char>    color_buffer_; //�����ɫͼ��Ļ���
	cv::Mat                       color_mat_;
	cv::Mat                       depth_mat_;
	cv::Mat                       map_mat_;
	ColorSpacePoint*              colorSpace;


};


#endif //KINECTCOLLECTION_H_
