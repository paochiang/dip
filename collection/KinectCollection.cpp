#include "stdafx.h"
#include <thread>
#include <opencv2/opencv.hpp>
#include "FrameSync.h"
#include "KinectCollection.h"
#include <time.h>
#define	SafeRelease(ptr)		do{ if ( ptr != NULL ){ ptr->Release(); ptr = NULL; } }while(0)
#define	SafeDelete(ptr)		do{ if ( ptr != NULL ){ delete ptr; ptr = NULL; } }while(0)

template<class Interface>
inline void SafeReleaseObj(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}


KinectCollection::KinectCollection()
{
}


KinectCollection::~KinectCollection()
{
}

bool KinectCollection::init(int idx)
{
	is_collction_ = false;
    color_buffer_.resize(color_width_*color_height_* sizeof(RGBQUAD));
	map_mat_ = cv::Mat(depth_height_, depth_width_, CV_8UC3);
	color_mat_ = cv::Mat::zeros(color_height_, color_width_, CV_8UC4);
	return true;
}

void KinectCollection::setCallBack(CollctionCallBack callback)
{
	collction_cb_ = callback;
}

bool KinectCollection::start()
{
	if (!initSensor())
		return false;

	if (!is_collction_)
	{
		is_collction_ = true;
		std::thread collction_thread(&KinectCollection::collctionThread,this);
		collction_thread.detach();
		return true;
	}
	return false;

}

void KinectCollection::stop()
{
	is_collction_ = false;
	std::lock_guard<std::mutex> lock(on_collction_mutex_);
	uninitSensor();
}

void KinectCollection::collctionThread()
{
	clock_t s1, s2;
	std::lock_guard<std::mutex> lock(on_collction_mutex_);
	__int64 last_tick = GetTickCount();
	while (is_collction_ )
	{
		if (GetTickCount() - last_tick >=fps && getFrameSync().isNeedSend())
		{
			//s1 = clock();
			FrameData frame;
			frame.timestamp = GetTickCount();
			updateColor();
			updateDepth();
			updateMap();
			//s2 = clock();
			
			frame.color = color_mat_.clone();
			frame.depth = depth_mat_.clone();
			frame.c2d_map = map_mat_.clone();
			//std::cout << "time cost:" << s2 - s1 << std::endl;
			collction_cb_(frame);
			last_tick = frame.timestamp;
			
	
		}
	}
}

bool KinectCollection::initSensor()
{
	uninitSensor();
    HRESULT hr;

    hr = GetDefaultKinectSensor(&kinect_sensor_);
    if (FAILED(hr))
    {
		return false;
   }

    if (kinect_sensor_)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IColorFrameSource* pColorFrameSource = NULL;
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = kinect_sensor_->Open();

        if (SUCCEEDED(hr))
        {
            hr = kinect_sensor_->get_CoordinateMapper(&coordinate_mapper_);
        }

       if (SUCCEEDED(hr))
        {
            hr = kinect_sensor_->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&color_frame_reader_);
        }

        if (SUCCEEDED(hr))
        {
            hr = kinect_sensor_->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&depth_frame_reader_);
        }


	if (!SUCCEEDED(hr)) {
		return false; }
        SafeReleaseObj(pColorFrameSource);
        SafeReleaseObj(pDepthFrameSource);
    }

    if (!kinect_sensor_ || FAILED(hr))
    {
       return false;
    }


}

bool KinectCollection::uninitSensor()
{
    SafeReleaseObj(color_frame_reader_);

    SafeReleaseObj(depth_frame_reader_);

    SafeReleaseObj(coordinate_mapper_);

   // close the Kinect Sensor
    if (kinect_sensor_)
    {
        kinect_sensor_->Close();
    }

    SafeReleaseObj(kinect_sensor_);
	return true;
}

void KinectCollection::updateDepth()
{
    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = depth_frame_reader_->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        UINT16 *pBuffer = NULL;

        hr = pDepthFrame->get_RelativeTime(&nTime);

		hr = getFrameSize(pDepthFrame,&pFrameDescription,depth_width_,depth_height_);

		UINT nDepthBufferSize = 0;		//缓存深度图像大小
        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pBuffer);            
        }

        if (SUCCEEDED(hr))
        {
            if (depth_mat_.rows != depth_height_ || depth_mat_.cols != depth_width_)
            {
				depth_mat_ = cv::Mat::zeros(depth_height_,depth_width_,CV_16UC1);
            }
            memcpy(depth_mat_.data,pBuffer,nDepthBufferSize*sizeof(UINT16));
        }
        SafeRelease(pFrameDescription);
    }

    SafeRelease(pDepthFrame);
}

void KinectCollection::updateColor()
{
    RGBQUAD *pBuffer = NULL;
    if (!color_frame_reader_)
    {
		return;
    }
    
    IColorFrame* pColorFrame = NULL;

    HRESULT hr = color_frame_reader_->AcquireLatestFrame(&pColorFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        int nWidth = 0;
        int nHeight = 0;
       // ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nBufferSize = 0;       
  //      hr = pColorFrame->get_RelativeTime(&nTime);

  //      IFrameDescription* pFrameDescription = NULL;
		//hr = getFrameSize(pColorFrame,&pFrameDescription,nWidth,nHeight);

        //if (SUCCEEDED(hr))
        //{
        //    hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        //}

        if (SUCCEEDED(hr))
        {
			//auto pDstBuffer = &color_buffer_[0];
   //         pBuffer = (RGBQUAD*)pDstBuffer;
            nBufferSize = color_width_ * color_height_ * sizeof(RGBQUAD);
            hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize
               // , reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);     
				, reinterpret_cast<BYTE*>(color_mat_.data), ColorImageFormat_Bgra);
			//int pixsize = sizeof(RGBQUAD);
			//for (int row = 0;row < color_mat_.rows;row++)
			//{
			//	for (int col = 0;col < color_mat_.cols;col++)
			//	{
			//		auto& dst = color_mat_.at<cv::Vec3b>(row,col);
			//		auto& src = pBuffer[row*color_width_ + col];
			//		dst[0] = src.rgbBlue;
			//		dst[1] = src.rgbGreen;
			//		dst[2] = src.rgbRed;
			//	}
			//}

       }
      //  SafeReleaseObj(pFrameDescription);
    }

    SafeReleaseObj(pColorFrame);

}

void KinectCollection::updateMap()
{
	if (depth_mat_.empty())
		return;
    if (coordinate_mapper_ != NULL)
    {
		UINT16* tmpDepth = new UINT16[depth_width_ * depth_height_];
		for (int y = 0; y < depth_height_; y++)
			for (int x = 0; x < depth_width_; x++)
			{
				int index = y * depth_width_ + x;
				tmpDepth[index] = depth_mat_.at<unsigned short>(y, x);
			}
		colorSpace = new ColorSpacePoint[depth_width_ * depth_height_];
	    HRESULT hr = coordinate_mapper_->MapDepthFrameToColorSpace(depth_width_ * depth_height_	, tmpDepth, depth_width_ * depth_height_, colorSpace);
		if (SUCCEEDED(hr)) {
			map_mat_ = cv::Mat::zeros(depth_height_, depth_width_, CV_8UC3);
			for (int y = 0; y < depth_height_; y++)
				for (int x = 0; x < depth_width_; x++) {
					int index = y * depth_width_ + x;
					ColorSpacePoint p = colorSpace[index];
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity()) {
						int colorX = static_cast<int>(p.X + 0.5f);
						int colorY = static_cast<int>(p.Y + 0.5f);
						if ((colorX >= 0 && colorX < color_width_) && (colorY >= 0 && colorY < color_height_)) {
							map_mat_.at<cv::Vec3b>(y, x) = cv::Vec3b(color_mat_.at<cv::Vec4b>(colorY, colorX)[0], color_mat_.at<cv::Vec4b>(colorY, colorX)[1], color_mat_.at<cv::Vec4b>(colorY, colorX)[2]);
						}
							
					}
				}
		}
		delete[] tmpDepth;
		delete[] colorSpace;
    }
	

}
