/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 14:06
Description	: Ö¡Í¬²½
**************************************************************************/
#ifndef FRAMESYNC_H_
#define FRAMESYNC_H_
#include "DataCommon.h"
#include <list>
#include <mutex>
class FrameSync
{
public:
	static FrameSync& getInstance() {
		static FrameSync frame_sync;
		return frame_sync;
	}
	~FrameSync();
	
	void pushKinectData(FrameData& data);
	void pushRealSense1Data(FrameData& data);
	void pushRealSense2Data(FrameData& data);
	void setSyncDataCallback(FrameSyncCallBack callback );
	void stopSend();
	void startSend();
	bool isNeedSend();
private:
	void sendIfAllReady();
	void pushData(std::list<FrameData>& q,FrameData& d);
private:
	FrameSync();
	FrameSyncCallBack callback_;
	bool is_need_send_;
	std::list<FrameData> kinect_list_;
	std::list<FrameData> realsense1_list_;
	std::list<FrameData> realsense2_list_;
	std::mutex mutex_is_send_;
};

FrameSync& getFrameSync();

#endif //FRAMESYNC_H_
