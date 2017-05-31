#include "stdafx.h"
#include "FrameSync.h"
#define push_queue(q,d) 	q.push(d);\
							if(q.size() > 10) \
								{ q.pop(); }


void FrameSync::sendIfAllReady()
{
	if (!is_need_send_)
	{
		return;
	}
	int min_delta_time = 0;
	/*
	int cur_delta[3];
	FrameData min_idx[3];
	FrameData cur_idx[3];
	mutex_is_send_.lock();
	for (auto& k:kinect_list_)
	{
		cur_idx[0] = k;
		for (auto& r1 : realsense1_list_)
		{

			cur_idx[1] = r1;
			for (auto& r2 : realsense2_list_)
			{

				cur_idx[2] = r2;
				cur_delta[0] = abs(k.timestamp - r1.timestamp);
				cur_delta[1] = abs(k.timestamp - r2.timestamp);
				cur_delta[2] = abs(r1.timestamp - r1.timestamp);

				__int64 cur_delta_time = cur_delta[0] + cur_delta[1] + cur_delta[2];
				if (cur_delta[0] < sync_max_interval &&
					cur_delta[1] < sync_max_interval &&
					cur_delta[2] < sync_max_interval &&
					cur_delta_time < min_delta_time  &&
					cur_delta_time > 0
					)
				{

					min_delta_time = cur_delta_time;
					min_idx[0] = cur_idx[0];
					min_idx[1] = cur_idx[1];
					min_idx[2] = cur_idx[2];
				}
			}
		}
	}
	*/
	SyncFrameData sync_data;
	if (min_delta_time < sync_max_interval)
	{
		//sync_data.kinect_frame = min_idx[0];
		sync_data.kinect_frame = kinect_list_.front();
		kinect_list_.clear();
		/*
		sync_data.realsense1_frame = min_idx[1];
		sync_data.realsense2_frame = min_idx[2];
		realsense1_list_.clear();
		realsense2_list_.clear();
		std::cout << "sync delta time:" << min_delta_time<<std::endl;
		*/
		mutex_is_send_.unlock();
	}
	else
	{
		mutex_is_send_.unlock();
	}
	if (min_delta_time < sync_max_interval)
	{
		callback_(sync_data);
	}
	
}

void FrameSync::pushData(std::list<FrameData>& q, FrameData& d)
{
	std::lock_guard<std::mutex> lock(mutex_is_send_);
	if (!is_need_send_)
	{
		q.clear();
		return;
	}

	q.insert(q.end(), d);
	if(q.size() > 10) 
	{ 
		q.erase(q.begin()); 
	}


}

FrameSync::FrameSync()
{
	is_need_send_ = true;
}


FrameSync::~FrameSync()
{
	stopSend();
}

void FrameSync::pushKinectData(FrameData& data)
{
	if (!is_need_send_)
	{
		return;
	}
	pushData(kinect_list_,data);
	sendIfAllReady();
}

void FrameSync::pushRealSense1Data(FrameData& data)
{
	if (!is_need_send_)
	{
		return;
	}
	
	pushData(realsense1_list_,data);
	sendIfAllReady();
}

void FrameSync::pushRealSense2Data(FrameData& data)
{
	if (!is_need_send_)
	{
		return;
	}
	
	pushData(realsense2_list_,data);
	sendIfAllReady();
}

void FrameSync::setSyncDataCallback(FrameSyncCallBack data)
{
	callback_ = data;
}

void FrameSync::stopSend()
{
	is_need_send_ = false;
}

void FrameSync::startSend()
{
	is_need_send_ = true;
}

bool FrameSync::isNeedSend()
{
	return is_need_send_;
}

FrameSync& getFrameSync()
{
	return FrameSync::getInstance();
}
