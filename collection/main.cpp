// collection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "KinectCollection.h"
#include "RealSenseCollection.h"
#include "FrameSync.h"
#include "Reconstruction.h"
#include <functional>
#include <windows.h>

#define SINGLE_KINECT

int main()
{
    // ÆÁ±Î¹Ø±Õ°´Å¥
    DeleteMenu(GetSystemMenu(GetConsoleWindow(), FALSE), SC_CLOSE, MF_BYCOMMAND);
    DrawMenuBar(GetConsoleWindow());

	Reconstruction recon;
	ICollction* kinect = new KinectCollection;
#ifndef SINGLE_KINECT
	ICollction* realsense1 = new RealSenseCollection;
	ICollction* realsense2 = new RealSenseCollection;
#endif
	kinect->setCallBack(std::bind(&FrameSync::pushKinectData,&getFrameSync(),std::tr1::placeholders::_1));
#ifndef SINGLE_KINECT
	realsense1->setCallBack(std::bind(&FrameSync::pushRealSense1Data,&getFrameSync(),std::tr1::placeholders::_1));
	realsense2->setCallBack(std::bind(&FrameSync::pushRealSense2Data,&getFrameSync(),std::tr1::placeholders::_1));
#endif
	getFrameSync().setSyncDataCallback(std::bind(&Reconstruction::updateFrameData,&recon,std::tr1::placeholders::_1));
	recon.setCompleteMsgCallback([]() {std::cout << "reconstruntion complete" << std::endl; });

	kinect->init(0);
#ifndef SINGLE_KINECT
	realsense1->init(0);
	realsense2->init(1);
#endif
	kinect->start();
#ifndef SINGLE_KINECT
	realsense1->start();
	realsense2->start();
#endif
	while (getchar() != 'q') {};

	kinect->stop();
#ifndef SINGLE_KINECT
	realsense1->stop();
	realsense1->stop();
#endif	
	delete kinect;
#ifndef SINGLE_KINECT
	delete realsense1;
	delete realsense2;
#endif	
	std::cout << "end" << std::endl;
    return 0;
}

