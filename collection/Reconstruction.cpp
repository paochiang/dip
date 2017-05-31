#include "stdafx.h"
#include "Reconstruction.h"
#include "FrameSync.h"
#include <ctime>
#include <opencv/cv.hpp>
#include <opencv2/highgui.hpp>
#include <dip/io/objfile.h>
#include <fstream>
int count = 0; //used for output mesh.obj 
Reconstruction::Reconstruction() : modelling(NULL)
{
	CurrentDepthMap = new dip::Depth[WIDTH_*HEIGHT_];
	memset(CurrentDepthMap, 0, sizeof(dip::Depth) * WIDTH_ * HEIGHT_);
	CurrentNormalMap = new dip::Color[WIDTH_*HEIGHT_];
	memset(CurrentNormalMap, 0, sizeof(dip::Color) * WIDTH_ * HEIGHT_);
	LastValidDepthMap = new dip::Depth[WIDTH_*HEIGHT_];
	memset(LastValidDepthMap, 0, sizeof(dip::Depth) * WIDTH_ * HEIGHT_);
	start_t = clock();
}


Reconstruction::~Reconstruction()
{
	if (modelling != NULL) delete modelling;
	if (CurrentDepthMap != NULL)  delete[] CurrentDepthMap;
	if (CurrentNormalMap != NULL) delete[] CurrentNormalMap;
	if (LastValidDepthMap != NULL) delete[] LastValidDepthMap;
}

void Reconstruction::updateFrameData(SyncFrameData frame)
{

	start_t = clock();
	// Stop data qcquisition
	getFrameSync().stopSend();
	// Validate input data
	if (frame.kinect_frame.depth.empty())
	{
		getFrameSync().startSend();
		return;
	}	
	// Copy data (do a left-right flip if necessary)
	if (frame.kinect_frame.depth.isContinuous())
	{
		memcpy(CurrentDepthMap, frame.kinect_frame.depth.data, sizeof(unsigned short)*WIDTH_*HEIGHT_);
	}
	else
	{
		for (int r = 0; r < frame.kinect_frame.depth.rows; r++)
			for (int c = 0; c < frame.kinect_frame.depth.cols; c++)
				CurrentDepthMap[r * WIDTH_ + c] = frame.kinect_frame.depth.at<unsigned short>(r, c);
	}
	CurrentTimestamp = frame.kinect_frame.timestamp;
	// Restart data acquisition
	//getFrameSync().startSend();

	// Set up the modeller if it is empty
	if (modelling == NULL)
	{
		if ((modelling = new dip::FaceModeling(WIDTH_, HEIGHT_, FX_, FY_, CX_, CY_, "./shape_predictor_68_face_landmarks.dat")) == NULL)
		{
			std::cerr << "updateFrameData: failed to set up FaceModeling." << std::endl;
			getFrameSync().startSend();
			return;
		}
	}
	// Hold returned values
	std::vector<Eigen::Matrix4f> TransformFrameToGlobal;
	bool RetrieveMeshNow;
	int RetVal = modelling->Run(CurrentDepthMap, CurrentNormalMap, frame.kinect_frame.c2d_map,
		TransformFrameToGlobal, RetrieveMeshNow);
	if (RetVal == 0) // tracking is good
	{
		TimestampList.push_back(CurrentTimestamp);
		memcpy(LastValidDepthMap, CurrentDepthMap, sizeof(dip::Depth)*WIDTH_*HEIGHT_);
		TransformationList.push_back(TransformFrameToGlobal);
	}

	if (RetrieveMeshNow) // Reconstruction is finished
	{
		dip::Mesh CurrentMesh;
		modelling->Model(&CurrentMesh);
		//std::stringstream ss;
		//ss << count << std::endl;
		//std::string out;
		//ss >> out;
		//std::string outfileName = "H:/mesh" + out + ".obj";
		//dip::OBJFile obj_file(outfileName.c_str(), dip::CREATE_OBJ);
		//if (obj_file.enabled()) {
		//	obj_file.Write(&CurrentMesh);
		//}
		//count++;
		// TODO send TimestampList.back() to the data acquisition side to retrieve the corresponding RGB images
		// TODO send LastValidDepth, CurrentMesh and TransformationList.back() to Lu Aiyu
		TimestampList.clear();
		memset(LastValidDepthMap, 0, sizeof(dip::Depth)*WIDTH_*HEIGHT_);
		TransformationList.clear();
	}
	end_t = clock();
	std::cout << "time cost:" << end_t - start_t << std::endl;
	getFrameSync().startSend();
	
}

void Reconstruction::setCompleteMsgCallback(CompleteMsgCallBack msg_callback)
{
	reconstuct_complete_msg_ = msg_callback;
}
