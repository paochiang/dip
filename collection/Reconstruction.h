/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 14:35
Description	: �����ع�
**************************************************************************/
#ifndef RECONSTRUCTION_H_
#define RECONSTRUCTION_H_
#include "DataCommon.h"
#include "facemodeling.h"
#include "face_detect/JudgeFrontFace.h"
#include <time.h>
class Reconstruction
{
	// Camera specs
	const int WIDTH_ = 512;
	const int HEIGHT_ = 424;
	const float FX_ = 364.8929f;
	const float FY_ = 364.8929f;
	const float CX_ = (512 - 1) / 2;
	const float CY_ = (424 - 1) / 2;

	// Key data of current frame
	dip::Depth* CurrentDepthMap;
	__int64		CurrentTimestamp;

	std::vector<__int64> TimestampList;
	dip::Depth* LastValidDepthMap;
	// Returned data by modeler
	std::vector<std::vector<Eigen::Matrix4f>> TransformationList;
	// Visualzation
	dip::Color* CurrentNormalMap;
	clock_t start_t;
	clock_t end_t;
public:
	Reconstruction();
	~Reconstruction();
	
	//**********************************************************
	// Method   : updateFrameData
	// Remark   : ���վ���ͬ����֡����,�ع��㷨�ڴ˽�������
	// Date     : 2017/05/19 14:37
	// Returns  : void
	//**********************************************************
	void updateFrameData(SyncFrameData frame);

	//**********************************************************
	// Method   : setCompleteMsgCallback
	// Remark   : �����ع���ɺ����Ϣ֪ͨ,�ع���ɺ����reconstuct_complete_msg_();
	// Date     : 2017/05/19 14:55
	// Returns  : void
	//**********************************************************
	void setCompleteMsgCallback(CompleteMsgCallBack msg_callback);
private:
	CompleteMsgCallBack reconstuct_complete_msg_;
	dip::FaceModeling* modelling;
};

#endif //RECONSTRUCTION_H_
