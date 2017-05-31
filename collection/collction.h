/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 11:48
Description	: ���ݲɼ�����
**************************************************************************/
#ifndef COLLCTION_H_
#define COLLCTION_H_
#include <opencv2/opencv.hpp>
#include "DataCommon.h"

class ICollction
{
public:
	//**********************************************************
	// Method   : init
	// Remark   : ��ʼ���ɼ��豸
	// Date     : 2017/05/19 13:09
	// Returns  : bool
	//**********************************************************
	virtual bool init(int idx) = 0;

	//**********************************************************
	// Method   : setCallBack
	// Remark   : ���ý��ղɼ����ݵĻص�����
	// Date     : 2017/05/19 13:09
	// Returns  : void
	//**********************************************************
	virtual void setCallBack(CollctionCallBack callback) = 0;

	//**********************************************************
	// Method   : start
	// Remark   : ��ʼ�ɼ�
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual bool start() = 0;

	//**********************************************************
	// Method   : stop
	// Remark   : ֹͣ�ɼ�
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual void stop() = 0;
};
#endif //COLLCTION_H_

