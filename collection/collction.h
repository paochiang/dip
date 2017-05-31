/**************************************************************************
Copyright	: Cloudream Inc All Rights Reserved
Author		: xuwj
Date        : 2017/05/19 11:48
Description	: 数据采集基类
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
	// Remark   : 初始化采集设备
	// Date     : 2017/05/19 13:09
	// Returns  : bool
	//**********************************************************
	virtual bool init(int idx) = 0;

	//**********************************************************
	// Method   : setCallBack
	// Remark   : 设置接收采集数据的回调函数
	// Date     : 2017/05/19 13:09
	// Returns  : void
	//**********************************************************
	virtual void setCallBack(CollctionCallBack callback) = 0;

	//**********************************************************
	// Method   : start
	// Remark   : 开始采集
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual bool start() = 0;

	//**********************************************************
	// Method   : stop
	// Remark   : 停止采集
	// Date     : 2017/05/19 13:10
	// Returns  : void
	//**********************************************************
	virtual void stop() = 0;
};
#endif //COLLCTION_H_

