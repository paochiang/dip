#include "JudgeFrontFace.h"
#include <fstream>

using namespace cv;
using namespace std;
using namespace dlib;

//void main()
//{
//	// 1.ģ�ͳ�ʼ��
//	frontal_face_detector detector = get_frontal_face_detector();
//	shape_predictor sp;
//	deserialize("shape_predictor_68_face_landmarks.dat") >> sp;
//	//2. ������⡢�ؼ����⼰�����б�
//	cv::Mat depthMat = imread("D:\\Projects\\3DFace\\3DFaceRecognition\\FrontFaceD\\samples\\depth_raw1.png", -1);
//	cv::Mat colordepthMat = imread("D:\\Projects\\3DFace\\3DFaceRecognition\\FrontFaceD\\samples\\color_align_depth1.png",1);
//	JudgeFrontFace(depthMat, colordepthMat, detector, sp);
//}

BOOL JudgeFrontFace(const dip::Depth* depthshort, cv::Mat color, frontal_face_detector detector, shape_predictor sp)
{
	//1. ����depthѰ����ѷָ����������������Ԥ����
	dip::HeadSegmenter bs;
	cv::Rect _faceRect = bs.bodyDetect(50, 1100, 20, 512, 424, depthshort);

	//2. �б���ȷָ��������Ƿ����������ж��Ƿ�������
	if (_faceRect != cv::Rect(-1, -1, -1, -1))
	{
		//���������򣬽����������
		//cv::Mat color_Crop = color(_faceRect);
		cv_image<rgb_pixel> img(color(_faceRect));
		std::vector<dlib::rectangle> dets = detector(img);
		if (dets.size() > 0)
		{
			dlib::full_object_detection shape = sp(img, dets[0]);
			std::vector<cv::Point2i> landmark;
			for (int i = 0; i < 68; i++)
			{
				landmark.push_back(cv::Point2i(shape.part(i).x(), shape.part(i).y()));
				//circle(color_Crop, cv::Point(shape.part(i).x(), shape.part(i).y()), 1, cv::Scalar(0, 0, 255));
			}
			BOOL FrontFaceFlag = DetectProfile(landmark, 0.618, 1.618);	//����������1����������0
			return FrontFaceFlag;
		}
		else
		{
			return FALSE;
		}
	}
}

BOOL DetectProfile(std::vector<cv::Point2i> points, float val_min, float val_max)	//�б��Ƿ��ǲ���
{
	float centerX = (points[27].x + points[28].x + points[29].x + points[30].x) / 4.0;

	float suml = 0, sumr = 0;
	for (auto index = 0; index < 8; index++)
	{
		suml += points[index].x;
	}
	for (auto index = 9; index < 17; index++)
	{
		sumr += points[index].x;
	}
	suml = suml / 8;
	sumr = sumr / 8;

	//float value1 = abs(abs(suml - centerX) - abs(sumr - centerX));
	float value2 = abs(suml - centerX) / abs(sumr - centerX);
	//std::cout << "value:" << value2 << std::endl;
	if (value2 > val_max || value2 < val_min)
		return FALSE;
	else
		return TRUE;
}
