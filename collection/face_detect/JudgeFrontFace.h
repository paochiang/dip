#ifndef JUDGE_FRONT_FACE_H
#define JUDGE_FRONT_FACE_H
#include <opencv2/opencv.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/opencv/cv_image.h>
#include "headsegmenter.h"
BOOL DetectProfile(std::vector<cv::Point2i> points);	//≈–± «∑Ò «≤‡¡≥
BOOL JudgeFrontFace(const dip::Depth* depthshort, cv::Mat color, dlib::frontal_face_detector detector, dlib::shape_predictor sp);
#endif