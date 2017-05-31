/*
Copyright (c) 2013-2015, Gregory P. Meyer
                         University of Illinois Board of Trustees
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder(s) nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "stdafx.h"
#include "facemodeling.h"
#include <opencv/cv.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <numeric>
#include "face_detect/JudgeFrontFace.h"
//#define DEBUGING
//#define STDOUT

using namespace Eigen;

namespace dip {

FaceModeling::FaceModeling(int width, int height, float fx, float fy,
                           float cx, float cy, std::string face_detection_model_file) :
                           min_weight_(0.0f), width_(width), height_(height),
                           fx_(fx), fy_(fy), cx_(cx), cy_(cy),
                           initial_frame_(true), failed_frames_(0) {
	CurrentFaceId = -1;
	isHumanPresent = false;
	for (int i = 0; i < nFrameInDeadZone; i++) isFacePresentInFrames.push_back(false);
	NumIntegratedDepth = 0;

	FaceDetector = dlib::get_frontal_face_detector();
	dlib::deserialize(face_detection_model_file) >> ShapePredictor;
	std::cout << "Face detection model has been loaded." << std::endl;

  // Allocate depth image on the CPU.
  depth_ = new Depth[width_ * height_ * kKinect];

  // Allocate depth images on the GPU.
  Allocate((void**)&segmented_depth_, sizeof(Depth) * width_ * height_ * kKinect);
  Allocate((void**)&filtered_depth_, sizeof(Depth) * width_ * height_ * kKinect);
  Allocate((void**)&denoised_depth_, sizeof(Depth) * width_ * height_ * kKinect);

  // Allocate pyramids on the GPU.
  for (int k = 0; k < kKinect; k++) {
	  for (int i = 0; i < kPyramidLevels; i++) {
		  Allocate((void**)&(depth_pyramid_[k][i]), sizeof(Depth) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));

		  Allocate((void**)&(vertices_[k][i].x), sizeof(float) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));
		  Allocate((void**)&(vertices_[k][i].y), sizeof(float) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));
		  Allocate((void**)&(vertices_[k][i].z), sizeof(float) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));

		  Allocate((void**)&(normals_[k][i].x), sizeof(float) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));
		  Allocate((void**)&(normals_[k][i].y), sizeof(float) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));
		  Allocate((void**)&(normals_[k][i].z), sizeof(float) *
			  (width_ >> (i * kDownsampleFactor)) *
			  (height_ >> (i * kDownsampleFactor)));
	  }

	  Allocate((void**)&(model_vertices_[k].x), sizeof(float) * width_ * height_);
	  Allocate((void**)&(model_vertices_[k].y), sizeof(float) * width_ * height_);
	  Allocate((void**)&(model_vertices_[k].z), sizeof(float) * width_ * height_);

	  Allocate((void**)&(model_normals_[k].x), sizeof(float) * width_ * height_);
	  Allocate((void**)&(model_normals_[k].y), sizeof(float) * width_ * height_);
	  Allocate((void**)&(model_normals_[k].z), sizeof(float) * width_ * height_);
  }
  // Allocate the volume on the GPU and
  // set the value of each voxel to zero.
  Allocate((void**)&volume_, sizeof(Voxel) *
           kVolumeSize * kVolumeSize * kVolumeSize);
  Clear((void*)volume_, sizeof(Voxel) *
        kVolumeSize * kVolumeSize * kVolumeSize);

  // Allocate normal map on GPU.
  Allocate((void**)&normal_map_, sizeof(Color) * width_ * height_);

  // Initialize rigid body transformation to the identity matrix.
  for (int k = 0; k < 2; k++)
	  transform_from_side_to_central[k].setIdentity();

  transformation_[0].setIdentity();
  if (kKinect > 1) transformation_[1] = transform_from_side_to_central[0];
  if (kKinect > 2) transformation_[2] = transform_from_side_to_central[1];
}

FaceModeling::~FaceModeling() {
  delete [] depth_;

  Deallocate((void*)(segmented_depth_));
  Deallocate((void*)(filtered_depth_));
  Deallocate((void*)(denoised_depth_));

  for (int k = 0; k < kKinect; k++) {
	  for (int i = 0; i < kPyramidLevels; i++) {
		  Deallocate((void*)depth_pyramid_[k][i]);

		  Deallocate((void*)vertices_[k][i].x);
		  Deallocate((void*)vertices_[k][i].y);
		  Deallocate((void*)vertices_[k][i].z);

		  Deallocate((void*)normals_[k][i].x);
		  Deallocate((void*)normals_[k][i].y);
		  Deallocate((void*)normals_[k][i].z);
	  }

	  Deallocate((void*)model_vertices_[k].x);
	  Deallocate((void*)model_vertices_[k].y);
	  Deallocate((void*)model_vertices_[k].z);

	  Deallocate((void*)model_normals_[k].x);
	  Deallocate((void*)model_normals_[k].y);
	  Deallocate((void*)model_normals_[k].z);
  }
  Deallocate((void*)volume_);

  Deallocate((void*)normal_map_);
}

// Return value:
// -1 means current tracking fails
// 0 means current tracking can be used
int FaceModeling::Run(const Depth *depth, Color *normal_map, cv::Mat ColorImage,
	std::vector<Eigen::Matrix4f>& TransformFrameToGlobal, bool& RetrieveMeshNow) {
	// Hold the mesh and initialize output transformations
	RetrieveMeshNow = false;
	TransformFrameToGlobal.resize(kKinect, Eigen::Matrix4f::Identity());

		//bool isFrontalFaceDetected = JudgeFrontFace(depth, ColorImage, FaceDetector, ShapePredictor);
		//if (!isFrontalFaceDetected) {
		//	std::cout << "Failed to detect frontal face!================================" << std::endl;
		//	return -1;
		//}
		//return -1;

	// Face detection
	if (!isHumanPresent) {
		bool isFrontalFaceDetected = JudgeFrontFace(depth, ColorImage, FaceDetector, ShapePredictor);
		if (!isFrontalFaceDetected) {
#ifdef STDOUT
			std::cout << "Failed to detect frontal face!" << std::endl;
#endif
			return -1;
		}
	}

	// Segment the user's head from the depth image.
	std::vector<bool> isSegmentationOK(kKinect, false);
	std::vector<bool> isAlignmentOK(kKinect, true);
	for (int k = 0; k < kKinect; k++) {
		isSegmentationOK[k] = (0 == head_segmentation_.Run(kMinDepth, kMaxDepth, kMaxDifference,
			kMinHeadWidth, kMinHeadHeight,
			kMaxHeadWidth, kMaxHeadHeight,
			fx_, fy_, width_, height_, depth, depth_ + k * width_ * height_, 0));
		if (!isSegmentationOK[0]) break;
	}

#ifdef DEBUGING
	// DEBUG
	cv::Mat DepthImage(height_, width_, CV_8UC1);
	cv::Mat SegImage(height_, width_, CV_8UC1);
	for (int r = 0; r < height_; r++)
		for (int c = 0; c < width_; c++) {
			DepthImage.at<unsigned char>(r, c) = (unsigned char)(depth[r*width_ + c] / 4096.0 * 255.0);
			SegImage.at<unsigned char>(r, c) = (unsigned char)(depth_[r*width_+c] / 4096.0 * 255.0);
		}
	cv::imshow("depth", DepthImage);
	cv::imshow("segmentation", SegImage);
	cv::waitKey(1);
#endif

	// Update states
	isFacePresentInFrames.push_back(isSegmentationOK[0]);
	isFacePresentInFrames.pop_front();
	if (!isHumanPresent) {
		if (std::accumulate(isFacePresentInFrames.begin(), isFacePresentInFrames.end(), 0) < nFrameInDeadZone)
		{
			; // state will not change
		}
		else {
			long t1 = clock();
			// Confirm that a person is standing in front of the device and start volumetric reconstruction
			isHumanPresent = true;
			// This is the only case when initial_frame_ becomes true
			initial_frame_ = true;
			// Update face ID
			CurrentFaceId++;
			// Clear volume
			Clear((void*)volume_, sizeof(Voxel) * kVolumeSize * kVolumeSize * kVolumeSize);
			min_weight_ = 0;

			transformation_[0].setIdentity();
			if (kKinect > 1) transformation_[1] = transform_from_side_to_central[0];
			if (kKinect > 2) transformation_[2] = transform_from_side_to_central[1];

			NumIntegratedDepth = 0;
			long t2 = clock();
			//printf("clear volume_ time:%d", t2 - t1);
		}
	}
	else {
		if (std::accumulate(isFacePresentInFrames.begin(), isFacePresentInFrames.end(), 0) > 0)
		{
			; // state will not change
		}
		else
		{
			isHumanPresent = false;
			RetrieveMeshNow = true; // signal the caller that a mesh is ready
		}
	}
#ifdef STDOUT
	if(!isHumanPresent)
		printf("Waiting next people\n");
#endif
	if (!(isHumanPresent && isSegmentationOK[0])) {
		//std::cout << isHumanPresent << " " << isSegmentationOK[0] << std::endl;
		return -1;
	}
#ifdef STDOUT
	printf("Current Face ID: %d\n", CurrentFaceId);
#endif
  // Upload the segmented depth image from the CPU to the GPU.
  Upload(segmented_depth_, depth_, sizeof(Depth) * width_ * height_ * kKinect);

  // Filter the depth image.
  for (int k = 0; k < kKinect; k++) {
	  if (isSegmentationOK[k]) {
		  //variance_filter_.Run(width_, height_, segmented_depth_ + k * width_ * height_, filtered_depth_ + k * width_ * height_);
		  Copy(filtered_depth_, segmented_depth_, sizeof(Depth)*width_*height_);
		  bilateral_filter_.Run(kRegistrationBilateralFilterSigmaD,
			  kRegistrationBilateralFilterSigmaR,
			  width_, height_, filtered_depth_ + k * width_ * height_, denoised_depth_ + k * width_ * height_);
		  // Construct depth image pyramid.
		  Copy(depth_pyramid_[k][0], denoised_depth_ + k * width_ * height_, sizeof(Depth) * width_ * height_);
		  for (int i = 1; i < kPyramidLevels; i++) {
			  downsample_.Run(kDownsampleFactor, kDownsampleMaxDifference,
				  (width_ >> ((i - 1) * kDownsampleFactor)),
				  (height_ >> ((i - 1) * kDownsampleFactor)),
				  (width_ >> (i * kDownsampleFactor)),
				  (height_ >> (i * kDownsampleFactor)),
				  depth_pyramid_[k][i - 1], depth_pyramid_[k][i]);
		  }
		  // Construct the point-cloud pyramid by
		  // back-projecting the depth image pyramid.
		  for (int i = 0; i < kPyramidLevels; i++) {
			  back_projection_.Run((width_ >> (i * kDownsampleFactor)),
				  (height_ >> (i * kDownsampleFactor)),
				  (fx_ / (1 << (i * kDownsampleFactor))),
				  (fy_ / (1 << (i * kDownsampleFactor))),
				  (cx_ / (1 << (i * kDownsampleFactor))),
				  (cy_ / (1 << (i * kDownsampleFactor))),
				  depth_pyramid_[k][i], vertices_[k][i], normals_[k][i]);
		  }
	  }
  }

  // Compute the center of mass of the point-cloud (using only the central kinect's point cloud)
  Vertex center = centroid_.Run(width_, height_, vertices_[0][0]);

  // Register the current frame to the previous frame.
  if (!initial_frame_) {
    Matrix4f previous_transformation = transformation_[0];

    // Create Transformation that aligns the Current Frame
    // with the Previous Frame based on the centroids
    Matrix4f frame_transformation;
    frame_transformation.setIdentity();

    frame_transformation(0, 3) = previous_center_.x - center.x;
    frame_transformation(1, 3) = previous_center_.y - center.y;
    frame_transformation(2, 3) = previous_center_.z - center.z;

    // Approximate the current frame's global transformation.
    transformation_[0] = transformation_[0] * frame_transformation;

    // Perform ICP for middle image
    for (int i = kPyramidLevels - 1; i >= 0; i--) {
      if (icp_.Run(kICPIterations[i],
                   kMinCorrespondences[i + 1], kMinCorrespondences[i],
                   kDistanceThreshold[i + 1], kDistanceThreshold[i],
                   kNormalThreshold[i + 1], kNormalThreshold[i],
                   kMaxRotation, kMaxTranslation,
                   fx_, fy_, cx_, cy_,
                   (width_ >> (i * kDownsampleFactor)),
                   (height_ >> (i * kDownsampleFactor)),
                   width_, height_, vertices_[0][i], normals_[0][i],
                   model_vertices_[0], model_normals_[0],
                   previous_transformation, transformation_[0])) {
		  std::cout << "ICP failed." << std::endl;
		isAlignmentOK[0] = false;
        transformation_[0] = previous_transformation; // no need to render again

        if (failed_frames_ >= kMaxFailedFrames) {
		  printf("current mesh fusing frames amount:%d\n", NumIntegratedDepth);
		  // reset states
		  isHumanPresent = false;
		  for (auto& Elem : isFacePresentInFrames) Elem = false;
		  // signal the caller that a mesh is ready
		  RetrieveMeshNow = true;
        }

        failed_frames_++;
		return -1;
      }
    }
	if (isAlignmentOK[0]) {
		// Align side images
		for (int k = 1; k < kKinect; k++) {
			previous_transformation = transformation_[k]; // camera transformation in last rendering
			transformation_[k] = transformation_[0] * transform_from_side_to_central[k - 1];
			isAlignmentOK[k] = (0 == icp_.Run(kICPIterations[kPyramidLevels - 1],
												kMinCorrespondences[kPyramidLevels], kMinCorrespondences[kPyramidLevels - 1],
												kDistanceThreshold[kPyramidLevels], kDistanceThreshold[kPyramidLevels - 1],
												kNormalThreshold[kPyramidLevels], kNormalThreshold[kPyramidLevels - 1],
												kMaxRotation, kMaxTranslation,
												fx_, fy_, cx_, cy_,
												(width_ >> ((kPyramidLevels - 1) * kDownsampleFactor)),
												(height_ >> ((kPyramidLevels - 1) * kDownsampleFactor)),
												width_, height_, vertices_[k][kPyramidLevels - 1], normals_[k][kPyramidLevels - 1],
												model_vertices_[k], model_normals_[k],
												previous_transformation, transformation_[k]));
		}
	}
  }
  else {
    // Set the center of the volume to the
    // center of mass of the initial frame.
    volume_center_ = center;
	for (int k = 1; k < kKinect; k++) {
		if (!isSegmentationOK[k]) continue;
		// Align the k-th depth image to the first one
		Eigen::Matrix4f previous_transformation = transformation_[0];
		Eigen::Matrix4f predict_transformation = transformation_[0] * transform_from_side_to_central[k - 1];
		if (0 == icp_.Run(kICPIterations[kPyramidLevels - 1],
			kMinCorrespondences[kPyramidLevels], kMinCorrespondences[kPyramidLevels - 1],
			kDistanceThreshold[kPyramidLevels], kDistanceThreshold[kPyramidLevels - 1],
			kNormalThreshold[kPyramidLevels], kNormalThreshold[kPyramidLevels - 1],
			kMaxRotation, kMaxTranslation,
			fx_, fy_, cx_, cy_,
			(width_ >> ((kPyramidLevels - 1) * kDownsampleFactor)),
			(height_ >> ((kPyramidLevels - 1) * kDownsampleFactor)),
			width_, height_, vertices_[k][kPyramidLevels - 1], normals_[k][kPyramidLevels - 1],
			vertices_[0][kPyramidLevels - 1], normals_[0][kPyramidLevels - 1],
			previous_transformation, predict_transformation))
		{
			transformation_[k] = predict_transformation;
		}
		else {
			isAlignmentOK[k] = false;
		}
	}
  }

  failed_frames_ = 0;

  // Integrate the segmented depth image into the volumetric model.
  for (int k = 0; k < kKinect; k++) {
	  if (isAlignmentOK[k]) {
		  bilateral_filter_.Run(kIntegrationBilateralFilterSigmaD,
			  kIntegrationBilateralFilterSigmaR,
			  width_, height_, filtered_depth_ + k * width_ * height_, denoised_depth_ + k * width_ * height_);

		  volumetric_.Run(kVolumeSize, kVolumeDimension, kVoxelDimension,
			  kMaxTruncation, kMaxWeight, width_, height_,
			  fx_, fy_, cx_, cy_, volume_center_, transformation_[k].inverse(),
			  denoised_depth_ + k * width_ * height_, normals_[k][0], volume_);
		  NumIntegratedDepth++;
	  }
  }
  min_weight_ = MIN(min_weight_ + kMinWeightPerFrame / kMaxWeight,
                    kMaxMinWeight / kMaxWeight);

  // Render the volume using ray casting. Update the model point-cloud
  // for the next frame's registration step. Generate the normal map of
  // the model to display the current state of the model to the user.
  // NOTE that the rendering of side images will not use transformation_ because ICP may have failed
  for (int k = 0; k < kKinect; k++) {
	  Eigen::Matrix4f T = transformation_[0];
	  if (k > 0) {
		  T = transformation_[0] * transform_from_side_to_central[k - 1];
		  transformation_[k] = T;
	  }
	  ray_casting_.Run(kMaxDistance, kMaxTruncation, kVolumeSize, kVolumeDimension,
		  kVoxelDimension, min_weight_, width_, height_,
		  fx_, fy_, cx_, cy_, volume_center_, T,
		  volume_, model_vertices_[k], model_normals_[k], normal_map_);
  }
  // Update TransformFromFrameToGlobal
  for (int k = 0; k < kKinect; k++) {
	  if (isSegmentationOK[k] && isAlignmentOK[k])
		  TransformFrameToGlobal[k] = transformation_[k]; // note that transform_[0] is always valid at this step
	  else {
		  TransformFrameToGlobal[k] = transformation_[0] * transform_from_side_to_central[k - 1];
	  }
  }

#ifdef DEBUGING
  // Download the normal map from the GPU to the CPU.
  if (normal_map != NULL)
    Download(normal_map, normal_map_, sizeof(Color) * width_ * height_);
  // debug
  cv::Mat NormalImage;
  NormalImage.create(height_, width_, CV_8UC3);
  for (int r = 0; r < height_; r++)
	  for (int c = 0; c < width_; c++)
		  NormalImage.at<cv::Vec3b>(r, c) = cv::Vec3b(normal_map[r * width_ + c].b,
													  normal_map[r * width_ + c].g,
													  normal_map[r * width_ + c].r);
  cv::imshow("rendered normals", NormalImage);
  cv::waitKey(1);
#endif

  // Update Model Center
  previous_center_ = center;
  initial_frame_ = false;

  if (NumIntegratedDepth >= kMaxIntegratedDepth)
  {
	  printf("current mesh fusing frames amount:%d\n", NumIntegratedDepth);
	  // reset states
	  isHumanPresent = false;
	  for (auto& Elem : isFacePresentInFrames) Elem = false;
	  // signal the caller that a mesh is ready
	  RetrieveMeshNow = true;
  }
  return 0;
}

void FaceModeling::Model(Mesh *mesh) {
  // Allocate volume on the CPU.
  Voxel *volume = new Voxel[kVolumeSize * kVolumeSize * kVolumeSize];

  // Download the volume from the GPU to the CPU.
  Download((void*)volume, volume_, sizeof(Voxel) *
           kVolumeSize * kVolumeSize * kVolumeSize);

  // Construct mesh using marching cubes.
  marching_cubes_.Run(kVolumeSize, kVolumeDimension, kVoxelDimension,
                      min_weight_, volume_center_, volume, mesh);

  delete [] volume;
}

} // namespace dip
