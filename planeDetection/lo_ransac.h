#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


size_t GetIterationNumber(
	const double& inlier_ratio,
	const double& confidence,
	const size_t& sample_size
);

void SelectMinimalSample
(
	int& n_points,
	std::vector<int>& sample,
	int k_sample_size
);


void CalculateInliers
(
	const std::vector<cv::Point3d>& points,
	std::vector<size_t>& current_inliers,
	const double& threshold,
	double& a,
	double& b,
	double& c,
	double& d
);

void FitPlaneInnerRANSAC
(
	const std::vector<cv::Point3d>& points,
	std::vector<cv::Point2d>& projected_points,
	std::vector<size_t>& best_inliers,
	std::vector<double>& best_plane,
	const double& threshold,
	const size_t& n_iterations,
	cv::Mat* img
);

void FitPlaneLoRANSAC
(
	const std::vector<cv::Point3d>& points,
	std::vector<cv::Point2d>& projected_points,
	std::vector<size_t>& inliers,
	std::vector<double>& plane,
	const double& threshold,
	const double& confidence,
	const size_t& max_iteration_desired,
	cv::Mat* img,
	const int& method
);
