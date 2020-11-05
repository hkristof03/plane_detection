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

void FitPlaneLoRANSAC
(
	const std::vector<cv::Point3d>& points,
	std::vector<cv::Point2d>& projected_points,
	std::vector<size_t>& inliers,
	std::vector<double>& plane,
	const double& threshold,
	const double& confidence,
	const size_t& max_iteration_desired,
	cv::Mat* img
);

