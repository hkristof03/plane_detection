#pragma once
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

const double PI = 3.14159265;

cv::Mat initialize_camera_matrix(
	double& focal_length,
	double& k_size,
	double& u,
	double& v
);

std::vector<cv::Point2d> project_points(
	std::vector<cv::Point3d> points3d,
	const double& u,
	const double& v,
	const double& rad,
	const cv::Mat& camera_matrix
);

void draw_points(
	std::vector<cv::Point2d>& points2d,
	cv::Mat& img,
	const cv::Scalar& point_color
);
