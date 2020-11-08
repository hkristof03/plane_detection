#pragma once
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>


double CalculateAverageError(
	const std::vector<cv::Point3d> &points,
	std::vector<size_t>& inliers,
	std::vector<double>& plane
);

void FitPlaneLSQ(
	const std::vector<cv::Point3d> &points,
	std::vector<size_t> &inliers,
	std::vector<double>& plane
);
