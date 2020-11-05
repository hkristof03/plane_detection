#include "optimizers.h"


double CalculateAverageError(
	const std::vector<cv::Point3d>& points,
	std::vector<size_t>& inliers,
	std::vector<double>& plane
)
{
	cv::Point3d normal_vec = cv::Point3d(plane.at(0), 
		plane.at(1), plane.at(2));
	
	double avg_error = 0.0;
	
	for (const auto &idx: inliers)
	{
		double distance = abs(points[idx].dot(normal_vec) + plane.at(3));
		avg_error += distance;
	}
	avg_error /= inliers.size();

	return avg_error;
}



void FitPlaneLSQ(
	const std::vector<cv::Point3d>& points,
	std::vector<size_t>& inliers,
	std::vector<double>& plane
)
{
	std::cout << "Executing Least-Squared Optimization for the plane..." << std::endl;
	std::vector<cv::Point3d> normalized_points;
	normalized_points.reserve(inliers.size());

	// Calculating the mass of the points
	cv::Point3d mass_point(0, 0, 0);

	for (const auto& idx : inliers)
	{
		mass_point += points.at(idx);
		normalized_points.emplace_back(points.at(idx));
	}
	mass_point *= (1.0 / inliers.size());
	// Move the point to have the origin in their mass point and
	// Calculate the average distance from the origin
	double avg_dist = 0.0;

	for (auto& p : normalized_points)
	{
		p -= mass_point;
		avg_dist += cv::norm(p);
	}
	avg_dist /= normalized_points.size();
		
	const double ratio = sqrt(3) / avg_dist;
	// Make the average distance for each point from the mass to be sqrt(2)
	for (auto& p : normalized_points)
		p *= ratio;
	// Solve the equation to get the principal components (PCA)
	cv::Mat A(static_cast<int>(normalized_points.size()), 3, CV_64F);
	// Build the coefficient matrix
	for (int idx = 0; idx < normalized_points.size(); ++idx)
	{
		A.at<double>(idx, 0) = normalized_points[idx].x;
		A.at<double>(idx, 1) = normalized_points[idx].y;
		A.at<double>(idx, 2) = normalized_points[idx].z;
	}
	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(A.t() * A, eigenvalues, eigenvectors);

	std::cout << "Eigen values: " << std::endl << eigenvalues << std::endl
		<< "Eigen vectors: " << std::endl << eigenvectors << std::endl;

	// The eigenvector that corresponds to the lowest eigenvalue has the lowest
	// variance with respect to the examined points (always in decreasing order)
	const cv::Mat& normal = eigenvectors.row(2);
	const double& a = normal.at<double>(0);
	const double& b = normal.at<double>(1);
	const double& c = normal.at<double>(2);
	const double d = -(cv::Point3d(a, b, c).dot(mass_point));

	plane.at(0) = a;
	plane.at(1) = b;
	plane.at(2) = c;
	plane.at(3) = d;
	
	std::cout << "Parameters of the LSQ optimized plane are: " << std::endl 
		<< "a: " << a << std::endl << "b: " << b << std::endl << "c: " << c 
		<< std::endl << "d: " << d << std::endl;
}