#include "lo_ransac.h"
#include "projection.h"
#include "optimizers.h"


//extern double u, v, rad;
//extern cv::Mat camera_matrix;
extern std::string window_name;
extern cv::Scalar points_plane_color;
extern cv::Scalar inlier_color;


size_t GetIterationNumber(
	const double& inlier_ratio,
	const double& confidence,
	const size_t& sample_size
)
{
	double a = log(1.0 - confidence);
	double b = log(1.0 - std::pow(inlier_ratio, sample_size));

	if (abs(b) < std::numeric_limits<double>::epsilon())
		return std::numeric_limits<double>::epsilon();

	size_t it_num = static_cast<size_t>(a / b);

	return it_num;
}

void FitPlaneLoRANSAC
(
	const std::vector<cv::Point3d>& points,
	std::vector<cv::Point2d> &projected_points,
	std::vector<size_t>& inliers,
	std::vector<double>& plane,
	const double& threshold,
	const double &confidence,
	const size_t& max_iteration_desired,
	cv::Mat* img
)
{
	// set random seed
	srand(time(NULL));
	// The current number of iterations
	int iteration_number = 0;
	// The numer of inliers of the current best model
	int best_inlier_number = 0;
	// The indices of the inliers of the current best model
	std::vector<size_t> best_inliers, current_inliers;
	best_inliers.reserve(points.size());
	current_inliers.reserve(points.size());
	// The parameters of the best plane
	std::vector<double> best_plane{ 0.0, 0.0, 0.0, 0.0 };
	// The sample size i.e 3 for 3D plane
	constexpr int k_sample_size = 3;
	// The current sample
	std::vector<int> sample(k_sample_size);
	// In the beginning the maximum iterations is equals to the desired steps
	size_t max_iterations = max_iteration_desired;

	bool should_draw = img != nullptr;
	cv::Mat tmp_img;

	std::cout << "Executing Locally-optimized RANSAC!" << std::endl,
	std::cout << "Number of points: " << points.size() << std::endl;
	std::cout << "Threshold is: " << threshold << std::endl;
	std::cout << "Confidence is: " << confidence << std::endl;
	std::cout << "Desired maximum iterations: " << max_iteration_desired << std::endl;
	// RANSAC:
	// 1. Select a minimal sample in this case 3 random points
	// 2. Fit a plane to the points
	// 3. Count the number of inliers: number of points closer than the threshold to the plane
	// 4. Store the inlier number and the plane parameters if it is better than the previous best

	while (iteration_number++ < max_iterations)
	{
		// 1. Select a minimal sample -> 3 random points
		for (std::size_t sample_idx = 0; sample_idx < k_sample_size; ++sample_idx)
		{
			do
			{
				// Generate a random index between [0, n_points]
				sample[sample_idx] =
					int((points.size() - 1.0) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));

				if (sample_idx == 0)
					break;

				if (sample_idx == 1 && sample[0] != sample[1])
					break;

				if (sample_idx == 2 && sample[0] != sample[2] && sample[1] != sample[2])
					break;

			} while (true);
		}
		const cv::Point3d& p1 = points[sample[0]];
		const cv::Point3d& p2 = points[sample[1]];
		const cv::Point3d& p3 = points[sample[2]];

		// 2. Fit a plane to the points
		// These two vectors are on the plane
		cv::Point3d v1 = p3 - p1;
		cv::Point3d v2 = p2 - p1;

		// cross product is a vector normal to the plane
		cv::Point3d cp = v1.cross(v2);
		// Divide length to get unit length normal vector
		cp = cp / cv::norm(cp);
		// Equation of a plane through three points:
		// A * x + B * y + C * z = D
		double a = cp.x, b = cp.y, c = cp.z;
		// D = A * x0 + B * y0 + C * z0
		double d = -cp.dot(p3);

		// Distance of a line and a point
		// n = <A, B, C>
		// dist = |A * x + B * y + C * z + D| / sqrt(A^2 + B^2 + C^2)
		// if n is unit length -> divide by 1

		// 3. Count the number of inliers -> the number of points closer than the threshold
		current_inliers.clear();
		for (std::size_t point_idx = 0; point_idx < points.size(); ++point_idx)
		{
			const cv::Point3d& p = points[point_idx];
			const double distance = abs(a * p.x + b * p.y + c * p.z + d);

			if (distance < threshold)
			{
				current_inliers.emplace_back(point_idx);
			}
		}
		// 4. Store the inlier number and the line parameters if it is better than the previous best
		if (current_inliers.size() > best_inliers.size())
		{
			std::cout << "Iteration number: " << iteration_number << std::endl;
			std::cout << "Current best inliers size: " << current_inliers.size()
				<< std::endl;

			best_inliers.swap(current_inliers);
			current_inliers.clear();
			current_inliers.resize(0);

			best_plane.at(0) = a;
			best_plane.at(1) = b;
			best_plane.at(2) = c;
			best_plane.at(3) = d;

			std::cout << "p1: " << p1 << std::endl;
			std::cout << "p2: " << p2 << std::endl;
			std::cout << "p3: " << p3 << std::endl;
			
			std::cout << "Parameters of the plane are: " << std::endl << "a: " << a
				<< std::endl << "b: " << b << std::endl << "c: " << c << std::endl
				<< "d: " << d << std::endl;

			double avg_error = CalculateAverageError(points, best_inliers, best_plane);
			std::cout << "Average Error before LSQ optimization: " << avg_error << std::endl;

			FitPlaneLSQ(points, best_inliers, best_plane);

			avg_error = CalculateAverageError(points, best_inliers, best_plane);
			std::cout << "Average Error after LSQ optimization: " << avg_error << std::endl;

			// Update the maximum iteration number
			double inlier_ratio = static_cast<double>(best_inliers.size()) /
				static_cast<double>(points.size());
			max_iterations = GetIterationNumber(
				inlier_ratio,
				confidence,
				k_sample_size
			);
			std::cout << "Max iterations: " << max_iterations << std::endl;

			if (should_draw)
			{
				tmp_img = img->clone();

				for (const auto& idx : best_inliers)
					DrawPoint(tmp_img, projected_points[idx], 0.5, inlier_color);

				for (const auto& idx : sample)
					DrawPoint(tmp_img, projected_points[idx], 3, points_plane_color);

				cv::imshow(window_name, tmp_img);
				cv::waitKey(500);
			}
		}
	}
	inliers = best_inliers;
	plane = best_plane;
}