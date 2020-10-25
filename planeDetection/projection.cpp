#include "projection.h"


cv::Mat initialize_camera_matrix(
	double& focal_length,
	double& k_size,
	double& u,
	double& v
) 
{
	// k_size sizes of pixels in horizontal and vertical direction
	// u, v are center of the image (width/2, height/2)
	cv::Mat camera_matrix(3, 3, CV_64F);
	cv::setIdentity(camera_matrix);

	camera_matrix.at<double>(0, 0) = focal_length * k_size;
	camera_matrix.at<double>(0, 2) = u;
	camera_matrix.at<double>(1, 1) = focal_length * k_size;
	camera_matrix.at<double>(1, 2) = v;

	return camera_matrix;
}

std::vector<cv::Point2d> project_points(
	std::vector<cv::Point3d> points3d,
	const double& u,
	const double& v,
	const double& rad,
	const cv::Mat& camera_matrix
)
{
	cv::Mat rotation_matrix(3, 3, CV_64F);
	cv::Mat translation_vec;
	translation_vec.create(3, 1, CV_64F);

	double tx = cos(u) * sin(v);
	double ty = sin(u) * sin(v);
	double tz = cos(v);

	translation_vec.at<double>(0, 0) = rad * tx;
	translation_vec.at<double>(1, 0) = rad * ty;
	translation_vec.at<double>(2, 0) = rad * tz;

	// Mirroring problem
	int count_pi = (int)floor(v / PI);

	// Axes
	cv::Point3d ax_z(-1.0 * tx, -1.0 * ty, -1.0 * tz);
	cv::Point3d ax_x(ty, -tx, 0.0);

	double length_x = sqrt(pow(ax_x.x, 2.0) + pow(ax_x.y, 2.0) + pow(ax_x.z, 2.0));
	
	if (count_pi % 2)
		ax_x = (1.0 / length_x) * ax_x;
	else
		ax_x = (-1.0 / length_x) * ax_x;

	cv::Point3d ax_y = ax_x.cross(ax_z);
	
	rotation_matrix.at<double>(0, 0) = ax_x.x;
	rotation_matrix.at<double>(0, 1) = ax_x.y;
	rotation_matrix.at<double>(0, 2) = ax_x.z;

	rotation_matrix.at<double>(1, 0) = ax_y.x;
	rotation_matrix.at<double>(1, 1) = ax_y.y;
	rotation_matrix.at<double>(1, 2) = ax_y.z;

	rotation_matrix.at<double>(2, 0) = ax_z.x;
	rotation_matrix.at<double>(2, 1) = ax_z.y;
	rotation_matrix.at<double>(2, 2) = ax_z.z;

	std::vector<cv::Point2d> points2d;
	points2d.reserve(points3d.size());
	
	cv::Mat matrix_points3d = cv::Mat(points3d).reshape(1);

	for (size_t i = 0; i < matrix_points3d.rows; i++)
	{
		cv::Mat point3d = matrix_points3d.row(i).t();
		cv::Mat tr_vec = 
			camera_matrix * rotation_matrix * (point3d - translation_vec);
		tr_vec = tr_vec / tr_vec.at<double>(2, 0);

		cv::Point2d point2d{ tr_vec.at<double>(0, 0), tr_vec.at<double>(1, 0) };
		points2d.push_back(point2d);
	}

	return points2d;
}

void draw_points(
	std::vector<cv::Point2d> &points2d, 
	cv::Mat& img, 
	const cv::Scalar &point_color
)
{
	cv::Mat tmp_img = img.clone();

	for (const auto& p : points2d)
		cv::circle(tmp_img, p, 1, point_color, -1);

	cv::imshow("3D Point Cloud", tmp_img);
	cv::waitKey(0);
}