#pragma once
#include <iostream>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "data_reader.h"
#include "projection.h"
#include "lo_ransac.h"
#include "optimizers.h"


double focal_length = 3000.0, k_size = 2.0;
double u;
double v;
double rad;

std::string window_name = "3D Point Cloud";
cv::Scalar point_color{ 255, 255, 255 };
cv::Scalar points_plane_color{ 0, 0, 255 };
cv::Scalar inlier_color{ 0, 255, 0 };

int m_x = 0;
int m_y = 0;
int event_cnt = 0;


void mouse_callback(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		m_x = x;
		m_y = y;
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
		if (m_x != 0 && m_y != 0)
		{
			if (x < m_x) u -= 0.1;
			else if (x > m_x) u += 0.1;
			if (y < m_y) v -= 0.1;
			else if (y > m_y) v += 0.1;

			m_x = x;
			m_y = y;

			event_cnt++;
		}
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
		m_x = 0;
		m_y = 0;
	}
	else if (event == cv::EVENT_MOUSEWHEEL)
	{
		if (cv::getMouseWheelDelta(flags) > 0)
		{
			rad /= (float)1.1;
			event_cnt++;
		}

		else if (cv::getMouseWheelDelta(flags) < 0)
		{
			rad *= (float)1.1;
			event_cnt++;
		}
	}
}


int main(int argc, const char** argv)
{
	std::string path = argv[1];
	std::vector<std::string> file_paths = ListDirectory(path);
	cv::Mat img = cv::Mat::zeros(1200, 2000, CV_8UC3);
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

	std::vector<std::vector<double>> pparams;
	pparams.reserve(3);
	pparams.push_back(std::vector<double>{ 982.5, 599.1, 200.0 });
	pparams.push_back(std::vector<double>{ 986.1, 598.8, 57.9329 });
	pparams.push_back(std::vector<double>{983.2, 598.2, 354.313});
	
	const double threshold = 0.1;
	const size_t n_iterations = 1000;
	const double confidence = 0.99;

	for (int i = 0; i < file_paths.size(); ++i)
	{
		u = pparams[i][0];
		v = pparams[i][1]; 
		rad = pparams[i][2];
		cv::Mat camera_matrix = InitializeCameraMatrix(focal_length,
			k_size, u, v);

		std::vector<cv::Point3d> points3d = ReadData(file_paths.at(i));
		std::vector<cv::Point2d> points2d = ProjectPoints(points3d,
			u, v, rad, camera_matrix);
		
		cv::Mat tmp_img = img.clone();

		DrawPoints(points2d, window_name, tmp_img, point_color);
		cv::imshow(window_name, tmp_img);
		cv::waitKey(500);
		//LO-RANSAC
		std::vector<size_t> inliers;
		std::vector<double> best_plane;

		FitPlaneLoRANSAC(points3d, points2d, inliers, best_plane, threshold,
			confidence, n_iterations, &tmp_img);
	}
	
	/*
	u = pparams[2][0];
	v = pparams[2][1];
	rad = pparams[2][2];
	cv::Mat camera_matrix = InitializeCameraMatrix(focal_length,
		k_size, u, v);

	std::vector<cv::Point3d> points3d = ReadData(file_paths.at(2));
	std::vector<cv::Point2d> points2d = ProjectPoints(points3d,
		u, v, rad, camera_matrix);
		
	cv::Mat tmp_img = img.clone();

	DrawPoints(points2d, window_name, tmp_img, point_color);
	cv::imshow(window_name, tmp_img);
	cv::waitKey(500);

	cv::setMouseCallback(window_name, mouse_callback, nullptr);
	char key;
	
	while (true)
	{
		key = cv::waitKey(30);

		switch (key)
		{
		case 'q':
			u += 0.1;
			event_cnt++;
			break;
		case 'a':
			u -= 0.1;
			event_cnt++;
			break;
		case 'w':
			v += 0.1;
			event_cnt++;
			break;
		case 's':
			v -= 0.1;
			event_cnt++;
			break;
		case 'e':
			rad *= 1.1;
			event_cnt++;
			break;
		case 'd':
			rad /= 1.1;
			event_cnt++;
			break;
		}
		if (event_cnt % 3 == 0)
		{
			std::cout << "event_cnt: " << event_cnt << std::endl;
			std::cout << "u: " << u << std::endl;
			std::cout << "v: " << v << std::endl;
			std::cout << "rad: " << rad << std::endl;

			tmp_img = img.clone();
			auto points2d = ProjectPoints(points3d, u, v, rad, camera_matrix);
			DrawPoints(points2d, window_name, tmp_img, point_color);
			cv::imshow(window_name, tmp_img);
			cv::waitKey(0);
		}
	}
	*/
    return 0;
}
