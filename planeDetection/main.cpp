#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "data_reader.h"
#include "projection.h"
#include "control.h"


int main(int argc, const char** argv)
{
    std::string path = argv[1];

    std::vector<std::string> file_paths = list_directory(path);

    std::vector<cv::Point3d> points3d = read_data(file_paths.at(0));


    const cv::Scalar point_color{ 255, 255, 255 };
    cv::Mat img = cv::Mat::zeros(1200, 2000, CV_64F);

    cv::Mat camera_matrix = initialize_camera_matrix(focal_length, 
        k_size, u, v);


    auto points2d = project_points(points3d, u, v, rad, camera_matrix);
	draw_points(points2d, img, point_color);
    
	char key;
	int cnt = 0;

	while (true)
	{
		bool changed = false;

		key = cvWaitKey(30);

		if (key == 27) break;

		switch (key)
		{
		case 'q':
			u += 5.0;
			changed ^= 1;
			cnt++;
			break;
		case 'a':
			u -= 5.0;
			changed ^= 1;
			cnt++;
			break;
		case 'w':
			v += 5.0;
			changed ^= 1;
			cnt++;
			break;
		case 's':
			v -= 5.0;
			changed ^= 1;
			cnt++;
			break;
		case 'e':
			rad *= 1.1;
			changed ^= 1;
			cnt++;
			break;
		case 'd':
			rad /= 1.1;
			changed ^= 1;
			cnt++;
			break;
		}
		if (changed && cnt % 3 == 0)
		{
			std::cout << "cnt: " << cnt << std::endl;
			std::cout << "u: " << u << std::endl;
			std::cout << "v: " << v << std::endl;
			std::cout << "rad: " << rad << std::endl;

			auto points2d = project_points(points3d, u, v, rad, camera_matrix);
			draw_points(points2d, img, point_color);
		}
	}
    return 0;
}
