#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "data_reader.h"


int main(int argc, const char** argv)
{
    std::string path = argv[1];

    std::vector<std::string> file_paths = list_directory(path);

    std::vector<cv::Point3d> points3d = read_data(file_paths.at(0));


    std::cout << points3d.size() << std::endl;
    std::cout << points3d[0] << std::endl;


    const double v = 1.0, u = 0.5, rad = 100.0;

    int m_x = 0;
    int m_y = 0;

    cv::Mat img = cv::Mat::zeros(1200, 2000, CV_64F);


    

    cv::Mat rvec, tvec, cmat;
    
    rvec.create(1, 3, CV_64F);
    rvec.at<double>(0) = 0.0;
    rvec.at<double>(1) = 0.0;
    rvec.at<double>(2) = 0.0;

    tvec.create(3, 1, CV_64F);
    tvec.at<double>(0) = 0.0;
    tvec.at<double>(1) = 0.0;
    tvec.at<double>(2) = 0.0;

    cmat.create(3, 3, CV_64F);
    cv::setIdentity(cmat);
    cmat.at<double>(0, 0) = 50.0;
    cmat.at<double>(1, 1) = 50.0;
    cmat.at<double>(0, 2) = 1000.0;
    cmat.at<double>(1, 2) = 600.0;

    std::vector<cv::Point2d> points2d;
    points2d.reserve(points3d.size());

    cv::projectPoints(points3d, rvec, tvec, cmat, cv::noArray(), points2d);

    std::cout << points2d[0] << std::endl;

    //cv::Mat img = cv::Mat::zeros(2000, 2000, CV_64F);

    for (const auto& p : points2d)
        cv::circle(img, p, 3, cv::Scalar(255, 255, 255), -1);

    cv::imshow("Image", img);
    cv::waitKey(0);
    
    return 0;
}
