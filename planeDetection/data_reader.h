#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

std::vector<std::string> list_directory(std::string& path);

std::vector<cv::Point3d> read_data(std::string& path);

void draw_points(std::vector<cv::Point2d> points2d);
