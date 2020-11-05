#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

std::vector<std::string> ListDirectory(std::string& path);

std::vector<cv::Point3d> ReadData(std::string& path);
