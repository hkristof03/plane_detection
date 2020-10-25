#include "data_reader.h"


std::vector<std::string> list_directory(std::string& path)
{
    std::vector<std::string> file_paths;
    file_paths.reserve(3);

    for (const auto& entry : fs::directory_iterator(path))
        file_paths.push_back(entry.path().string());

    return file_paths;
}

std::vector<cv::Point3d> read_data(std::string& str)
{
    std::vector<cv::Point3d> points3d;
    std::string line;
    std::ifstream PointFile(str);

    while (std::getline(PointFile, line))
    {
        double x, y, z;
        std::string s_x, s_y, s_z;
        std::stringstream ss(line);

        ss >> s_x >> s_y >> s_z;
        x = atof(s_x.c_str());
        y = atof(s_y.c_str());
        z = atof(s_z.c_str());

        cv::Point3d p{ x, y, z };

        points3d.push_back(p);
    }
    PointFile.close();

    return points3d;
}