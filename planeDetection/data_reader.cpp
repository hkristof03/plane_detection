#include "data_reader.h"


std::vector<std::string> list_directory(std::string& path)
{
    std::vector<std::string> file_paths;
    file_paths.reserve(3);

    for (const auto& entry : fs::directory_iterator(path))
        file_paths.push_back(entry.path().string());

    return file_paths;
}

std::vector<cv::Point3d> read_data(std::string& path)
{
    std::vector<cv::Point3d> points3d;
    std::string line;
    std::ifstream PointFile(path);

    std::cout << "Reading 3D point coordinates from path: " << std::endl 
        << path << std::endl;

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

    std::cout << "Finished reading from file." << std::endl;

    return points3d;
}