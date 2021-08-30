#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>
#include <math.h>
#include <chrono>
#include <experimental/filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>

namespace fs = std::experimental::filesystem;

#include "../../src/PipelineMain.h"

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Check if the cloud is of type E.
bool isExpression(const std::string &filename)
{
    const std::string token = "_E_";
    if (filename.find(token) != std::string::npos) {
        return true;
    }
    return false;
}

// Check if the cloud is of type N.
bool isNeutral(const std::string &filename)
{
    const std::string token = "_N_";
    if (filename.find(token) != std::string::npos) {
        return true;
    }
    return false;
}

constexpr double EPSILON = 0.0001; // 1e-4

// Check if the two points are equal, given a small epsilon difference.
bool nearly_equal(double a1, double a2, double epsilon)
{
  if (a1 == 0 && a2 == 0)
    return true;

  return std::abs(a1 - a2) < epsilon * pow (2.0, static_cast<int> (std::log2(std::max(std::abs(a1), std::abs(a2)))));
}

void runExperiments(
    std::string pointType,
    std::string filename,
    bool onlyExpression,
    bool onlyNeutral,
    std::vector<std::string>& filters,
    std::vector<std::string>& kdtreeMethods,
    std::vector<float>& kdtreeValues,
    std::vector<float>& minThresholds,
    std::vector<float>& maxThresholds
)
{
    const int nFolders = 104;

    std::ofstream myfile;
    myfile.open(filename); // std::ios_base::app

    myfile << "cloud,original_size,filtered_size,contains" << std::endl;

    for (int i = 0; i <= nFolders; i++)
    {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << i;
        std::string folder = "bs" + oss.str();

        // Hardcoded paths where all original and landmarks cloud are located
        std::string landmarksPath = "/home/thalisson/Documents/landmarks/landmarks/" + pointType + "/" + folder;
        std::string cloudsPath = "/media/thalisson/Seagate Expansion Drive/BD Faces/Bosphorus_Original_PCD/" + folder + "/";

        std::cout << "Folder [" << folder << "]" << std::endl;

        for (const auto & entry : fs::directory_iterator(landmarksPath))
        {
            std::string landmarkPath = entry.path();
            std::string landmarkFilename = fs::path(landmarkPath).filename();
            std::string cloudPath = cloudsPath + landmarkFilename;

            if (has_suffix(landmarkFilename, ".pcd"))
            {
                if (onlyExpression && !isExpression(landmarkFilename)) {
                    continue;
                }
                if (onlyNeutral && !isNeutral(landmarkFilename)) {
                    continue;
                }
                myfile << landmarkFilename << ",";
                CloudXYZ::Ptr landmarkCloud(new CloudXYZ);
                if (pcl::io::loadPCDFile(landmarkPath, *landmarkCloud) == -1)
                {
                    myfile << "null,null,null" << std::endl;
                    continue;
                }

                CloudXYZ::Ptr cloud(new CloudXYZ);
                if (pcl::io::loadPCDFile(cloudPath, *cloud) == -1)
                {
                    myfile << "null,null,null" << std::endl;
                    continue;
                }

                myfile << cloud->points.size() << ",";

                try
                {
                    PipelineMain::run(
                        cloud,
                        filters,
                        kdtreeMethods,
                        kdtreeValues,
                        minThresholds,
                        maxThresholds);
                }
                catch (const std::exception& e) {
                   myfile << "0,0" << std::endl;
                   continue;
                }

                double lx = landmarkCloud->points[0].x;
                double ly = landmarkCloud->points[0].y;
                double lz = landmarkCloud->points[0].z;

                myfile << cloud->points.size() << ",";

                bool exists = false;
                for (int j = 0; j < cloud->points.size(); j++)
                {
                    double fx = cloud->points[j].x;
                    double fy = cloud->points[j].y;
                    double fz = cloud->points[j].z;

                    if ((nearly_equal(fx, lx, EPSILON)) && (nearly_equal(fy, ly, EPSILON)) && (nearly_equal(fz, lz, EPSILON)))
                    {
                        myfile << true;
                        exists = true;
                        break;
                    }
                }

                if (!exists)
                {
                    myfile << false;
                }

                myfile << std::endl;
            }
        }
    }

    myfile.close();
}

int main(int argc, char **argv)
{
    // As a future improvement, the parameters of each filter can be passed as configuration files, for example,
    // making the process more dynamic.
    std::vector<std::string> filters{"gaussianCurvature", "shapeIndex"};
    std::vector<std::string> kdtreeMethods{"radius", "radius"};
    std::vector<float> kdtreeValues{10, 10};
    std::vector<float> minThresholds{0.002, -1};
    std::vector<float> maxThresholds{0.009, -0.6};

    std::string pointType;
    if (pcl::console::parse_argument(argc, argv, "-pointType", pointType) == -1) {
        std::cout << "pointType parameter required!" << std::endl;
        return -1;
    }

    std::string filename;
    if (pcl::console::parse_argument(argc, argv, "-filename", filename) == -1) {
        std::cout << "filename parameter required!" << std::endl;
        return -1;
    }

    bool onlyExpression = false;
    bool onlyNeutral = false;

    if (pcl::console::find_argument(argc, argv, "-expression") >= 0) {
        onlyExpression = true;
    }

    if (pcl::console::find_argument(argc, argv, "-neutral") >= 0) {
        onlyNeutral = true;
    }

    if (onlyExpression && onlyNeutral) {
        std::cout << "Choose -expression or -neutral, not both!" << std::endl;
        return -1;
    }

    runExperiments(
        pointType,
        filename,
        onlyExpression,
        onlyNeutral,
        filters,
        kdtreeMethods,
        kdtreeValues,
        minThresholds,
        maxThresholds);
}
