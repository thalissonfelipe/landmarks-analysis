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

namespace fs = std::experimental::filesystem;

#include "../../src/PipelineMain.h"

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

constexpr double EPSILON = 0.0001; // 1e-4

bool nearly_equal(double a1, double a2, double epsilon)
{
  if (a1 == 0 && a2 == 0)
    return true;

  return std::abs(a1 - a2) < epsilon * pow (2.0, static_cast<int> (std::log2(std::max(std::abs(a1), std::abs(a2)))));
}

int main(int, char **argv)
{
    const int nFolders = 104;
    std::string pointType = argv[1]; // left-eye, right-eye, nose-tip

    std::vector<std::string> filters{"sum"};
    std::vector<std::string> kdtreeMethods{"radius"};
    std::vector<float> kdtreeValues{10};
    std::vector<float> minThresholds{30};
    std::vector<float> maxThresholds{40};

    for (int i = 0; i <= nFolders; i++)
    {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << i;
        std::string folder = "bs" + oss.str();

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
                auto totalStart = std::chrono::steady_clock::now();

                CloudXYZ::Ptr landmarkCloud(new CloudXYZ);
                if (pcl::io::loadPCDFile(landmarkPath, *landmarkCloud) == -1)
                {
                    continue;
                }

                CloudXYZ::Ptr cloud(new CloudXYZ);
                if (pcl::io::loadPCDFile(cloudPath, *cloud) == -1)
                {
                    continue;
                }

                CloudXYZ::Ptr filteredCloud(new CloudXYZ);

                std::cout << "Original cloud size: " << cloud->points.size();

                PipelineMain::runv2(
                    cloud,
                    filters,
                    kdtreeMethods,
                    kdtreeValues,
                    minThresholds,
                    maxThresholds,
                    filteredCloud);

                double lx = landmarkCloud->points[0].x;
                double ly = landmarkCloud->points[0].y;
                double lz = landmarkCloud->points[0].z;

                std::cout << " Filtered cloud size: " << cloud->points.size();
                std::cout << " Landmark in filtered cloud? ";

                // cloud, original_size, filtered_size, boolean

                for (int j = 0; j < cloud->points.size(); j++)
                {
                    double fx = cloud->points[j].x;
                    double fy = cloud->points[j].y;
                    double fz = cloud->points[j].z;

                    if ((nearly_equal(fx, lx, EPSILON)) && (nearly_equal(fy, ly, EPSILON)) && (nearly_equal(fz, lz, EPSILON)))
                    {
                        std::cout << "Yes!";
                        break;
                    }
                }

                auto totalEnd = std::chrono::steady_clock::now();
                auto totalDiff = totalEnd - totalStart;
                double executionTime = std::chrono::duration<double, std::milli>(totalDiff).count();

                std::cout << "\n\tExecution time: " << executionTime << std::endl;;
            }
        }
        break;
    }
}
