//
// Created by marcusv on 10/03/19.
//

#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include "NosetipFinder.h"

pcl::PointXYZ NosetipFinder::chooseANoseTip(CloudXYZ::Ptr inputCloud,
                                            int searchRadius,
                                            CloudsLog &cloudsLog)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    pcl::PointXYZ noseTip;

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        if (kdTree.radiusSearch(inputCloud->points[i], searchRadius, points_index_vector[i], points_rsd_vector[i]) <= 0)
        {
            throw std::runtime_error("Couldn't search points");
        }
    }

    int biggest_index = 0;

    for (int i = 1; i < inputCloud->points.size(); i++)
    {
        if (points_index_vector[i].size() > points_index_vector[biggest_index].size())
        {
            biggest_index = i;
        }
    }

    noseTip = inputCloud->points[biggest_index];

    return noseTip;
}

bool NosetipFinder::itsAGoodNoseTip(pcl::PointXYZ noseTip, float xValue, float yValue, float zValue, float maxDistance)
{
    double distance = sqrt(pow((noseTip.x - xValue), 2) + pow((noseTip.y - yValue), 2) + pow((noseTip.z - zValue), 2));
    return distance <= maxDistance;
}

bool NosetipFinder::saveNoseTip(pcl::PointXYZ noseTip, std::string filename, std::string cloudName)
{
    if (filename.substr(filename.length() - 3) == "pcd")
    {
        CloudXYZ::Ptr noseTipCloud(new CloudXYZ);

        float bad_point = std::numeric_limits<float>::quiet_NaN();
        pcl::PointXYZ nan_point = pcl::PointXYZ(bad_point, bad_point, bad_point);

        for (int i = 0; i <= 13; i++)
        {
            if (i == 13)
            {
                noseTipCloud->push_back(noseTip);
            }
            else
            {
                noseTipCloud->push_back(nan_point);
            }
        }

        pcl::io::savePCDFile(filename, *noseTipCloud);

        return true;
    }
    else if (filename.substr(filename.length() - 3) == "txt")
    {
        std::ofstream ofs;
        ofs.open(filename.c_str(), std::ios_base::app);

        if (ofs.is_open())
        {
            ofs << cloudName << " " << noseTip << std::endl;
            ofs.close();
            return true;
        }

        return false;
    }

    return true;
}