//
// Created by marcusv on 10/03/19.
//

#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include "Anisotropy.h"
#include "EyeRightCorner.h"

void EyeRightCornerFinder::thresholdByShapeIndex(CloudXYZ::Ptr &inputCloud,
                                          std::vector<float> shapeIndexes,
                                          float thresholdMin,
                                          float thresholdMax,
                                          CloudXYZ::Ptr &outputCloud,
                                          std::vector<float> outputShapeIndexes)
{
    if (inputCloud->points.size() != shapeIndexes.size())
    {
        throw std::runtime_error("Input Cloud and Shape Indexes Vector must have the same size in thresholdByShapeIndex");
        return;
    }

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        if ((shapeIndexes[i] > thresholdMin) && (shapeIndexes[i] < thresholdMax))
        {
            outputCloud->push_back(inputCloud->points[i]);
            outputShapeIndexes.push_back(shapeIndexes[i]);
        }
    }
}

void EyeRightCornerFinder::thresholdByGaussianCurvature(CloudXYZ::Ptr &inputCloud,
                                                 CloudPC::Ptr &inputPrincipalCurvaturesCloud,
                                                 float thresholdMin,
                                                 CloudXYZ::Ptr &outputCloud,
                                                 CloudPC::Ptr &outputPrincipalCurvaturesCloud)
{
    if (inputCloud->points.size() != inputPrincipalCurvaturesCloud->points.size())
    {
        throw std::runtime_error("Input Cloud and Principal Curvatures Vector must have the same size in thresholdByGaussianCurvature");
        return;
    }

    float k1, k2;

    for (int i = 0; i < inputPrincipalCurvaturesCloud->size(); i++)
    {
        k1 = inputPrincipalCurvaturesCloud->points[i].pc1;
        k2 = inputPrincipalCurvaturesCloud->points[i].pc2;

        if (k1 * k2 > thresholdMin)
        {
            outputCloud->push_back(inputCloud->points[i]);
            outputPrincipalCurvaturesCloud->push_back(inputPrincipalCurvaturesCloud->points[i]);
        }
    }
}

void EyeRightCornerFinder::thresholdByShapeIndexAndGaussianCurvature(CloudXYZ::Ptr &inputCloud,
                                                              std::vector<float> shapeIndexes,
                                                              CloudPC::Ptr &inputPrincipalCurvaturesCloud,
                                                              float thresholdShapeIndexMin,
                                                              float thresholdShapeIndexMax,
                                                              float thresholdPrincipalCurvatureMin,
                                                              CloudXYZ::Ptr &outputCloud,
                                                              std::vector<float> outputShapeIndexes,
                                                              CloudPC::Ptr &outputPrincipalCurvaturesCloud)
{
    if (
        (inputCloud->points.size() != shapeIndexes.size()) ||
        (inputCloud->points.size() != inputPrincipalCurvaturesCloud->points.size()))
    {
        throw std::runtime_error("Input Cloud, Shape Indexes Vector and Principal Curvatures Cloud must have the same size.");
    }

    float k1, k2, gc;

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        k1 = inputPrincipalCurvaturesCloud->points[i].pc1;
        k2 = inputPrincipalCurvaturesCloud->points[i].pc2;

        gc = k1 * k2;
        
        if (gc > thresholdPrincipalCurvatureMin) 
        {
            if ((shapeIndexes[i] > thresholdShapeIndexMin) && (shapeIndexes[i] < thresholdShapeIndexMax))
            {
                outputCloud->push_back(inputCloud->points[i]);
                outputPrincipalCurvaturesCloud->push_back(inputPrincipalCurvaturesCloud->points[i]);
                outputShapeIndexes.push_back(shapeIndexes[i]);
            }
        }
    }
}

pcl::PointXYZ EyeRightCornerFinder::choosePoint(CloudXYZ::Ptr inputCloud,
                                            int searchRadius,
                                            CloudsLog &cloudsLog)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree, kdtree;
    // kdTree.setInputCloud(inputCloud);
    kdtree.setInputCloud(inputCloud);

    pcl::PointXYZ eyerCorner;

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    // for (int i = 0; i < inputCloud->points.size(); i++)
    // {
    //     if (kdTree.radiusSearch(inputCloud->points[i], searchRadius, points_index_vector[i], points_rsd_vector[i]) > 0)
    //     {
    //         // std::cout << "Searching in the neighboorhod of the point " << i << std::endl;
    //     }
    //     else
    //     {
    //         throw std::runtime_error("Couldn't search points");
    //         return pcl::PointXYZ();
    //     }
    // }

    std::vector<GeometricFeatures> gfs;

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        // std::vector<int> pointIdxKNNSearch;
        // std::vector<float> pointKNNSquaredDistance;

        if (kdtree.radiusSearch(inputCloud->points[i], 13, points_index_vector[i], points_rsd_vector[i]) > 0)
        {
            GeometricFeatures gf;
            CloudXYZ::Ptr filteredCloud(new CloudXYZ);

            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            gf = Anisotropy::geometricFeatures(*filteredCloud);
            gfs.push_back(gf);
            // std::cout << Anisotropy::printGeometricFeatures(gf) << std::endl;
        }
        else
        {
            throw std::runtime_error("Couldn't search points");
            return pcl::PointXYZ();
        }
        
        // std::cout << "points_index_vector.size: " << points_index_vector[i].size() << std::endl;
        // if (points_index_vector[i].size() > points_index_vector[biggest_index].size())
        // {
        //     biggest_index = i;
        // }
    }

    CloudXYZ::Ptr filteredCloud(new CloudXYZ);
    std::vector<int> indices;
    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        // radius 13
        // gfs[i].gf04 > 0.86
        // gfs[i].gf09 > 0.2
        // gfs[i].gf06 < 0.4
        if (gfs[i].gf04 > 0.86) {
            std::cout << Anisotropy::printGeometricFeatures(gfs[i]) << std::endl;
            filteredCloud->points.push_back(inputCloud->points[i]);
            indices.push_back(i);
        }
    }
    std::cout << "Filtered cloud size: " << filteredCloud->points.size() << std::endl;
    cloudsLog.add("4.1 Anisotropy (radius13)", filteredCloud);

    CloudXYZ::Ptr filteredCloud1(new CloudXYZ);
    indices.clear();
    for (int i = 0; i < filteredCloud->points.size(); i++)
    {
        if (gfs[i].gf03 > -280 && gfs[i].gf03 < -100) {
            filteredCloud1->points.push_back(filteredCloud->points[i]);
            indices.push_back(i);
        }
    }
    std::cout << "Filtered cloud size: " << filteredCloud1->points.size() << std::endl;
    cloudsLog.add("4.2 Anisotropy (radius13)", filteredCloud1);
    
    int selectedIndex = 0;
    // for (int i = 1; i < indices.size(); i++)
    // {
    //     if (points_index_vector[i].size() > points_index_vector[selectedIndex].size())
    //     {
    //         selectedIndex = i;
    //     }
    // }

    // kdTree.setInputCloud(filteredCloud1);
    // std::vector<std::vector<int>> points_index_vector1(filteredCloud1->points.size());
    // std::vector<std::vector<float>> points_rsd_vector1(filteredCloud1->points.size());

    // for (int i = 0; i < filteredCloud1->points.size(); i++)
    // {
    //     if (kdtree.radiusSearch(filteredCloud1->points[i], 13, points_index_vector1[i], points_rsd_vector1[i]) <= 0)
    //     {
    //         throw std::runtime_error("Couldn't search points");
    //         return pcl::PointXYZ();
    //     }
    // }

    // for (int i = 1; i < filteredCloud1->points.size(); i++)
    // {
    //    if (points_index_vector1[i].size() < points_index_vector1[selectedIndex].size())
    //     {
    //         selectedIndex = i;
    //     }
    // }

    // eyerCorner = filteredCloud1->points[selectedIndex];

    kdTree.setInputCloud(filteredCloud);
    std::vector<std::vector<int>> points_index_vector1(filteredCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector1(filteredCloud->points.size());

    for (int i = 0; i < filteredCloud->points.size(); i++)
    {
        if (kdtree.radiusSearch(filteredCloud->points[i], 13, points_index_vector1[i], points_rsd_vector1[i]) <= 0)
        {
            throw std::runtime_error("Couldn't search points");
            return pcl::PointXYZ();
        }
    }

    for (int i = 1; i < filteredCloud->points.size(); i++)
    {
       if (points_index_vector1[i].size() < points_index_vector1[selectedIndex].size())
        {
            selectedIndex = i;
        }
    }

    eyerCorner = filteredCloud->points[selectedIndex];

    return eyerCorner;
}

bool EyeRightCornerFinder::itsAGoodPoint(pcl::PointXYZ noseTip, float xValue, float yValue, float zValue, float maxDistance)
{
    double distance = sqrt(pow((noseTip.x - xValue), 2) + pow((noseTip.y - yValue), 2) + pow((noseTip.z - zValue), 2));
    return distance <= maxDistance;
}

void EyeRightCornerFinder::removeNonExistingIndices(CloudXYZ::Ptr &inputCloud, std::vector<int> indicesToKeep)
{
    CloudXYZ::Ptr tempCloud(new CloudXYZ);
    pcl::copyPointCloud(*inputCloud, indicesToKeep, *tempCloud);
    *inputCloud = *tempCloud;
}

void EyeRightCornerFinder::removeNonExistingIndices(CloudPC::Ptr &inputCloud, std::vector<int> indicesToKeep)
{
    CloudPC::Ptr tempCloud(new CloudPC);
    *tempCloud = *inputCloud;

    inputCloud->points.clear();

    for (int i = 0; i < indicesToKeep.size(); i++)
    {
        inputCloud->points.push_back(tempCloud->points[indicesToKeep[i]]);
    }
}

bool EyeRightCornerFinder::savePoint(pcl::PointXYZ noseTip, std::string filename, std::string cloudName)
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
    else
    {
        if (filename.substr(filename.length() - 3) == "txt")
        {
            std::ofstream ofs;
            ofs.open(filename.c_str(), std::ios_base::app);
            if (ofs.is_open())
            {
                ofs << cloudName << " " << noseTip << std::endl;
                ofs.close();
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    return true;
}