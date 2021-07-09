//
// Created by Thalisson on 01/05/2021.
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
                                            float gfSearchRadius,
                                            std::string features,
                                            std::string featuresThreshold,
                                            CloudsLog &cloudsLog)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree, kdtree;

    kdtree.setInputCloud(inputCloud);

    pcl::PointXYZ eyeCorner;

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    std::vector<GeometricFeatures> gfs;

    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        kdtree.radiusSearch(inputCloud->points[i], gfSearchRadius, points_index_vector[i], points_rsd_vector[i]);

        GeometricFeatures gf;
        CloudXYZ::Ptr filteredCloud(new CloudXYZ);

        pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

        gf = Anisotropy::geometricFeatures(*filteredCloud);
        gfs.push_back(gf);
    }

    std::stringstream ss(features);
    std::vector<std::string> feats;
    std::vector<float> thresholds;

    std::string token;

    while(std::getline(ss, token, ',')) {
        feats.push_back(token);
    }

    ss.str("");
    ss.clear();
    ss << featuresThreshold;

    while(std::getline(ss, token, ',')) {
        thresholds.push_back(std::stof(token));
    }

    CloudXYZ::Ptr filteredCloud(new CloudXYZ);
    std::vector<int> indices;

    int cloudsLogCounter = 1;

    // iterating over features (Anisotropy, Sum...)
    for (int i = 0; i < feats.size(); i++)
    {
        for (int j = 0; j < inputCloud->points.size(); j++)
        {
            if (feats[i].compare("Somatório") == 0)
            {
                if (gfs[j].gf01 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Autoentropia") == 0) {
                if (gfs[j].gf03 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Anisotropia") == 0) {
                if (gfs[j].gf04 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Planaridade") == 0)
            {
                if (gfs[j].gf05 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Linearidade") == 0) {
                if (gfs[j].gf06 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Variação de superfı́cie") == 0) {
                if (gfs[j].gf07 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Esfericidade") == 0)
            {
                if (gfs[j].gf08 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
            else if (feats[i].compare("Verticalidade") == 0) {
                if (gfs[j].gf09 > thresholds[i]) {
                    filteredCloud->points.push_back(inputCloud->points[j]);
                }
            }
        }

        std::cout << "Filtered cloud size after " << feats[i] << ": " << filteredCloud->points.size() << std::endl;
        cloudsLog.add("4." + std::to_string(cloudsLogCounter) + " " + feats[i], filteredCloud);
        cloudsLogCounter++;

        if (i == feats.size() - 1)
            break;

        *inputCloud = *filteredCloud;
        filteredCloud->points.clear();
    }

    if (filteredCloud->points.size() == 0)
    {
       throw std::runtime_error("Número de pontos após a filtragem é 0.");
    }

    kdTree.setInputCloud(filteredCloud);

    std::vector<std::vector<int>> points_index_vector1(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector1(inputCloud->points.size());

    for (int i = 0; i < filteredCloud->points.size(); i++)
    {
        kdtree.radiusSearch(filteredCloud->points[i], 13, points_index_vector1[i], points_rsd_vector1[i]);
    }

    int selectedIndex = 0;

    for (int i = 1; i < filteredCloud->points.size(); i++)
    {
        if (points_index_vector1[i].size() > points_index_vector1[selectedIndex].size())
        {
            selectedIndex = i;
        }
    }

    eyeCorner = filteredCloud->points[selectedIndex];

    return eyeCorner;
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