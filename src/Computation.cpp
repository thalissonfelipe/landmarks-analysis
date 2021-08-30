//
// Created by marcusv on 10/03/19.
//

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include "Computation.h"

void Computation::normalComputation(
    CloudXYZ::Ptr inputCloud,
    std::string radiusOrKSearch,
    float radiusOrK,
    CloudNormal::Ptr outputNormalCloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    normalEstimation.setInputCloud(inputCloud);
    normalEstimation.setSearchMethod(tree);

    if (radiusOrKSearch == "radius")
    {
        normalEstimation.setRadiusSearch(radiusOrK);
    }
    else if (radiusOrKSearch == "k")
    {
        normalEstimation.setKSearch(radiusOrK);
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in normalComputation");
    }

    normalEstimation.compute(*outputNormalCloud);
}

void Computation::principalCurvaturesComputation(CloudXYZ::Ptr inputCloud,
                                                 CloudNormal::Ptr normalInputCloud,
                                                 std::string radiusOrKSearch, int radiusOrK,
                                                 CloudPC::Ptr
                                                     outputPrincipalCurvaturesCloud)
{
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    principalCurvaturesEstimation.setInputCloud(inputCloud);
    principalCurvaturesEstimation.setInputNormals(normalInputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    principalCurvaturesEstimation.setSearchMethod(tree);

    if (radiusOrKSearch == "radius")
    {
        principalCurvaturesEstimation.setRadiusSearch(radiusOrK);
    }
    else
    {
        if (radiusOrKSearch == "k")
        {
            principalCurvaturesEstimation.setKSearch(radiusOrK);
        }
        else
        {
            throw std::runtime_error("Use 'radius' or 'k' in principalCurvaturesComputation");
        }
    }

    principalCurvaturesEstimation.compute(*outputPrincipalCurvaturesCloud);
}

void Computation::shapeIndexComputation(CloudPC::Ptr principalCurvaturesCloud,
                                        std::vector<float> &outputShapeIndexes,
                                        std::vector<int> &notNaNIndices)
{
    float shapeIndex;
    float k1;
    float k2;
    float atg;

    for (int i = 0; i < principalCurvaturesCloud->points.size(); i++)
    {
        k1 = principalCurvaturesCloud->points[i].pc1;
        k2 = principalCurvaturesCloud->points[i].pc2;

        if (k1 >= k2)
        {
            atg = atan((k2 + k1) / (k2 - k1));
        }
        else
        {
            atg = atan((k1 + k2) / (k1 - k2));
        }

        shapeIndex = (2 / M_PI) * (atg);

        if (!std::isnan(shapeIndex))
        {
            outputShapeIndexes.push_back(shapeIndex);
            notNaNIndices.push_back(i);
        }
    }
}

float Computation::findMaxValueInPointCloud(CloudXYZ::Ptr inputCloud, char axis)
{
    float maxValue;

    if (axis == 'x')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (maxValue < inputCloud->points[i].x)
                {
                    maxValue = inputCloud->points[i].x;
                }
            }
            else
            {
                maxValue = inputCloud->points[i].x;
            }
        }

        return maxValue;
    }
    else if (axis == 'y')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (maxValue < inputCloud->points[i].y)
                {
                    maxValue = inputCloud->points[i].y;
                }
            }
            else
            {
                maxValue = inputCloud->points[i].y;
            }
        }

        return maxValue;
    }
    else if (axis == 'z')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (maxValue < inputCloud->points[i].z)
                {
                    maxValue = inputCloud->points[i].z;
                }
            }
            else
            {
                maxValue = inputCloud->points[i].z;
            }
        }

        return maxValue;
    }

    throw std::runtime_error("Use 'x', 'y' or 'z' in findMaxValueInPointCloud");
}

float Computation::findMinValueInPointCloud(CloudXYZ::Ptr inputCloud, char axis)
{
    float minValue;

    if (axis == 'x')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (minValue > inputCloud->points[i].x)
                {
                    minValue = inputCloud->points[i].x;
                }
            }
            else
            {
                minValue = inputCloud->points[i].x;
            }
        }

        return minValue;
    }
    else if (axis == 'y')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (minValue > inputCloud->points[i].y)
                {
                    minValue = inputCloud->points[i].y;
                }
            }
            else
            {
                minValue = inputCloud->points[i].y;
            }
        }

        return minValue;
    }
    else if (axis == 'z')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (minValue < inputCloud->points[i].z)
                {
                    minValue = inputCloud->points[i].z;
                }
            }
            else
            {
                minValue = inputCloud->points[i].z;
            }
        }

        return minValue;
    }

    throw std::runtime_error("Use 'x', 'y' or 'z' in findMaxValueInPointCloud");
}

std::vector<int> Computation::findKPointsWithLargestGaussianCurvatures(CloudXYZ &inputCloud,
                                                                       CloudPC
                                                                       &inputPrincipalCurvaturesCloud,
                                                                       int kPoints)
{
    std::vector<int> largestGaussianCurvaturesIndices;

    return std::vector<int>();
}

void Computation::thresholdByShapeIndex(CloudXYZ::Ptr &inputCloud,
                                        std::vector<float> shapeIndexes,
                                        float thresholdMin,
                                        float thresholdMax,
                                        CloudXYZ::Ptr &outputCloud,
                                        std::vector<float> outputShapeIndexes)
{
    if (inputCloud->points.size() != shapeIndexes.size())
    {
        throw std::runtime_error("Input Cloud and Shape Indexes Vector must have the same size in thresholdByShapeIndex");
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

void Computation::thresholdByGaussianCurvature(CloudXYZ::Ptr &inputCloud,
                                               CloudPC::Ptr &inputPrincipalCurvaturesCloud,
                                               float thresholdMin,
                                               CloudXYZ::Ptr &outputCloud,
                                               CloudPC::Ptr &outputPrincipalCurvaturesCloud)
{
    if (inputCloud->points.size() != inputPrincipalCurvaturesCloud->points.size())
    {
        throw std::runtime_error("Input Cloud and Principal Curvatures Vector must have the same size in thresholdByGaussianCurvature");
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

void Computation::thresholdByShapeIndexAndGaussianCurvature(CloudXYZ::Ptr &inputCloud,
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

void Computation::removeNonExistingIndices(CloudXYZ::Ptr &inputCloud, std::vector<int> indicesToKeep)
{
    CloudXYZ::Ptr tempCloud(new CloudXYZ);
    pcl::copyPointCloud(*inputCloud, indicesToKeep, *tempCloud);
    *inputCloud = *tempCloud;
}

void Computation::removeNonExistingIndices(CloudPC::Ptr &inputCloud, std::vector<int> indicesToKeep)
{
    CloudPC::Ptr tempCloud(new CloudPC);
    *tempCloud = *inputCloud;

    inputCloud->points.clear();

    for (int i = 0; i < indicesToKeep.size(); i++)
    {
        inputCloud->points.push_back(tempCloud->points[indicesToKeep[i]]);
    }
}
