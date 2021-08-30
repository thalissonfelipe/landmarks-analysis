//
// Created by marcusv on 10/03/19.
//

#ifndef NOSETIP_FINDER_COMPUTATION_H
#define NOSETIP_FINDER_COMPUTATION_H

#include <pcl/point_types.h>
#include <string.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

class Computation
{
    typedef pcl::PointCloud<pcl::PointXYZ>            CloudXYZ;
    typedef pcl::PointCloud<pcl::Normal>              CloudNormal;
    typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

    public:

        void static normalComputation
        (
            CloudXYZ::Ptr inputCloud,
            std::string radiusOrKSearch,
            float radiusOrK,
            CloudNormal::Ptr outputNormalCloud
        );

        void static principalCurvaturesComputation
        (
            CloudXYZ::Ptr inputCloud,
            CloudNormal::Ptr normalInputCloud,
            std::string radiusOrKSearch,
            int radiusOrK,
            CloudPC::Ptr outputPrincipalCurvaturesCloud
        );

        void static shapeIndexComputation
        (
            CloudPC::Ptr principalCurvaturesCloud,
            std::vector<float>& outputShapeIndexes,
            std::vector<int>& nanIndices
        );

        float static findMaxValueInPointCloud(CloudXYZ::Ptr inputCloud, char axis);

        float static findMinValueInPointCloud(CloudXYZ::Ptr inputCloud, char axis);

        std::vector<int> static findKPointsWithLargestGaussianCurvatures
        (
            CloudXYZ &inputCloud,
            CloudPC &inputPrincipalCurvaturesCloud,
            int kPoints
        );

        void static thresholdByShapeIndex(
            CloudXYZ::Ptr &inputCloud,
            std::vector<float> shapeIndexes,
            float thresholdMin,
            float thresholdMax,
            CloudXYZ::Ptr &outputCloud,
            std::vector<float> outputShapeIndexes);

        void static thresholdByGaussianCurvature(
            CloudXYZ::Ptr &inputCloud,
            CloudPC::Ptr &inputPrincipalCurvaturesCloud,
            float thresholdMin,
            CloudXYZ::Ptr &outputCloud,
            CloudPC::Ptr &outputPrincipalCurvaturesCloud);

        void static thresholdByShapeIndexAndGaussianCurvature(
            CloudXYZ::Ptr &inputCloud,
            std::vector<float> shapeIndexes,
            CloudPC::Ptr &inputPrincipalCurvaturesCloud,
            float thresholdShapeIndexMin,
            float thresholdShapeIndexMax,
            float thresholdPrincipalCurvatureMin,
            CloudXYZ::Ptr &outputCloud,
            std::vector<float> outputShapeIndexes,
            CloudPC::Ptr &outputPrincipalCurvaturesCloud);

        void static removeNonExistingIndices(CloudXYZ::Ptr &inputCloud, std::vector<int> indicesToKeep);

        void static removeNonExistingIndices(CloudPC::Ptr &inputCloud, std::vector<int> indicesToKeep);
};


#endif //NOSETIP_FINDER_COMPUTATION_H
