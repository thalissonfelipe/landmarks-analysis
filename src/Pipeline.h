#ifndef PIPELINE_H
#define PIPELINE_H

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Pipeline
{
public:
    Pipeline(){};
    ~Pipeline(){};

    void static normalComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        pcl::PointCloud<pcl::Normal>::Ptr outputNormalCloud);

    void static principalCurvaturesComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        pcl::PointCloud<pcl::Normal>::Ptr normalInputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud);

    void static shapeIndexComputation(
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud,
        std::vector<float>& outputShapeIndexes,
        std::vector<int>& nanIndices);

    void static geometricFeatureSumComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureOmnivarianceComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureEigenentropyComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureAnisotropyComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeaturePlanarityComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureLinearityComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureSphericityComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureSurfaceVariationComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static geometricFeatureVerticalityComputation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
        std::string kdtreeMethod,
        float kdtreeValue,
        std::vector<double>& output);

    void static thresholdByShapeIndex(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
        std::vector<float> shapeIndexes,
        float thresholdMin,
        float thresholdMax,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud,
        std::vector<float>& outputShapeIndexes);

    void static thresholdByGaussianCurvature(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &inputPrincipalCurvaturesCloud,
        float thresholdMin,
        float thresholdMax,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud,
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &outputPrincipalCurvaturesCloud);

    void static thresholdByGeometricFeature(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
        std::string feature,
        std::string kdtreeMethod,
        float kdtreeValue,
        float thresholdMin,
        float thresholdMax,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud);
};

#endif // PIPELINE_H
