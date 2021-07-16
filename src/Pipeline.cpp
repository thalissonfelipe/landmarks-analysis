#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>

#include "Utils.h"
#include "Pipeline.h"
#include "Computation.h"
#include "GeometricFeatures.h"

void Pipeline::normalComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    pcl::PointCloud<pcl::Normal>::Ptr &outputNormalCloud
)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);

    normalEstimation.setInputCloud(inputCloud);
    normalEstimation.setSearchMethod(kdTree);

    if (kdtreeMethod == "radius")
    {
        normalEstimation.setRadiusSearch(kdtreeValue);
    }
    else if (kdtreeMethod == "k")
    {
        normalEstimation.setKSearch(kdtreeValue);
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in normal computation");
    }

    normalEstimation.compute(*outputNormalCloud);
}

void Pipeline::principalCurvaturesComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    pcl::PointCloud<pcl::Normal>::Ptr &normalInputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &outputPrincipalCurvaturesCloud
)
{
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    principalCurvaturesEstimation.setInputCloud(inputCloud);
    principalCurvaturesEstimation.setInputNormals(normalInputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    principalCurvaturesEstimation.setSearchMethod(kdTree);

    if (kdtreeMethod == "radius")
    {
        principalCurvaturesEstimation.setRadiusSearch(kdtreeValue);
    }
    else if (kdtreeMethod == "k")
    {
        principalCurvaturesEstimation.setKSearch(kdtreeValue);
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in principal curvatures computation");
    }

    principalCurvaturesEstimation.compute(*outputPrincipalCurvaturesCloud);
}

void Pipeline::shapeIndexComputation(
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &principalCurvaturesCloud,
    std::vector<float>& outputShapeIndexes,
    std::vector<int>& nanIndices
)
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
            nanIndices.push_back(i);
        }
    }
}

void Pipeline::thresholdByShapeIndex(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    std::vector<float> shapeIndexes,
    float thresholdMin,
    float thresholdMax,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud,
    std::vector<float>& outputShapeIndexes
)
{
    if (inputCloud->points.size() != shapeIndexes.size())
    {
        throw std::runtime_error("Input Cloud and Shape Indexes Vector must have the same size in thresholdByShapeIndex function");
    }

    for (int i = 0; i < shapeIndexes.size(); i++)
    {
        if ((shapeIndexes[i] >= thresholdMin) && (shapeIndexes[i] <= thresholdMax))
        {
            outputCloud->push_back(inputCloud->points[i]);
            outputShapeIndexes.push_back(shapeIndexes[i]);
        }
    }
}

void Pipeline::thresholdByGaussianCurvature(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &inputPrincipalCurvaturesCloud,
    float thresholdMin,
    float thresholdMax,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud,
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &outputPrincipalCurvaturesCloud
)
{
    if (inputCloud->points.size() != inputPrincipalCurvaturesCloud->points.size())
    {
        throw std::runtime_error("Input Cloud and Principal Curvatures Vector must have the same size in thresholdByGaussianCurvature function");
    }

    float k1, k2, gc;

    for (int i = 0; i < inputPrincipalCurvaturesCloud->size(); i++)
    {
        k1 = inputPrincipalCurvaturesCloud->points[i].pc1;
        k2 = inputPrincipalCurvaturesCloud->points[i].pc2;
        gc = k1 * k2;

        if (gc >= thresholdMin && gc <= thresholdMax)
        {
            outputCloud->push_back(inputCloud->points[i]);
            outputPrincipalCurvaturesCloud->push_back(inputPrincipalCurvaturesCloud->points[i]);
        }
    }
}

void Pipeline::geometricFeatureSumComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureSumComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf01);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureSumComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf01);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in sum computation");
    }
}

void Pipeline::geometricFeatureOmnivarianceComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureOmnivarianceComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf02);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureOmnivarianceComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf02);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in omnivariance computation");
    }
}

void Pipeline::geometricFeatureEigenentropyComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureEigenentropyComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf03);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureEigenentropyComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf03);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in eigenentropy computation");
    }
}

void Pipeline::geometricFeatureAnisotropyComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureAnisotropyComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf04);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureAnisotropyComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf04);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in anisotropy computation");
    }
}

void Pipeline::geometricFeaturePlanarityComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeaturePlanarityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf05);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeaturePlanarityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf05);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in planarity computation");
    }
}

void Pipeline::geometricFeatureLinearityComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureLinearityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf06);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureLinearityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf06);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in linearity computation");
    }
}

void Pipeline::geometricFeatureSphericityComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureSphericityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf07);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureSphericityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf07);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in sphericity computation");
    }
}

void Pipeline::geometricFeatureSurfaceVariationComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureSurfaceVariationComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf08);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureSurfaceVariationComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf08);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in surface variation computation");
    }
}

void Pipeline::geometricFeatureVerticalityComputation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    std::vector<double>& output,
    std::vector<int>& indicesToKeep
)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    std::vector<std::vector<int>> points_index_vector(inputCloud->points.size());
    std::vector<std::vector<float>> points_rsd_vector(inputCloud->points.size());

    if (kdtreeMethod == "radius")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.radiusSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (radius): geometricFeatureVerticalityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf09);
                indicesToKeep.push_back(i);
            }
        }
    }
    else if (kdtreeMethod == "k")
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (kdTree.nearestKSearch(inputCloud->points[i], kdtreeValue, points_index_vector[i], points_rsd_vector[i]) <= 0)
            {
                throw std::runtime_error("Could not search points (k neighbors): geometricFeatureVerticalityComputation");
            }

            GeometricFeatures gf;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*inputCloud, points_index_vector[i], *filteredCloud);

            if (GeometricFeaturesComputation::geometricFeatures(*filteredCloud, &gf))
            {
                output.push_back(gf.gf09);
                indicesToKeep.push_back(i);
            }
        }
    }
    else
    {
        throw std::runtime_error("Use 'radius' or 'k' in verticality computation");
    }
}

void Pipeline::thresholdByGeometricFeature(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
    std::string feature,
    std::string kdtreeMethod,
    float kdtreeValue,
    float thresholdMin,
    float thresholdMax,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud
)
{
    std::vector<double> gfValues;
    std::vector<int> indicesToKeep;

    if (feature == "sum")
    {
        Pipeline::geometricFeatureSumComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "omnivariance")
    {
        Pipeline::geometricFeatureOmnivarianceComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "eigenentropy")
    {
        Pipeline::geometricFeatureEigenentropyComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "anisotropy")
    {
        Pipeline::geometricFeatureAnisotropyComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "planarity")
    {
        Pipeline::geometricFeaturePlanarityComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "linearity")
    {
        Pipeline::geometricFeatureLinearityComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "sphericity")
    {
        Pipeline::geometricFeatureSphericityComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "surfaceVariation")
    {
        Pipeline::geometricFeatureSurfaceVariationComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else if (feature == "verticality")
    {
        Pipeline::geometricFeatureVerticalityComputation(inputCloud, kdtreeMethod, kdtreeValue, gfValues, indicesToKeep);
    }
    else {
        throw std::runtime_error("Invalid feature!");
    }

    Computation::removeNonExistingIndices(inputCloud, indicesToKeep);

    if (inputCloud->points.size() != gfValues.size())
    {
        throw std::runtime_error("Input Cloud and gfValues Vector must have the same size in thresholdByGeometricFeature function");
    }

    for (int i = 0; i < gfValues.size(); i++)
    {
        if (gfValues[i] >= thresholdMin && gfValues[i] <= thresholdMax)
        {
            outputCloud->push_back(inputCloud->points[i]);
        }
    }
}

void Pipeline::filterByGeometricFeatures(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    std::string feature,
    std::string kdtreeMethod,
    float kdtreeValue,
    float thresholdMin,
    float thresholdMax,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

    Pipeline::thresholdByGeometricFeature(
        cloud,
        feature,
        kdtreeMethod,
        kdtreeValue,
        thresholdMin,
        thresholdMax,
        outputCloud);
}

void Pipeline::filterByShapeIndex(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    float thresholdMin,
    float thresholdMax,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& outputCloud,
    std::vector<float>& outputShapeIndexes
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);
    indices.clear();

    pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr filteredNormalCloud(new pcl::PointCloud<pcl::Normal>);

    Pipeline::normalComputation(filteredCloud, kdtreeMethod, kdtreeValue, normalCloud);
    pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);

    Computation::removeNonExistingIndices(filteredCloud, indices);

    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    Pipeline::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, kdtreeMethod, kdtreeValue, principalCurvaturesCloud);

    std::vector<float> shapeIndexes;
    std::vector<int> nanIndices;
    Pipeline::shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, nanIndices);

    if (nanIndices.size() != filteredCloud->points.size() || nanIndices.size() != principalCurvaturesCloud->points.size())
    {
        Computation::removeNonExistingIndices(filteredCloud, nanIndices);
        Computation::removeNonExistingIndices(principalCurvaturesCloud, nanIndices);
    }

    Pipeline::thresholdByShapeIndex(
        filteredCloud,
        shapeIndexes,
        thresholdMin,
        thresholdMax,
        outputCloud,
        outputShapeIndexes);
}

void Pipeline::filterByGaussianCurvature(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    std::string kdtreeMethod,
    float kdtreeValue,
    float thresholdMin,
    float thresholdMax,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud,
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &outputPrincipalCurvaturesCloud
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);
    indices.clear();

    pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr filteredNormalCloud(new pcl::PointCloud<pcl::Normal>);

    Pipeline::normalComputation(filteredCloud, kdtreeMethod, kdtreeValue, normalCloud);
    pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);

    Computation::removeNonExistingIndices(filteredCloud, indices);

    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    Pipeline::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, kdtreeMethod, kdtreeValue, principalCurvaturesCloud);

    Pipeline::thresholdByGaussianCurvature(
        filteredCloud,
        principalCurvaturesCloud,
        thresholdMin,
        thresholdMax,
        outputCloud,
        outputPrincipalCurvaturesCloud);
}
