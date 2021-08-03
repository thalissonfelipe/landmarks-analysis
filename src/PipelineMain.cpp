#include <string>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include "Pipeline.h"
#include "PipelineMain.h"
#include "Utils.h"
#include "CloudsLog.h"

const std::string OUTPUT_CLOUDS_DIRECTORY = "./output_clouds/";

PipelineMainResponse PipelineMain::run(
    std::string filename,
    std::string filepath,
    std::string outputFilename,
    std::vector<std::string>& filters,
    std::vector<std::string>& kdtreeMethods,
    std::vector<float>& kdtreeValues,
    std::vector<float>& minThresholds,
    std::vector<float>& maxThresholds,
    bool saveResults
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::loadCloudFile(filepath, cloud);

    std::cout << "Número de pontos iniciais: " << cloud->points.size() << std::endl;

    std::vector<float> shapeIndexes;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    CloudsLog cloudsLog;

    for (int i = 0; i < filters.size(); i++) {
        std::cout << "Filtro aplicado: " << filters[i] << std::endl;
        std::cout << "Parâmetros:" << std::endl;
        std::cout << "\tMétodo KDTree: " << kdtreeMethods[i] << std::endl;
        std::cout << "\tValor KDTree: " << kdtreeValues[i] << std::endl;
        std::cout << "\tThreshold mínimo: " << minThresholds[i] << std::endl;
        std::cout << "\tThreshold máximo: " << maxThresholds[i] << std::endl;

        if (filters[i] == "shapeIndex")
        {
            Pipeline::filterByShapeIndex(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                shapeIndexes);

            // not using for now
            shapeIndexes.clear();
        }
        else if (filters[i] == "gaussianCurvature")
        {
            Pipeline::filterByGaussianCurvature(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "gaussianCurvature")
        {
            Pipeline::filterByGaussianCurvature(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "principalCurvatureRatio")
        {
            Pipeline::filterByPrincipalCurvatureRatio(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "meanCurvature")
        {
            Pipeline::filterByMeanCurvature(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "curvedness")
        {
            Pipeline::filterByCurvedness(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else {
            Pipeline::filterByGeometricFeatures(
                cloud,
                filters[i],
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud);
        }

        if (filteredCloud->points.size() == 0)
        {
            throw std::runtime_error("Número de pontos após filtragens é 0! Por favor, escolha outros parâmetros.");
        }

        if (saveResults)
        {
            std::string folder = filename.substr(0, filename.length()-4);
            std::time_t t = std::time(0);
            Utils::saveCloud(OUTPUT_CLOUDS_DIRECTORY+folder+"/"+filters[i]+"_"+folder+"_"+std::to_string(t)+".pcd", filteredCloud);
        }

        std::cout << "Número de pontos após filtragem: " << filteredCloud->points.size() << " (" << filters[i] << ")" << std::endl;
        cloudsLog.add(filters[i], filteredCloud);
        *cloud = *filteredCloud;
        filteredCloud->points.clear();
    }

    PipelineMainResponse response;
    response.cloudsLog = cloudsLog;
    response.lastFilteredCloud = cloud;

    return response;
}

void PipelineMain::run(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<std::string>& filters,
    std::vector<std::string>& kdtreeMethods,
    std::vector<float>& kdtreeValues,
    std::vector<float>& minThresholds,
    std::vector<float>& maxThresholds
)
{
    std::vector<float> shapeIndexes;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < filters.size(); i++) {
        if (filters[i] == "shapeIndex")
        {
            Pipeline::filterByShapeIndex(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                shapeIndexes);

            // not using for now
            shapeIndexes.clear();
        }
        else if (filters[i] == "gaussianCurvature")
        {
            Pipeline::filterByGaussianCurvature(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "gaussianCurvature")
        {
            Pipeline::filterByGaussianCurvature(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "principalCurvatureRatio")
        {
            Pipeline::filterByPrincipalCurvatureRatio(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "meanCurvature")
        {
            Pipeline::filterByMeanCurvature(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else if (filters[i] == "curvedness")
        {
            Pipeline::filterByCurvedness(
                cloud,
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud,
                outputPrincipalCurvaturesCloud);

            outputPrincipalCurvaturesCloud->points.clear();
        }
        else {
            Pipeline::filterByGeometricFeatures(
                cloud,
                filters[i],
                kdtreeMethods[i],
                kdtreeValues[i],
                minThresholds[i],
                maxThresholds[i],
                filteredCloud);
        }

        if (filteredCloud->points.size() == 0)
        {
            throw std::runtime_error("Número de pontos após filtragens é 0! Por favor, escolha outros parâmetros.");
        }

        *cloud = *filteredCloud;
        filteredCloud->points.clear();
    }
}