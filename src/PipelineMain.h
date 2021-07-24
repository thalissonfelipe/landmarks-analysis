#ifndef PIPELINE_MAIN_H
#define PIPELINE_MAIN_H

#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "CloudsLog.h"

struct PipelineMainResponse
{
    CloudsLog cloudsLog;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastFilteredCloud;
};

class PipelineMain
{
public:
    PipelineMain(){};
    ~PipelineMain(){};

    PipelineMainResponse static run(
        std::string filename,
        std::string outputFilename,
        std::vector<std::string>& filters,
        std::vector<std::string>& kdtreeMethods,
        std::vector<float>& kdtreeValues,
        std::vector<float>& minThresholds,
        std::vector<float>& maxThresholds);

    void static run(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        std::vector<std::string>& filters,
        std::vector<std::string>& kdtreeMethods,
        std::vector<float>& kdtreeValues,
        std::vector<float>& minThresholds,
        std::vector<float>& maxThresholds);
};

#endif //PIPELINE_MAIN_H
