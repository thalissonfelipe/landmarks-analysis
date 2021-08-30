//
// Created by marcusv on 10/03/19.
//

#ifndef EYERIGHTCORNER_FINDER_EYERIGHTCORNERFINDER_H
#define EYERIGHTCORNER_FINDER_EYERIGHTCORNERFINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include "CloudsLog.h"

class EyeRightCornerFinder
{
    typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<pcl::Normal> CloudNormal;
    typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

public:
    pcl::PointXYZ static choosePoint(
        CloudXYZ::Ptr inputCloud,
        float gfSearchRadius,
        std::string features,
        std::string featuresThreshold,
        CloudsLog &cloudsLog);

    bool static itsAGoodPoint(pcl::PointXYZ point, float xValue, float yValue, float zValue, float maxDistance);

    bool static savePoint(pcl::PointXYZ point, std::string filename, std::string cloudName);

    int static findPoint(int argc, char **argv);
};

#endif //EYERIGHTCORNER_FINDER_EYERIGHTCORNERFINDER_H
