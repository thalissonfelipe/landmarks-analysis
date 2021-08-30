//
// Created by marcusv on 10/03/19.
//
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "CloudsLog.h"

#ifndef NOSETIP_FINDER_NOSETIPFINDER_H
#define NOSETIP_FINDER_NOSETIPFINDER_H

class NosetipFinder
{
    typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<pcl::Normal> CloudNormal;
    typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

public:
    pcl::PointXYZ static chooseANoseTip(
        CloudXYZ::Ptr inputCloud,
        int searchRadius,
        CloudsLog &cloudsLog);

    bool static itsAGoodNoseTip(pcl::PointXYZ noseTip, float xValue, float yValue, float zValue, float maxDistance);

    bool static saveNoseTip(pcl::PointXYZ noseTip, std::string filename, std::string cloudName);

    int static findNosetip(int argc, char **argv);
};

#endif //NOSETIP_FINDER_NOSETIPFINDER_H
