#include <string>
#include <iostream>

#include <nan.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include "../../src/Pipeline.h"
#include "../../src/Utils.h"
#include "../../src/CloudsLog.h"

v8::Local<v8::Array> parsePointCloudToV8Array(pcl::PointCloud<pcl::PointXYZ> pointCloud)
{
    int pointCloudSize = pointCloud.points.size();
    v8::Local<v8::Array> response = Nan::New<v8::Array>(pointCloudSize);

    for (int i = 0; i < pointCloudSize; i++)
    {
        pcl::PointXYZ point;
        v8::Local<v8::Object> obj = Nan::New<v8::Object>();

        point = pointCloud.points[i];

        obj->Set(Nan::New("x").ToLocalChecked(), Nan::New<v8::Number>(point.x));
        obj->Set(Nan::New("y").ToLocalChecked(), Nan::New<v8::Number>(point.y));
        obj->Set(Nan::New("z").ToLocalChecked(), Nan::New<v8::Number>(point.z));

        Nan::Set(response, i, obj);
    }

    return response;
}

v8::Local<v8::Array> cloudsLogsEntriestoV8Array(std::vector<CloudsLogEntry> logs)
{
    int logsSize = logs.size();
    v8::Local<v8::Array> response = Nan::New<v8::Array>(logsSize);

    for (int i = 0; i < logsSize; i++)
    {
        CloudsLogEntry log = logs[i];

        v8::Local<v8::Object> obj = Nan::New<v8::Object>();
        obj->Set(Nan::New("cloud_label").ToLocalChecked(), Nan::New(log.cloudLabel).ToLocalChecked());
        obj->Set(Nan::New("cloud").ToLocalChecked(), parsePointCloudToV8Array(log.cloud));

        Nan::Set(response, i, obj);
    }

    return response;
}

// https://medium.com/@muehler.v/tutorial-to-node-js-native-c-modules-part-2-arrays-json-and-callbacks-9b81f09874cd
NAN_METHOD(Pipeline)
{
    try
    {
        std::string filename(*Nan::Utf8String(info[0]));
        std::string outputFilename(*Nan::Utf8String(info[1]));
        v8::Local<v8::Array> filters = v8::Local<v8::Array>::Cast(info[2]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Utils::loadCloudFile(filename, cloud);

        std::cout << "Número de pontos iniciais: " << cloud->points.size() << std::endl;

        std::vector<float> shapeIndexes;
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

        CloudsLog cloudsLog;

        for (int i = 0; i < filters->Length(); i++)
        {
            v8::Local<v8::Object> filter = filters->Get(i)->ToObject();
            v8::Local<v8::Value> filterName = Nan::Get(filter, Nan::New("filterName").ToLocalChecked()).ToLocalChecked();

            // if (Nan::HasOwnProperty(filter, Nan::New("params").ToLocalChecked()).FromMaybe(false))
            // {
            //     std::cout << "Tem sim!" << std::endl;
            // }

            // v8::Local<v8::Object> params = filter->ToObject();
            // v8::Local<v8::Object> params = Nan::Get(filter, Nan::New("params").ToLocalChecked()).ToLocalChecked().ToObject();
            // v8::Local<v8::Value> kdTreeMethod = Nan::Get(params, Nan::New("kdtreeMethod").ToLocalChecked()).ToLocalChecked();
            // v8::Local<v8::Value> kdTreeValue = Nan::Get(params, Nan::New("kdtreeValue").ToLocalChecked()).ToLocalChecked();
            // v8::Local<v8::Value> minThreshold = Nan::Get(params, Nan::New("minThreshold").ToLocalChecked()).ToLocalChecked();
            // v8::Local<v8::Value> maxThreshold = Nan::Get(params, Nan::New("maxThreshold").ToLocalChecked()).ToLocalChecked();
            v8::Local<v8::Value> kdTreeMethod = Nan::Get(filter, Nan::New("kdtreeMethod").ToLocalChecked()).ToLocalChecked();
            v8::Local<v8::Value> kdTreeValue = Nan::Get(filter, Nan::New("kdtreeValue").ToLocalChecked()).ToLocalChecked();
            v8::Local<v8::Value> minThreshold = Nan::Get(filter, Nan::New("minThreshold").ToLocalChecked()).ToLocalChecked();
            v8::Local<v8::Value> maxThreshold = Nan::Get(filter, Nan::New("maxThreshold").ToLocalChecked()).ToLocalChecked();

            std::string filterStr = std::string(*Nan::Utf8String(filterName->ToString()));
            std::string kdtreeMethodStr = std::string(*Nan::Utf8String(kdTreeMethod->ToString()));
            float kdtreeValueFloat = kdTreeValue->NumberValue();
            float minThresholdFloat = minThreshold->NumberValue();
            float maxThresholdFloat = maxThreshold->NumberValue();

            // std::cout << filterStr << std::endl;
            // std::cout << kdtreeMethodStr << std::endl;
            // std::cout << kdtreeValueFloat << std::endl;
            // std::cout << minThresholdFloat << std::endl;
            // std::cout << maxThresholdFloat << std::endl;

            if (filterStr == "shapeIndex")
            {
                Pipeline::filterByShapeIndex(
                    cloud,
                    kdtreeMethodStr,
                    kdtreeValueFloat,
                    minThresholdFloat,
                    maxThresholdFloat,
                    filteredCloud,
                    shapeIndexes);

                // not using for now
                shapeIndexes.clear();
            }
            else if (filterStr == "gaussianCurvature")
            {
                Pipeline::filterByGaussianCurvature(
                    cloud,
                    kdtreeMethodStr,
                    kdtreeValueFloat,
                    minThresholdFloat,
                    maxThresholdFloat,
                    filteredCloud,
                    outputPrincipalCurvaturesCloud);

                outputPrincipalCurvaturesCloud->points.clear();
            }
            else {
                Pipeline::filterByGeometricFeatures(
                    cloud,
                    filterStr,
                    kdtreeMethodStr,
                    kdtreeValueFloat,
                    minThresholdFloat,
                    maxThresholdFloat,
                    filteredCloud);
            }

            if (filteredCloud->points.size() == 0)
            {
                throw std::runtime_error("Número de pontos após filtragens é 0! Por favor, escolha outros parâmetros.");
            }

            std::cout << "Número de pontos após filtragem: " << filteredCloud->points.size() << " (" << filterStr << ")" << std::endl;
            cloudsLog.add(filterStr, filteredCloud);
            *cloud = *filteredCloud;
            filteredCloud->points.clear();
        }

        if (outputFilename != "") {
            Utils::saveCloud(outputFilename, cloud);
        }

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("intermediary_clouds").ToLocalChecked(), cloudsLogsEntriestoV8Array(cloudsLog.getLogs()));
        info.GetReturnValue().Set(moduleResponse);
    }
    catch(const std::exception& e)
    {
        v8::Isolate* isolate = v8::Isolate::GetCurrent();
        isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, e.what())));
    }
}

NAN_METHOD(GaussianCurvature)
{
    try
    {
        std::string filename(*Nan::Utf8String(info[0]));
        std::string kdtreeMethod(*Nan::Utf8String(info[1]));
        float kdtreeValue = info[2]->NumberValue();
        float thresholdMin = info[3]->NumberValue();
        float thresholdMax = info[4]->NumberValue();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Utils::loadCloudFile(filename, cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);
        indices.clear();

        pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr filteredNormalCloud(new pcl::PointCloud<pcl::Normal>);

        Pipeline::normalComputation(filteredCloud, kdtreeMethod, kdtreeValue, normalCloud);
        pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);

        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);
        Pipeline::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, kdtreeMethod, kdtreeValue, principalCurvaturesCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr outputPrincipalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);

        Pipeline::thresholdByGaussianCurvature(
            filteredCloud,
            principalCurvaturesCloud,
            thresholdMin,
            thresholdMax,
            outputCloud,
            outputPrincipalCurvaturesCloud);

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("output_label").ToLocalChecked(), Nan::New("GC").ToLocalChecked());
        moduleResponse->Set(Nan::New("output_cloud").ToLocalChecked(), parsePointCloudToV8Array(*outputCloud));
        info.GetReturnValue().Set(moduleResponse);
    }
    catch(const std::exception& e)
    {
        v8::Isolate* isolate = v8::Isolate::GetCurrent();
        isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, e.what())));
    }
}

NAN_METHOD(ShapeIndex)
{
    try
    {
        std::string filename(*Nan::Utf8String(info[0]));
        std::string kdtreeMethod(*Nan::Utf8String(info[1]));
        float kdtreeValue = info[2]->NumberValue();
        float thresholdMin = info[3]->NumberValue();
        float thresholdMax = info[4]->NumberValue();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Utils::loadCloudFile(filename, cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);
        indices.clear();

        pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr filteredNormalCloud(new pcl::PointCloud<pcl::Normal>);

        Pipeline::normalComputation(filteredCloud, kdtreeMethod, kdtreeValue, normalCloud);
        pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);
        indices.clear();

        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvaturesCloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);
        Pipeline::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, kdtreeMethod, kdtreeValue, principalCurvaturesCloud);

        std::vector<float> shapeIndexes;
        Pipeline::shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<float> outputShapeIndexes;

        Pipeline::thresholdByShapeIndex(
            filteredCloud,
            shapeIndexes,
            thresholdMin,
            thresholdMax,
            outputCloud,
            outputShapeIndexes);

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("output_label").ToLocalChecked(), Nan::New("SI").ToLocalChecked());
        moduleResponse->Set(Nan::New("output_cloud").ToLocalChecked(), parsePointCloudToV8Array(*outputCloud));
        info.GetReturnValue().Set(moduleResponse);
    }
    catch(const std::exception& e)
    {
        v8::Isolate* isolate = v8::Isolate::GetCurrent();
        isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, e.what())));;
    }

}

NAN_METHOD(GeometricFeature)
{
    try
    {
        std::string filename(*Nan::Utf8String(info[0]));
        std::string feature(*Nan::Utf8String(info[1]));
        std::string kdtreeMethod(*Nan::Utf8String(info[2]));
        float kdtreeValue = info[3]->NumberValue();
        float thresholdMin = info[4]->NumberValue();
        float thresholdMax = info[5]->NumberValue();
        std::string outputFilename(*Nan::Utf8String(info[6]));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Utils::loadCloudFile(filename, cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

        Pipeline::thresholdByGeometricFeature(
            cloud,
            feature,
            kdtreeMethod,
            kdtreeValue,
            thresholdMin,
            thresholdMax,
            outputCloud);

        Utils::saveCloud(outputFilename, outputCloud);

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("output_label").ToLocalChecked(), Nan::New(outputFilename).ToLocalChecked());
        moduleResponse->Set(Nan::New("output_cloud").ToLocalChecked(), parsePointCloudToV8Array(*outputCloud));
        info.GetReturnValue().Set(moduleResponse);
    }
    catch(const std::exception& e)
    {
        v8::Isolate* isolate = v8::Isolate::GetCurrent();
        isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, e.what())));;
    }

}

using Nan::GetFunction;
using Nan::New;
using Nan::Set;
using v8::Boolean;
using v8::FunctionTemplate;
using v8::Handle;
using v8::Number;
using v8::Object;
using v8::String;

NAN_MODULE_INIT(init)
{
    Set(target, New<String>("gaussianCurvature").ToLocalChecked(),
        GetFunction(New<FunctionTemplate>(GaussianCurvature)).ToLocalChecked());
    Set(target, New<String>("shapeIndex").ToLocalChecked(),
        GetFunction(New<FunctionTemplate>(ShapeIndex)).ToLocalChecked());
    Set(target, New<String>("geometricFeature").ToLocalChecked(),
        GetFunction(New<FunctionTemplate>(GeometricFeature)).ToLocalChecked());
    Set(target, New<String>("applyFilters").ToLocalChecked(),
        GetFunction(New<FunctionTemplate>(Pipeline)).ToLocalChecked());
}

NODE_MODULE(pipeline, init)
