#include <string>
#include <iostream>

#include <nan.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include "../../src/Pipeline.h"
#include "../../src/PipelineMain.h"
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
        std::string filepath(*Nan::Utf8String(info[1]));
        std::string outputFilename(*Nan::Utf8String(info[2]));
        v8::Local<v8::Array> filtersv8 = v8::Local<v8::Array>::Cast(info[3]);
        bool saveResults = info[4]->BooleanValue();

        std::vector<std::string> filters;
        std::vector<std::string> kdtreeMethods;
        std::vector<float> kdtreeValues;
        std::vector<float> minThresholds;
        std::vector<float> maxThresholds;

        for (int i = 0; i < filtersv8->Length(); i++)
        {
            v8::Local<v8::Object> filter = filtersv8->Get(i)->ToObject();
            v8::Local<v8::Value> filterName = Nan::Get(filter, Nan::New("filterName").ToLocalChecked()).ToLocalChecked();

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

            filters.push_back(filterStr);
            kdtreeMethods.push_back(kdtreeMethodStr);
            kdtreeValues.push_back(kdtreeValueFloat);
            minThresholds.push_back(minThresholdFloat);
            maxThresholds.push_back(maxThresholdFloat);
        }

        PipelineMainResponse response = PipelineMain::run(
            filename,
            filepath,
            outputFilename,
            filters,
            kdtreeMethods,
            kdtreeValues,
            minThresholds,
            maxThresholds,
            saveResults);

        if (outputFilename != "") {
            Utils::saveCloud(outputFilename, response.lastFilteredCloud);
        }

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("intermediary_clouds").ToLocalChecked(), cloudsLogsEntriestoV8Array(response.cloudsLog.getLogs()));
        info.GetReturnValue().Set(moduleResponse);
    }
    catch(const std::exception& e)
    {
        v8::Isolate* isolate = v8::Isolate::GetCurrent();
        isolate->ThrowException(v8::Exception::TypeError(v8::String::NewFromUtf8(isolate, e.what())));
    }
}

NAN_METHOD(JoinClouds)
{
    try
    {
        v8::Local<v8::Array> files = v8::Local<v8::Array>::Cast(info[0]);
        std::string outputFilename(*Nan::Utf8String(info[1]));
        bool saveResults = info[2]->BooleanValue();

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

        CloudsLog cloudsLog;

        for (int i = 0; i < files->Length(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            Utils::loadCloudFile(std::string(*Nan::Utf8String(files->Get(i)->ToString())), cloud);
            
            *outputCloud += *cloud;
            cloudsLog.add(std::string(*Nan::Utf8String(files->Get(i)->ToString())), cloud);
        }

        if (saveResults)
        {
            Utils::saveCloud(outputFilename, outputCloud);
        }

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("cloud").ToLocalChecked(), parsePointCloudToV8Array(*outputCloud));
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
    Set(target, New<String>("joinClouds").ToLocalChecked(),
        GetFunction(New<FunctionTemplate>(JoinClouds)).ToLocalChecked());
}

NODE_MODULE(pipeline, init)
