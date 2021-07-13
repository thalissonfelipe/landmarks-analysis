#include <string>

#include <nan.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

#include "../../src/Pipeline.h"
#include "../../src/Utils.h"

v8::Local<v8::Array> parsePointCloudToV8Array(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
    int pointCloudSize = pointCloud->points.size();
    v8::Local<v8::Array> response = Nan::New<v8::Array>(pointCloudSize);

    for (int i = 0; i < pointCloudSize; i++)
    {
        pcl::PointXYZ point;
        v8::Local<v8::Object> obj = Nan::New<v8::Object>();

        point = pointCloud->points[i];

        obj->Set(Nan::New("x").ToLocalChecked(), Nan::New<v8::Number>(point.x));
        obj->Set(Nan::New("y").ToLocalChecked(), Nan::New<v8::Number>(point.y));
        obj->Set(Nan::New("z").ToLocalChecked(), Nan::New<v8::Number>(point.z));

        Nan::Set(response, i, obj);
    }

    return response;
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
        moduleResponse->Set(Nan::New("output_cloud").ToLocalChecked(), parsePointCloudToV8Array(outputCloud));
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
        moduleResponse->Set(Nan::New("output_cloud").ToLocalChecked(), parsePointCloudToV8Array(outputCloud));
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

        v8::Local<v8::Object> moduleResponse = Nan::New<v8::Object>();
        moduleResponse->Set(Nan::New("output_label").ToLocalChecked(), Nan::New("GF - " + feature).ToLocalChecked());
        moduleResponse->Set(Nan::New("output_cloud").ToLocalChecked(), parsePointCloudToV8Array(outputCloud));
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
}

NODE_MODULE(pipeline, init)
