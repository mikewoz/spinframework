/*
 * Copyright (C) 2013 Emmanuel Durand
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#ifndef SHMPOINTCLOUD_H
#define SHMPOINTCLOUD_H

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "shmdata/any-data-reader.h"
#include "shmdata/any-data-writer.h"

#include "config.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::octree;

/*************/
// NetworkPointCloudCompression
/*************/
template<typename PointT, typename LeafT = OctreeContainerDataTVector<int>,
    typename BranchT = OctreeContainerEmpty<int>,
    typename OctreeT = Octree2BufBase<int, LeafT, BranchT> >
class NetworkPointCloudCompression : public OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>
{
    typedef typename OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;

    public:
        NetworkPointCloudCompression(compression_Profiles_e compressionProfile_arg = MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
                               bool showStatistics_arg = false,
                               const double pointResolution_arg = 0.001,
                               const double octreeResolution_arg = 0.01,
                               bool doVoxelGridDownDownSampling_arg = false,
                               const unsigned int iFrameRate_arg = 0,
                               bool doColorEncoding_arg = true,
                               const unsigned char colorBitResolution_arg = 6) :
            OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT> (compressionProfile_arg, showStatistics_arg, pointResolution_arg,
                octreeResolution_arg, doVoxelGridDownDownSampling_arg, iFrameRate_arg, doColorEncoding_arg, colorBitResolution_arg),
            validFrame_(PointCloudPtr()),
            prevValidFrame_(0)
        {
        }

        ~NetworkPointCloudCompression()
        {
        }

        void decodeNetworkPointCloud(std::istream &compressedTreeDataIn_arg, PointCloudPtr &cloud_arg);

    private:
        PointCloudPtr validFrame_;
        bool firstFrame_;

        unsigned int prevValidFrame_;
};

/*************/
template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
void NetworkPointCloudCompression<PointT, LeafT, BranchT, OctreeT>::decodeNetworkPointCloud(std::istream &compressedTreeDataIn_arg,
    PointCloudPtr &cloud_arg)
{
    typename pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>());

    try
    {
        this->decodePointCloud(compressedTreeDataIn_arg, tempCloud);
        if (this->frameID_ == prevValidFrame_+1 || this->iFrame_ == true)
        {
            validFrame_ = tempCloud;
            prevValidFrame_ = this->frameID_;
            firstFrame_ = false;
        }
    }
    catch (...)
    {
        std::cout << "Exception caught." << std::endl;
    }

    if (firstFrame_ == false)
    {
        cloud_arg = validFrame_;
    }
}

/*************/
// PointCloudBlob
/*************/
template <typename T>
class PointCloudBlob
{
    public:
        PointCloudBlob();
        ~PointCloudBlob() {free(_blob);}

        void* toBlob(typename PointCloud<T>::Ptr &cloud, int &size);
        typename PointCloud<T>::Ptr toCloud(void* blob, int size);

    private:
        float* _blob;
};

/*************/
template <typename T>
PointCloudBlob<T>::PointCloudBlob()
{
    _blob = NULL;
}

/*************/
template <typename T>
void* PointCloudBlob<T>::toBlob(typename PointCloud<T>::Ptr &cloud, int &size)
{
    if (_blob != NULL)
        free(_blob);

    // Get the size for the blob
    size_t cloudSize = cloud->size();
    int blobSize = cloudSize * (3 + 1); // each point is 3 floats for xyz and one float for the color
    _blob = (float*)malloc(blobSize*sizeof(float));

    for (unsigned int i = 0; i < cloudSize; ++i)
    {
        T* point = &(cloud->at(i));
        _blob[i*4 + 0] = point->x;
        _blob[i*4 + 1] = point->y;
        _blob[i*4 + 2] = point->z;
        _blob[i*4 + 3] = point->rgb;
    }

    size = blobSize*sizeof(float);
    return (void*)_blob;
}

/*************/
template <typename T>
typename PointCloud<T>::Ptr PointCloudBlob<T>::toCloud(void* blob, int size)
{
    typename PointCloud<T>::Ptr cloud(new PointCloud<T>());

    // Get the size of the cloud
    float* newBlob = (float*)blob;
    unsigned int cloudSize = size / (4*sizeof(float));

    cloud->reserve(cloudSize);
    for (unsigned int i = 0; i < cloudSize; ++i)
    {
        T point;
        point.x = newBlob[i*4 + 0];
        point.y = newBlob[i*4 + 1];
        point.z = newBlob[i*4 + 2];
        point.rgb = newBlob[i*4 + 3];

        cloud->push_back(point);
    }

    return cloud;
}

/*************/
// ShmPointCloud
/*************/
#define SHMCLOUD_TYPE_BASE          "application/x-pcd"
#define SHMCLOUD_TYPE_COMPRESSED    "application/x-pcd-compressed"

template <typename T>
class ShmPointCloud
{
    public:
        typedef typename pcl::PointCloud<T>::Ptr cloudPtr;

        ShmPointCloud(const char* filename, bool isWriter = false);
        ~ShmPointCloud();

        void setCompression(compression_Profiles_e compressionProfile = MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
                            bool showStatistics = false,
                            const double pointResolution = 0.001,
                            const double octreeResolution = 0.01,
                            bool doVoxelGridDownDownSampling = false,
                            const unsigned int iFrameRate = 0,
                            bool doColorEncoding = true,
                            const unsigned char colorBitResolution = 6);

        bool isUpdated() {return _updated;}
        unsigned long long getCloud(cloudPtr &cloud);
        void setCloud(cloudPtr &cloud, bool compress = false, unsigned long long timestamp = 0);

    private:
        bool _isWriter;

        string _filename;

        shmdata_any_writer_t* _writer;
        shmdata_any_reader_t* _reader;
        cloudPtr _cloud;

        bool _updated;
        unsigned long long _timestamp;

        char* _dataBuffer;
        unsigned int _dataBufferSize;

        std::mutex _mutex;

        std::shared_ptr<NetworkPointCloudCompression<T>> _networkPointCloudEncoder;
        std::shared_ptr<PointCloudBlob<T>> _blober;

        static void onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
            const char* type_description, void* user_data);
};

/*************/
template <typename T>
ShmPointCloud<T>::ShmPointCloud(const char* filename, bool isWriter) :
    _writer(NULL),
    _reader(NULL),
    _updated(false)
{
    _cloud.reset(new pcl::PointCloud<T>());

    _networkPointCloudEncoder.reset(new NetworkPointCloudCompression<T>());
    _blober.reset(new PointCloudBlob<T>());

    _filename = string(filename);

    _isWriter = isWriter;
    if (_isWriter)
    {
        // Shmdata writer
        _writer = shmdata_any_writer_init();
        shmdata_any_writer_set_data_type(_writer, SHMCLOUD_TYPE_BASE);
        if (!shmdata_any_writer_set_path(_writer, (char*)(filename)))
        {
            cout << "**** The file " << filename << " exists, therefore a shmdata cannot be operated with this path." << endl;
            shmdata_any_writer_close(_writer);
        }
        shmdata_any_writer_start(_writer);
    }
    else
    {
        // Shmdata reader
        _reader = shmdata_any_reader_init();
        shmdata_any_reader_set_on_data_handler(_reader, ShmPointCloud::onData, this);
        shmdata_any_reader_set_absolute_timestamp(_reader, SHMDATA_ENABLE_ABSOLUTE_TIMESTAMP);
        shmdata_any_reader_start(_reader, (char*)(filename));
    }

    // Buffer
    _dataBufferSize = 1e6; // 1MB is a good start for this buffer
    _dataBuffer = (char*)malloc(_dataBufferSize*sizeof(char));
}

/*************/
template <typename T>
ShmPointCloud<T>::~ShmPointCloud()
{
    if (_reader != NULL)
        shmdata_any_reader_close(_reader);

    if (_writer != NULL)
        shmdata_any_writer_close(_writer);

    free(_dataBuffer);
}

/*************/
template <typename T>
void ShmPointCloud<T>::setCompression(compression_Profiles_e compressionProfile, bool showStatistics,
                                      const double pointResolution, const double octreeResolution,
                                      bool doVoxelGridDownDownSampling, const unsigned int iFrameRate,
                                      bool doColorEncoding, const unsigned char colorBitResolution)
{
    _networkPointCloudEncoder.reset(new NetworkPointCloudCompression<T>(compressionProfile, showStatistics, pointResolution,
                                                                        octreeResolution, doVoxelGridDownDownSampling, iFrameRate,
                                                                        doColorEncoding, colorBitResolution));
}

/*************/
template <typename T>
unsigned long long ShmPointCloud<T>::getCloud(cloudPtr &cloud)
{
    std::lock_guard<std::mutex> lock(_mutex);
    cloud = _cloud;
    _updated = false;
    return _timestamp;
}

/*************/
template <typename T>
void ShmPointCloud<T>::setCloud(cloudPtr &cloud, bool compress, unsigned long long timestamp)
{
    if (cloud.get() == NULL || !_isWriter)
        return;

    static bool wasCompressed = false;
    if (compress != wasCompressed)
    {
        // We changed transmission mode, we need to restart the writer
        cout << "**** Restarting shmdata writer." << endl;

        shmdata_any_writer_close(_writer);
        _writer = shmdata_any_writer_init();

        if (compress)
            shmdata_any_writer_set_data_type(_writer, SHMCLOUD_TYPE_COMPRESSED);
        else
            shmdata_any_writer_set_data_type(_writer, SHMCLOUD_TYPE_BASE);

        if (!shmdata_any_writer_set_path(_writer, (char*)(_filename.c_str())))
        {
            cout << "**** The file " << _filename << " exists, therefore a shmdata cannot be operated with this path." << endl;
            shmdata_any_writer_close(_writer);
        }
        shmdata_any_writer_start(_writer);

        wasCompressed = compress;
    }

    // Get the current timestamp
    unsigned long long currentTime;
    if (timestamp == 0)
    {
        auto now = chrono::high_resolution_clock::now();
        currentTime = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
    }
    else
    {
        currentTime = timestamp;
    }

    // Compress (or not) and send through shmdata
    if (compress)
    {
        stringstream compressedData;
        _networkPointCloudEncoder->encodePointCloud(cloud, compressedData);
        compressedData.read(_dataBuffer, _dataBufferSize);

        unsigned int size = compressedData.gcount();
        if (size == _dataBufferSize)
        {
            // It is likely that the buffer is too short, we will resize it
            free(_dataBuffer);
            _dataBufferSize *= 2;
            _dataBuffer = (char*)malloc(_dataBufferSize*sizeof(char));
        }
        else
        {
            shmdata_any_writer_push_data(_writer, _dataBuffer, size, currentTime, NULL, NULL);
        }
    }
    else
    {
        int dataSize;
        void* blob =_blober->toBlob(cloud, dataSize);

        shmdata_any_writer_push_data(_writer, blob, dataSize, currentTime, NULL, NULL);
    }

    auto now = chrono::high_resolution_clock::now();
    currentTime = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
}

/*************/
template <typename T>
void ShmPointCloud<T>::onData(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp,
    const char* type_description, void* user_data)
{
    ShmPointCloud* context = static_cast<ShmPointCloud*>(user_data);

    std::lock_guard<std::mutex> lock(context->_mutex);

    cloudPtr cloudOut(new pcl::PointCloud<T>());

    string dataType(type_description);
    if (dataType == string(SHMCLOUD_TYPE_COMPRESSED))
    {
        std::stringstream compressedData;
        compressedData.write((const char*)data, data_size);
        context->_networkPointCloudEncoder->decodeNetworkPointCloud(compressedData, cloudOut);
        context->_cloud = cloudOut;
    }
    else if (dataType == string(SHMCLOUD_TYPE_BASE))
    {
        context->_cloud = context->_blober->toCloud(data, data_size);
    }
    else
    {
        return;
    }

    context->_timestamp = timestamp;
    context->_updated = true;

    shmdata_any_reader_free(shmbuf);
}

#endif // SHMPOINTCLOUD_H
