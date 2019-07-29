#pragma once
#include <RobotRaconteur.h>
#include "robotraconteur_generated.h"
#include <stdio.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <boost/enable_shared_from_this.hpp>oh

using namespace std;

using namespace boost;
using namespace RobotRaconteur;
using namespace ::sensors::kinect2;

class PCL_RR: public boost::enable_shared_from_this<PCL_RR>
{
public:
	PCL_RR(string data_address);
	void StartStreaming();
	void StopStreaming();
	void DisplayPointCloud();
	void get_kinect_data();
	pcl::PointCloud<pcl::PointXYZ>::Ptr FilterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	vector<pcl::PointIndices> EuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	//void LoadFeatureModels();
	void FindFaces(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointXYZ FindCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	//transform point cloud function
	//moment of inertia functions
	sensors::kinect2::KinectPtr kinect_interface;

private:
	bool streaming;
	bool displaying;
	TimerPtr streaming_timer;
	

};

extern boost::recursive_mutex global_lock;