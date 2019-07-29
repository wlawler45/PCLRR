
#include "PCLRR.h"


PCL_RR::PCL_RR(string address)
{
	ClientNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES);
	sensors::kinect2::KinectPtr kinect_interface= rr_cast<sensors::kinect2::Kinect>(RobotRaconteurNode::s()->ConnectService(string("tcp://localhost:8888/sensors.kinect2/Kinect2")));
	
	 
	
	//streaming_timer.reset();
}

void PCL_RR::StartStreaming()
{
	boost::recursive_mutex::scoped_lock lock(global_lock);
	if (streaming_timer) throw InvalidOperationException("Already streaming");
	RR_WEAK_PTR<PCL_RR> weak_this = shared_from_this();
	streaming_timer = RobotRaconteurNode::s()->CreateTimer(boost::posix_time::milliseconds(100),
		[weak_this](TimerEvent ev)
	{
		auto shared_this = weak_this.lock();
		if (!shared_this) return;
		shared_this->get_kinect_data();
	}
	);
	streaming_timer->Start();
}

void PCL_RR::get_kinect_data()
{
	sensors::kinect2::PointCloudPtr frameptr = kinect_interface->getPointCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//BOOST_FOREACH(frameptr& value, v) {
	
	/*for (unsigned i = 0; i < frameptr->points->size(); i++ ) {
		cloud->points[i].x = (*frameptr->points)[i].s.x;
		cloud->points[i].y = (*frameptr->points)[i].s.y;
		cloud->points[i].z = (*frameptr->points)[i].s.z;
		
	}*/

}

int main(int argc, char* argv[])
{
	
	PCL_RR inter =PCL_RR("");
	//RR_INTRUSIVE_PTR<sensors::kinect2::Image> pic= inter.kinect_interface->getCurrentColorImage();
	
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	inter.get_kinect_data();
	//viewer.showCloud(inter.cloud);
	//while (!viewer.wasStopped())
	//{
	//}

}
boost::recursive_mutex global_lock;