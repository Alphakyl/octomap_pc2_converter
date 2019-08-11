#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pub;

void conversionCallback(const octomap_msgs::OctomapConstPtr& octomap_in){
	/* Create an output PointCloud2 Message */
	sensor_msgs::PointCloud2* pc2_out;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	/* Convert OctoMap Msg to an Octomap */
	octomap::AbstractOcTree* abstract_octomap_in_ds = binaryMsgToMap(*octomap_in);
	octomap::OcTree* octomap_in_ds = NULL;
	if(abstract_octomap_in_ds){
		octomap_in_ds = dynamic_cast<octomap::OcTree*>(abstract_octomap_in_ds);
	} else {
		ROS_ERROR("Error creating OcTree from received message");
	}
	if(octomap_in_ds){
		size_t i = 0;
		for(octomap::OcTree::leaf_iterator it = octomap_in_ds->begin_leafs(), end = octomap_in_ds->end_leafs(); it != end; ++it){
			if(octomap_in_ds->isNodeOccupied(*it)){
				cloud_in->points[i].x = it.getX();
				cloud_in->points[i].y = it.getY();
				cloud_in->points[i].z = it.getZ();
				i++;
			}
		}
		pcl::toROSMsg(*cloud_in,*pc2_out);
		
		pc2_out->header.stamp = ros::Time::now();
		pc2_out->header.frame_id = "world";
		pub.publish(*pc2_out);
	} else {
		ROS_ERROR("Error reading OcTree from abstract");
	}
}

int main(int argc, char **argv){
	/* Required for ROS initialization */
	ros::init(argc,argv,"octomap_pc2_converter");

	/* Create a node object */
	ros::NodeHandle n;

	/*Create a subscriber on a specific topic*/
	ros::Subscriber sub = n.subscribe("octomap_binary",100,conversionCallback);

	/* advertise => publish on a specific topic name */
	pub =  n.advertise<sensor_msgs::PointCloud2>("pc2_out",1);

	/* Run forever */
	ros::spin();

	return 0;
}	
	
