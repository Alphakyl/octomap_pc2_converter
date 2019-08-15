#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud_xyz;
typedef pcl::PCLPointCloud2 PointCloud;

struct LabelledPC{
	std::string robot_name;
	sensor_msgs::PointCloud2 cloud;
};

int* merge_value;

ros::Publisher pub;

LabelledPC robot_1;
LabelledPC robot_2;
LabelledPC robot_3;

class PC{
	public:
	//Callbacks
	static void callback1(const sensor_msgs::PointCloud2Ptr& cloud_in){
		robot_1.cloud = *cloud_in;
	}	
	static void callback2(const sensor_msgs::PointCloud2Ptr& cloud_in){
		robot_2.cloud = *cloud_in;
	}
	static void callback3(const sensor_msgs::PointCloud2Ptr& cloud_in){
		robot_3.cloud = *cloud_in;
	}
	static void merge_callback(const std_msgs::Int8 to_merge){
		*merge_value = to_merge.data;
	}
	//Merge Function
	void mergePCL(LabelledPC robot_1, LabelledPC robot_2, LabelledPC robot_3, int merge_value){
		PointCloud::Ptr merge_cloud (new PointCloud);
		PointCloud::Ptr filtered_merge_cloud (new PointCloud);	
		pcl::VoxelGrid<PointCloud> sor;
		sensor_msgs::PointCloud2 merged_out;	
		switch(merge_value){
			case 0:
				ROS_WARN("No point clouds selected to merge");
				break;
			case 1:
				pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
				break;
			case 2:
				pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
				break;
			case 3:
				pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
				pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
				break;
			case 4:
				pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
				break;
			case 5:
				pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
				pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
				break;
			case 6:
				pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
				pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
				break;
			case 7: 
			default:
				if(merge_value != 7){
					ROS_WARN("Incorrect map merge values supplied:Defaulting to Merge All");
				}
				pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
				pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
				pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
		}
		pcl::toROSMsg(merged_out, filtered_merge_cloud);
		merged_out.header.stamp = ros::Time::now();
		merged_out.header.frame_id = "world";
		pub.publish(merged_out);
	}
};

int main(int argc, char **argv){
	/* Required for ROS initialization */
	ros::init(argc, argv, "map_merge");

	/* Create a node object */
	ros::NodeHandle n;

	/*Create a subscriber for each topic*/
	/* Robot map subscriptions */
	ros::Subscriber sub1 = n.subscribe("", 100, PC::callback1);
	ros::Subscriber sub2 = n.subscribe("", 100, PC::callback2);
	ros::Subscriber sub3 = n.subscribe("", 100, PC::callback3);

	/* Merge value subscription */
	ros::Subscriber sub = n.subscribe("",100, PC::merge_callback);

	PC::mergePCL(robot_1, robot_2, robot_3, *merge_value);


	/* Advertise => Publish on a specific topic name */
	pub = n.advertise<sensor_msgs::PointCloud2>("merged_map",1);



	/* Run forever */
	ros::spin();

	return 0;
};
