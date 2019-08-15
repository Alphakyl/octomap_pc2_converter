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
	PointCloud::Ptr cloud;
	LabelledPC() : cloud (new PointCloud) {};
};

int* merge_value;

ros::Publisher pub;

LabelledPC* robot_1 = new LabelledPC;
LabelledPC* robot_2 = new LabelledPC;
LabelledPC* robot_3 = new LabelledPC;

class PC{
	public:
	//Callbacks
	static void callback1(const sensor_msgs::PointCloud2Ptr& cloud_in){
		pcl_conversions::toPCL(*cloud_in, *robot_1->cloud);
	}	
	static void callback2(const sensor_msgs::PointCloud2Ptr& cloud_in){
		pcl_conversions::toPCL(*cloud_in, *robot_2->cloud);
	}
	static void callback3(const sensor_msgs::PointCloud2Ptr& cloud_in){
		pcl_conversions::toPCL(*cloud_in, *robot_3->cloud);
	}
	static void merge_callback(const std_msgs::Int8 to_merge){
		*merge_value = to_merge.data;
	}
	//Merge Function
	void mergePCL(LabelledPC robot_1, LabelledPC robot_2, LabelledPC robot_3, int merge_value){
		PointCloud::Ptr merge_cloud (new PointCloud);
		PointCloud filtered_merge_cloud (new PointCloud);	
		pcl::VoxelGrid<PointCloud> sor;
		sensor_msgs::PointCloud2 merged_out;	
		switch(merge_value){
			case 0:
				ROS_WARNING("No point clouds selected to merge");
				break;
			case 1:
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");
				break;
			case 2:
                                ROS_WARNING("No point clouds selected to merge");
                		sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");		
				break;
			case 3:
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");
				break;
			case 4:
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");
				break;
			case 5:
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");
				break;
			case 6:
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");		
				break;
			case 7: 
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");
				break;
			default:
				ROS_WARNING("Incorrect map merge values supplied:Defaulting to Merge All");
				sor.setInputCloud(merge_cloud);
				sor.setLeafSize(0.2,0.2,0.2);
				sor.filter(filtered_merge_cloud);
				ROS_INFO("Filtering Complete");
		}
		pcl::toROSMsg(filtered_merge_cloud, merged_out);
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

	PC::mergePCL(*robot_1, *robot_2, *robot_3, *merge_value);


	/* Advertise => Publish on a specific topic name */
	pub = n.advertise<sensor_msgs::PointCloud2>("merged_map",1);



	/* Run forever */
	ros::spin();

	return 0;
};
