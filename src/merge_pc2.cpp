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

int merge_value;

sensor_msgs::PointCloud2 merged_pub;	

ros::Publisher pub;

LabelledPC robot_1;
LabelledPC robot_2;
LabelledPC robot_3;

class PC{
	public:
		/* Callbacks */
		/* Calbacks assign a robots map to a global storage value, and ateemps to merge value */
		void callback1(const sensor_msgs::PointCloud2Ptr& cloud_in){
			robot_1.cloud = *cloud_in;
			mergePCL(robot_1, robot_2, robot_3, merge_value);
		}		
		void callback2(const sensor_msgs::PointCloud2Ptr& cloud_in){
			robot_2.cloud = *cloud_in;
			mergePCL(robot_1, robot_2, robot_3, merge_value);
		}
		void callback3(const sensor_msgs::PointCloud2Ptr& cloud_in){
			robot_3.cloud = *cloud_in;
			mergePCL(robot_1, robot_2, robot_3, merge_value);
		}
		/* Callback to assign which maps to merge dynamically w/ a ROS message */
		void merge_callback(const std_msgs::Int8 to_merge){
			merge_value = to_merge.data;
		}
		/* Function to merge all maps to a single value */
		void mergePCL(LabelledPC robot_1, LabelledPC robot_2, LabelledPC robot_3, int merge_value){
			PointCloud::Ptr merged_cloud (new PointCloud);
			PointCloud::Ptr filtered_merged_cloud (new PointCloud);	
			sensor_msgs::PointCloud2 merged_out;
			pcl::VoxelGrid<PointCloud> sor;
			
			
			/* Check to see which maps are merging and merge them */
			switch(merge_value){
				case 0: //000
					ROS_WARN("No point clouds selected to merge");
					break;
				case 1: //001
					pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
					break;
				case 2: //010
					pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
					break;
				case 3: //011
					pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
					pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
					break;
				case 4: //100
					pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
					break;
				case 5: //101
					pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
					pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
					break;
				case 6: //110
					pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
					pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
					break;
				case 7: //111
				default:
					if(merge_value != 7){
						ROS_WARN("Incorrect map merge values supplied:Defaulting to Merge All");
					}
					pcl::concatenatePointCloud(merged_out, robot_1.cloud, merged_out);
					pcl::concatenatePointCloud(merged_out, robot_2.cloud, merged_out);
					pcl::concatenatePointCloud(merged_out, robot_3.cloud, merged_out);
			}

			/* Filter merge cloud to avoid over publishing points */
			pcl_conversions::toPCL(merged_out, *merged_cloud);
			sor.setInputCloud(merged_cloud);
			sor.setLeafSize(0.1f, 0.1f, 0.1f);
			sor.filter(*filtered_merged_cloud);
			pcl_conversions::fromPCL(*filtered_merged_cloud, merged_pub);		
		}
};

int main(int argc, char **argv){
	/* Required for ROS initialization */
	ros::init(argc, argv, "merge_pc2");

	/* Create a node object */
	ros::NodeHandle n;

	PC global_PC;

	/*Create a subscriber for each topic*/
	/* Robot map subscriptions */
	ros::Subscriber sub1 = n.subscribe("H01/pc2_out", 100, &PC::callback1, &global_PC);
	ros::Subscriber sub2 = n.subscribe("H02/pc2_out", 100, &PC::callback2, &global_PC);
	ros::Subscriber sub3 = n.subscribe("H03/pc2_out", 100, &PC::callback3, &global_PC);

	/* Merge value subscription */
	ros::Subscriber sub = n.subscribe("map_choice",100, &PC::merge_callback, &global_PC);


	/* Advertise => Publish on a specific topic name */
	pub = n.advertise<sensor_msgs::PointCloud2>("merged_map",1);

	ros::Rate loop_rate(0.5);

	while(ros::ok()){
		merged_pub.header.stamp = ros::Time::now();
		merged_pub.header.frame_id = "world";
		pub.publish(merged_pub);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
