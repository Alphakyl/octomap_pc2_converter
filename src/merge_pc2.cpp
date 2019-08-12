#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct LabelledPC{
	std::string robot_name;
	PointCloud::Ptr cloud;
	LabelledPC() : cloud (new PointCloud) {};
}

class PC{
	public:
	//Callbacks
	void callback1(const sensor_msgs::PointCloud2Ptr& cloud_in){
		pcl::fromROSMsg(*cloud_in, cloud);
	}	
	void callback2(const sensor_msgs::PointCloud2Ptr& cloud_in){
		pcl::fromROSMsg(*cloud_in, cloud);
	}
	void callback3(const sensor_msgs::PointCloud2Ptr& cloud_in){
		pcl::fromROSMsg(*cloud_in, cloud);
	}
	void merge_callback(const std_msgs::int8Ptr& to_merge){
		*merge_value = to_merge;
	}
	//Merge Function
	sensor_msgs::PointCloud2 mergePCL(LabelledPC robot_1, LabelledPC robot_2, LabelledPc robot_3, merge_value){
		switch(merge_value){
			case 0:
				ROS_WARNING("No point clouds selected to merge");
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				break;
			case 6:
				break;
			case 7: 
				break;
			default:
				ROS_ERROR("Incorrect map merge values supplied");
		}
	}
};

int* merge_value;
