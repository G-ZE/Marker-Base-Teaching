// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service.h>
#include <ros/callback_queue.h>
#include <yaml-cpp/yaml.h>

// POSE ESTIMATION
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// POINT CLOUD PROCESSING
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/registration/icp.h>
//#include <pcl/recognition/hv/hv_go.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkPLYReader.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>

// MISC
#include <pthread.h>

// IMAGE PROCESSING
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

//CUSTOM MSGS
#include <marker_base_teaching/markers.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI IntensityType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::PointNormal NormalPointType;

// For ROS .yaml calibration
std::string yaml_path_;
std::string package_path_;

// For PCL visualisation
pcl::visualization::PCLVisualizer viewer("Kinect Viewer");

// For 2D camera parameters
cv::Mat camera_matrix;
cv::Mat dist_coeffs;
float focal_length;
//cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
float aruco_size = 0.08/2;
int target_id = 1;
std::string point_cloud_topic_name;
std::string image_topic_name;
std::string image_path_;
// For SD resolution
std::string camera_name = "kinect_sd";

// For HD resolution
//~ std::string camera_name = "kinect";

// For Image Processing
int blur_param_ = 5;
int hsv_target_ = 145;
int hsv_threshold_ = 10;
int contour_area_min_ = 500;
int contour_area_max_ = 4000;
double contour_ratio_min_ = 6;
double contour_ratio_max_ = 9;
double contour_length_ratio_min_ = 1;
double contour_length_ratio_max_ = 3;
marker_base_teaching::markers allmarkers;

// For Software control
int input_value_ = 0;
bool debug_;
bool got_image_ = false;
bool got_image_2 = false;
bool got_image_3 = false;
bool got_cloud_ = false;
int image_buffer_ = 0;
int image_buffer_2 = 0;
int image_buffer_3 = 0;

#include "data_manager.cpp"
DataManagement dm;


int main (int argc, char** argv){
    std::cout << std::endl << "Test Package" << std::endl;
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");
    
 
     
    // VARIABLE INITIALISATION
    package_path_ = ros::package::getPath("marker_base_teaching");
	image_path_ = "/pose_estimation_frames/test1/image.png";
	dm.setParameters(480,640,package_path_);
	point_cloud_topic_name = "/camera/depth/points";
	image_topic_name = "/camera/rgb/image_rect_color";
	
	
    // DEBUGGING
    std::cout << "Package Path: " << package_path_ << std::endl;
	// LOADING IMAGE AND POINT CLOUD
    int my_check_counter;
    float start =ros::Time::now().toSec();
	float now =ros::Time::now().toSec();
    bool data_publish = false;
    int counter = 0;
	while (true){
		
	    double begin =ros::Time::now().toSec();
	    double current = ros::Time::now().toSec();
	    
	    float duration = 0.2 + 0.2*counter;
	    double time_limit = 5.0;
	    double time_limit2 = 2.0;
		
		double starttimer =ros::Time::now().toSec();
		double endtimer =ros::Time::now().toSec();
	    double full_test_begin_ =ros::Time::now().toSec();
	    double full_test_end_ = ros::Time::now().toSec();
	    full_test_begin_ = ros::Time::now().toSec();
	    cv::Mat image;
	    image = cv::imread(package_path_ + image_path_, CV_LOAD_IMAGE_COLOR); 
	    dm.loadFrame(image);
	    //TESTING COMPUTATION
	    /*
	    float cost_p;
	    float v = 3.0;
	    std::vector<float> object_a(10);
	    std::vector<float> object_c(10);
	    object_c[0] = 10;
	    object_c[1] = 10;
	    object_c[2] = 10;
	    object_c[3] = 0;
	    object_c[4] = 0;
	    object_c[5] = 0;
	    object_c[6] = 0.5;
	    object_c[7] = 0;
	    object_c[8] = 0;
	    object_c[9] = 0;
	    
	    object_a[0] = 1.0;
	    object_a[1] = 1.0;
	    object_a[2] = 1.0;
	    object_a[3] = 9;
	    object_a[4] = 9;
	    object_a[5] = 9;
	    object_a[6] = 0;
	    object_a[7] = 0;
	    object_a[8] = 0;
	    object_a[9] = 0;
	    
		cost_p = dm.cost_function(object_a,object_c,v);
		std::cout <<"cost p "<< cost_p <<std::endl;
		*/
		
		std::vector <float> marker_position(11);
		std::vector < std::vector <float> > cloud_data;
		int n = 1 + counter;
		//int n = 2;
		//OBJECT 1
		for (int i=0; i<n; i++){
			marker_position[0] = 80 + 80*i;
			marker_position[1] = 60 + 60*i;
			marker_position[2] = 0;
			marker_position[3] = 0;
			marker_position[4] = 0;
			marker_position[5] = 0;
			marker_position[6] = 0;
			marker_position[7] = counter;
			marker_position[8] = 40 + 80*i;
			marker_position[9] = 40 + 60*i;
			marker_position[10] = 0;
			
		}
		cloud_data.push_back(marker_position);
		//OBJECT 2
		int m = 1 + counter;
		for (int j=0; j<m; j++){
			marker_position[0] = 560 - 80*j;
			marker_position[1] = 60 + 60*j;
			marker_position[2] = 0;
			marker_position[3] = 0;
			marker_position[4] = 0;
			marker_position[5] = 0;
			marker_position[6] = 0;
			marker_position[7] = counter;
			marker_position[8] = 560 - 80*j;
			marker_position[9] = 60 + 60*j;
			marker_position[10] = 0;
			
		}
		cloud_data.push_back(marker_position);
		//dm.markerTracking(duration,cloud_data);
		cloud_data.clear();
	    full_test_end_ = ros::Time::now().toSec();
	    std::cout <<"Time taken for marker detection : "<< full_test_end_ - begin <<std::endl;
	    std::cout << "Time taken for computation : " << full_test_end_  - full_test_begin_ << std::endl;
	
	    full_test_end_ = ros::Time::now().toSec();
		std::cout << "Time taken for full loop computation : " << full_test_end_  - full_test_begin_ << std::endl;
	    // VIEW THE CORRESPONDING IMAGE
	    std::cout << std::endl << "Starting Image Viewer..." << std::endl;
        cv::Mat image_display_;
        cv::Mat image_display_2;
        bool retrieve_image_ = false;
        retrieve_image_ = dm.getFrame(image_display_);
        
        bool retrieve_track_image_ = false;
        //retrieve_track_image_ = dm.getTrackingFrame(image_display_2);
        
        if(retrieve_image_ && !image_display_.empty()){
		    cv::imshow("Image Viewer", image_display_);
		    if(retrieve_track_image_){
				cv::imshow("Image tracking Viewer", image_display_);
			}
		    cv::waitKey(30);
		     
	    }
	    if(counter == 2){
			if(cv::waitKey(0)){
				break;
			}
		}
		counter++;
	    
	    //clear to restart
	    dm.clearFrameAndCloud();
	    dm.clearDescriptors();
	    dm.clearPixelPoints();
	    got_image_ = false;
	    got_image_2 = false;
	    got_image_3 = false;
        got_cloud_ = false;
        allmarkers.arraypoints.clear();
        allmarkers.arraypixelpoints.clear();
        allmarkers.arraypoints2.clear();
        
        
        std::cout<<"viewer clear "<<std::endl;
	   
    }

	ros::shutdown();
	
  return 0;
}

