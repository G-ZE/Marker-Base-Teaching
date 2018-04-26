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

// CONFIGURATION AND SET-UP FUNCTIONS
static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners, std::string patternType){
    corners.clear();

    if (patternType == "CHESSBOARD" || patternType == "CIRCLES_GRID"){
			for( int i = 0; i < boardSize.height; ++i )
					for( int j = 0; j < boardSize.width; ++j )
							corners.push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
		}

    else if (patternType == "ASYMMETRIC_CIRCLES_GRID"){
			for( int i = 0; i < boardSize.height; i++ )
					for( int j = 0; j < boardSize.width; j++ )
							corners.push_back(cv::Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
		}

}

void updateParameters(YAML::Node config){
	std::cout << "Updating Camera Parameters" << std::endl;
	if (config["fx"])
    camera_matrix.at<double>(0,0) = config["fx"].as<double>();
	if (config["fx"])
    camera_matrix.at<double>(1,1) = config["fx"].as<double>();
	if (config["x0"])
    camera_matrix.at<double>(0,2) = config["x0"].as<double>();
	if (config["y0"])
    camera_matrix.at<double>(1,2) = config["y0"].as<double>();
  if (config["k1"])
    dist_coeffs.at<double>(0,0) = config["k1"].as<double>();
  if (config["k2"])
    dist_coeffs.at<double>(1,0) = config["k2"].as<double>();
  if (config["k3"])
    dist_coeffs.at<double>(4,0) = config["k3"].as<double>();
  if (config["p1"])
    dist_coeffs.at<double>(2,0) = config["p1"].as<double>();
  if (config["p2"])
    dist_coeffs.at<double>(3,0) = config["p2"].as<double>();
  if (config["blur_param_"])
		blur_param_ = config["blur_param_"].as<int>();
  if (config["hsv_target_"])
		hsv_target_ = config["hsv_target_"].as<int>();
  if (config["hsv_threshold_"])
		hsv_threshold_ = config["hsv_threshold_"].as<int>();
  if (config["contour_area_min_"])
		contour_area_min_ = config["contour_area_min_"].as<int>();
  if (config["contour_area_max_ "])
		contour_area_max_  = config["contour_area_max_ "].as<int>();
	if (config["image_topic"])
		image_topic_name = config["image_topic"].as<std::string>();
	if (config["cloud_topic"])
		point_cloud_topic_name = config["cloud_topic"].as<std::string>();
}

// Retrieve data from .yaml configuration file and load them into the
// global calibration and distortion matrix. 
void loadCalibrationMatrix(std::string camera_name_){
	// Updates Parameter with .yaml file
	camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
	camera_matrix.at<double>(2,2) = 1;
  yaml_path_ = ros::package::getPath("marker_base_teaching") + "/config/camera_info.yaml";
  YAML::Node config;
  try 
  {
    config = YAML::LoadFile(yaml_path_);
  } 
  catch (YAML::Exception &e) 
  {
    ROS_ERROR_STREAM("YAML Exception: " << e.what());
    exit(EXIT_FAILURE);
  }
  if (!config[camera_name_])
  {
    ROS_ERROR("Cannot find default parameters in yaml file: %s", yaml_path_.c_str());
    exit(EXIT_FAILURE);
  }
  updateParameters(config[camera_name_]);
  
}

// Retrieve msg from /kinect2/sd/points and converts to PointType variable
// before loading it into the Data Manager.
void cloud_callback(const sensor_msgs::PointCloud2& cloud_msg){
	got_cloud_ = true;
  // Convert from msg to pcl 
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(cloud_msg,pcl_pc);
	pcl::PointCloud<PointType>::Ptr scene_cloud(new pcl::PointCloud<PointType>);
	pcl::fromPCLPointCloud2(pcl_pc,*scene_cloud);
	std::cout << "Cloud size: " << scene_cloud->points.size() << std::endl;
	dm.loadCloud(scene_cloud);
}

// Retrieves msg from /kinect2/sd/image_color_rect, converts to image, detects markers
// and undistorts the image before loading the image, rvec and tvec into the Data Manager.
void image_callback(const sensor_msgs::ImageConstPtr& msg){
	//~ std::cout << "Image callback, buffer: " << image_buffer_ << std::endl;
	if(image_buffer_>=0){
		
		try{
			cv::Mat image;
			cv::Mat unDistort;
			cv::Mat rvec;
			cv::Mat tvec;
			bool marker_found;
			cv_bridge::CvImage img_bridge;
			std_msgs::Header header;
			header.seq = 3;
			header.stamp = ros::Time::now();
			
			
			image = cv_bridge::toCvShare(msg, "rgb8")->image;
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
			//img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
            cv_ptr->toImageMsg(allmarkers.image);
            //img_bridge.toImageMsg(allmarkers.image);
            cv::cvtColor(image, image, CV_RGB2BGR);
            if (!image.empty()){
				got_image_ = true;
				dm.loadFrame(image);
			}
			
			//~ marker_found = false;
			// FOR ARUCO
			//marker_found = arucoPoseEstimation(image, target_id, tvec, rvec, camera_matrix, dist_coeffs, true);
			
			// FOR CIRCULAR MARKERS
			//~ marker_found = circleEstimation(image);
			
			//~ cv::undistort(image, unDistort, camera_matrix, dist_coeffs);
			//~ std::cout << "Image Size: " << unDistort.size() << std::endl;
			//~ dm.loadFrame(unDistort);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}
	else{
		image_buffer_++;
	}
}

void image_callback2(const sensor_msgs::ImageConstPtr& msg){
	//~ std::cout << "Image callback, buffer: " << image_buffer_ << std::endl;
	if(image_buffer_2>=0){
		got_image_2 = true;
		try{
			cv::Mat image2;
			cv::Mat unDistort;
			cv::Mat rvec;
			cv::Mat tvec;
			bool marker_found;
			image2 = cv_bridge::toCvShare(msg, "rgb8")->image;
            cv::cvtColor(image2, image2, CV_RGB2BGR);
            std::cout<<" storaging second image"<<std::endl;
            //cv::imshow("image", image2);
			dm.loadRefframe(image2);
			
			//~ marker_found = false;
			// FOR ARUCO
			//marker_found = arucoPoseEstimation(image, target_id, tvec, rvec, camera_matrix, dist_coeffs, true);
			
			// FOR CIRCULAR MARKERS
			//~ marker_found = circleEstimation(image);
			
			//~ cv::undistort(image, unDistort, camera_matrix, dist_coeffs);
			//~ std::cout << "Image Size: " << unDistort.size() << std::endl;
			//~ dm.loadFrame(unDistort);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}
	else{
		image_buffer_2++;
	}
}

void image_callback3(const sensor_msgs::ImageConstPtr& msg){
	
	if(image_buffer_3>=0){
		
		got_image_3 = true;
		try{
			
			cv::Mat image3;
			cv::Mat unDistort;
			cv::Mat rvec;
			cv::Mat tvec;
			bool marker_found;
			if (msg != 0){
				image3 = cv_bridge::toCvShare(msg, "bgr8")->image;
				std::cout<<" showing image"<<std::endl;
				cv::imshow("marker_image", image3);
				cv::waitKey(30);
			}
			else{
				std::cout<<" no image from kinect_tracking "<<std::endl;
		
			}
			     
			
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}
	else{
		
		image_buffer_3++;
	}
	
}
//not used
void *call_from_thread(void *){
	while (true){
		double tbegin =ros::Time::now().toSec();
	    double tcurrent = ros::Time::now().toSec();
	    double time_limit2 = 2.0;
	    ros::NodeHandle nh_t;
		image_buffer_3 = 0;
		std::cout << std::endl << "Subscribing to kinect tracking image..." << std::endl;
		image_transport::ImageTransport it3(nh_t);
		image_transport::Subscriber image_sub_3 = it3.subscribe("marker_number", 1, image_callback3);
		while (tcurrent-tbegin < time_limit2 && !got_image_3) {
			tcurrent =ros::Time::now().toSec();
			ros::spinOnce();
		}
		image_sub_3.shutdown();
	
	}
	return NULL;
}


// PCL VIEWER KEYBOARD CALL BACK
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  std::string key_value = event.getKeySym();
  std::cout << key_value << std::endl;
  
  if (event.getKeySym () == "d" && event.keyDown ()){
    input_value_ = 1;
  }
}

int main (int argc, char** argv){
    std::cout << std::endl << "Kinect Depth Test Package" << std::endl;
    ros::init(argc, argv, "kinect_pose_estimation");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");
    
    //new
    ros::Publisher marker_points = nh_.advertise<marker_base_teaching::markers> ("kinect_openrave_marker_points", 1);
    //marker_base_teaching::markers allmarkers;
    //ros::Rate loop_rate(10);
     
    // VARIABLE INITIALISATION
    package_path_ = ros::package::getPath("marker_base_teaching");
	nh_private_.getParam("blur_param_", blur_param_);
    nh_private_.getParam("hsv_target_", hsv_target_);
    nh_private_.getParam("hsv_threshold_", hsv_threshold_);
    nh_private_.getParam("contour_area_min_", contour_area_min_);
    nh_private_.getParam("contour_area_max_", contour_area_max_);
    nh_private_.getParam("contour_ratio_min_", contour_ratio_min_);
    nh_private_.getParam("contour_ratio_max_", contour_ratio_max_);
    nh_private_.getParam("contour_length_ratio_min_", contour_length_ratio_min_);
    nh_private_.getParam("contour_length_ratio_max_", contour_length_ratio_max_);
  
	
    // CAMERA CALIBRATION
    /*
	camera_matrix = cv::Mat::eye(3, 3, CV_64F);
	dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    loadCalibrationMatrix(camera_name);
	focal_length = camera_matrix.at<double>(0,0);
	*/
	
	//dm.setParameters(2*camera_matrix.at<double>(1,2), 2*camera_matrix.at<double>(0,2),package_path_);
	dm.setParameters(480,640,package_path_);
	point_cloud_topic_name = "/camera/depth/points";
	image_topic_name = "/camera/rgb/image_rect_color";
	
	
    // DEBUGGING
    std::cout << "Package Path: " << package_path_ << std::endl;
    /*
	std::cout << std::endl << "Calibration Matrix: " << std::endl << std::setprecision(5);
	for (int i=0 ; i<3 ; i++){
		std::cout << "[ " ;
		for (int j=0 ; j<3 ; j++)
			std::cout << camera_matrix.at<double>(i,j) << " ";
		std::cout << "]" << std::endl;
	}
	std::cout << std::endl << "Focal Length: " << focal_length << std::endl;
	
	std:: cout << std::endl << "Distortion Matrix: " << std::endl << "[ ";
	for (int i=0 ; i<5 ; i++){
		std::cout << dist_coeffs.at<double>(i,0) << " ";
	}
	std::cout << "]" << std::endl;
	*/
	
	//TO retrieve final image
	//pthread_t t;
	//pthread_create(&t, NULL, call_from_thread, NULL);
	
	//START SUBSCRIBING FOR IMAGE AND POINTCLOUD
	std::cout << std::endl << "Subscribing to Kinect Image Topic..." << std::endl;
	image_transport::ImageTransport it(nh_);
	image_buffer_ = 0;
	image_transport::Subscriber image_sub_ = it.subscribe(image_topic_name, 1, image_callback);
	
	image_buffer_3 = 0;
	std::cout << std::endl << "Subscribing to kinect tracking image..." << std::endl;
	image_transport::ImageTransport it3(nh_);
	image_transport::Subscriber image_sub_3 = it3.subscribe("marker_number", 1, image_callback3);
	
	std::cout << std::endl << "Subscribing to Kinect Point Cloud Topic..." << std::endl;
	ros::Subscriber point_cloud_sub_;
	point_cloud_sub_ = nh_.subscribe(point_cloud_topic_name, 1, cloud_callback);
	
	//Retrieve image and pointcloud
	
	// LOADING IMAGE AND POINT CLOUD
    int my_check_counter;
    double start =ros::Time::now().toSec();
	double now =ros::Time::now().toSec();
    bool data_publish = false;
	while (true){
		
	    double begin =ros::Time::now().toSec();
	    double current = ros::Time::now().toSec();
	    
	    double time_now = 0;
	    double time_limit = 5.0;
	    double time_limit2 = 2.0;
		
		double starttimer =ros::Time::now().toSec();
		double endtimer =ros::Time::now().toSec();
	    double full_test_begin_ =ros::Time::now().toSec();
	    double full_test_end_ = ros::Time::now().toSec();
	    full_test_begin_ = ros::Time::now().toSec();
	    
	    while ((!got_image_)&&(!got_cloud_)) {
		    ros::spinOnce();
	    }
	    /*
	    starttimer =ros::Time::now().toSec();
	    begin =ros::Time::now().toSec();
	    current = ros::Time::now().toSec();
	    image_buffer_2 = 0;
	    std::cout << std::endl << "Subscribing to Kinect Image2 Topic..." << std::endl;
	    image_transport::ImageTransport it2(nh_);
	    image_transport::Subscriber image_sub_2 = it2.subscribe(image_topic_name, 1, image_callback2);
	    while (current-begin < time_limit && !got_image_2) {
		    current =ros::Time::now().toSec();
		    ros::spinOnce();
	    }
	    image_sub_2.shutdown();
	    endtimer =ros::Time::now().toSec();
	    std::cout <<"Time taken for image2 retrieve : "<< endtimer - starttimer <<std::endl;
	    */
	
		   
	    
	    /*
	    begin =ros::Time::now().toSec();
	    current = ros::Time::now().toSec();
	    if (data_publish == true){
			image_buffer_3 = 0;
			std::cout << std::endl << "Subscribing to kinect tracking image..." << std::endl;
			image_transport::ImageTransport it3(nh_);
			image_transport::Subscriber image_sub_3 = it3.subscribe("marker_number", 1, image_callback3);
			while (current-begin < time_limit2 && !got_image_3) {
				current =ros::Time::now().toSec();
				ros::spinOnce();
			}
			image_sub_3.shutdown();
		}
		else{
			std::cout<<" data not published "<<std::endl;
		}
	    */
	
		begin = ros::Time::now().toSec();
	    //DETECT MARKERS and COMPUTE DESCRIPTORS
	    std::cout << "Using Circular Marker Detection" << std::endl;
	    dm.setMinMarkers(1);
	    bool tracking_confirm = false;
	    if (dm.detectMarkers(blur_param_, hsv_target_, hsv_threshold_ , contour_area_min_, contour_area_max_, contour_ratio_min_, contour_ratio_max_, contour_length_ratio_min_, contour_length_ratio_max_, false)){
		    //dm.computeDescriptors();
		    full_test_end_ = ros::Time::now().toSec();
			std::cout <<"Time taken for marker detection : "<< full_test_end_ - begin <<std::endl;
		    now = ros::Time::now().toSec();
		    time_now = now;
		    std::cout << "time now to for tracking " <<time_now<< std::endl;
		    tracking_confirm  = dm.markerTracking(time_now);
		    while (tracking_confirm != true){
				
			}
			//start = ros::Time::now().toSec();
			
		    std::cout<< "markers detected "<<std::endl;
	    }
	    else{
		    std::cout << "No markers detected" << std::endl;
	    }
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
	    full_test_end_ = ros::Time::now().toSec();
	    std::cout <<"Time taken for marker detection : "<< full_test_end_ - begin <<std::endl;
	    std::cout << "Time taken for computation : " << full_test_end_  - full_test_begin_ << std::endl;
	
	    // VIEW DETECTED POINTS ON THE CLOUD VIEWER
	    // VIEWER PARAMETERS
	    std::cout << std::endl << "Starting Point Cloud Viewer..." << std::endl;
	    bool retrieve_cloud_ = false;
	    bool retrieve_index_ = false;
	    int highlight_size_ = 0;
	    int cloud_index;
        pcl::PointCloud<PointType>::Ptr cloud_a;
        pcl::PointCloud<PointType>::Ptr	highlight_cloud (new pcl::PointCloud<PointType>);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> highlight_color_handler (highlight_cloud, 255, 0, 0);
        retrieve_cloud_ = dm.getCloud(cloud_a);
  
        if (retrieve_cloud_){
			viewer.removePointCloud("cloud_a");
		    viewer.addPointCloud(cloud_a, "cloud_a");
		    retrieve_index_ = dm.getPointIndexSize(highlight_size_);
		    std::cout << "Highlight points size: " << highlight_size_ << std::endl;
		    if (retrieve_index_){
			    for (int n=0 ; n < highlight_size_ ; n++){
				    
				    dm.getPointCloudIndex(cloud_index, n);
				    highlight_cloud->points.push_back(cloud_a->points[cloud_index]);
				    allmarkers.points.x = cloud_a->points[cloud_index].x;
				    allmarkers.points.y = cloud_a->points[cloud_index].y;
				    allmarkers.points.z = cloud_a->points[cloud_index].z;
				    allmarkers.arraypoints.push_back(allmarkers.points);
				    
				    cv::Point2f p;
				    dm.getPixelPoint(p, n);
				    std::cout<< "pixel point" << p<<std::endl;
				    allmarkers.pixelpoints.x = p.x;
				    allmarkers.pixelpoints.y = p.y;
				    allmarkers.arraypixelpoints.push_back(allmarkers.pixelpoints);
				    
			    }
			    viewer.removePointCloud("Highlight Cloud");
			    viewer.addPointCloud (highlight_cloud, highlight_color_handler, "Highlight Cloud");
			    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "Highlight Cloud");
			    
			    std::cout<< "publishing markers "<<std::endl;
			    
			    //PUB the tracking list
			    int listsize;
			    std::vector <int> listallsize;
			    std::vector < std::vector <double> > all_position;
			    bool retrieve_list = false;
			    retrieve_list = dm.gettrackinglist(listsize, listallsize, all_position);
			    if (retrieve_list == true){
					std::cout <<" retrieved tracking list"<<std::endl;
					allmarkers.size = listsize;
					for (int n=0 ; n < listallsize.size() ; n++){
						allmarkers.allsize.push_back(listallsize[n]);
					}
					
					for (int n=0 ; n < all_position.size() ; n++){
						allmarkers.points2.x = all_position[n][0];
						allmarkers.points2.y = all_position[n][1];
						allmarkers.points2.z = all_position[n][2];
						allmarkers.arraypoints2.push_back(allmarkers.points2);
					}	
			    }
			    //marker_points.publish(allmarkers);
			    data_publish = true;
			    //loop_rate.sleep();
		    }
	    }

	    // SAVE POINT CLOUDS
	    //cloud_a->width = cloud_a->points.size ();
	    //cloud_a->height = 1;
	    //highlight_cloud->width = highlight_cloud->points.size ();
	    //highlight_cloud->height = 1;
	    //pcl::io::savePCDFileASCII(package_path_ + "/pose_estimation_frames/cloud.pcd",*cloud_a);
	    //pcl::io::savePCDFileASCII(package_path_ + "/pose_estimation_frames/highlight_cloud.pcd",*highlight_cloud);
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
				cv::imshow("Image tracking Viewer", image_display_2);
			}
		    cv::waitKey(30);
		    //if((char)cv::waitKey(30) == 27)
		    
		    viewer.spinOnce();
		    if(viewer.wasStopped ()){
			    std::cout << "Key out" << std::endl;
			    image_sub_.shutdown();
			    image_sub_3.shutdown();
			    point_cloud_sub_.shutdown();
			    viewer.close();
			    break;
		    }  
	    }
	    
	    
	   /*while (!got_image_3) {
		    ros::spinOnce();
	    }
	    image_sub_3.shutdown();*/
	    
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

