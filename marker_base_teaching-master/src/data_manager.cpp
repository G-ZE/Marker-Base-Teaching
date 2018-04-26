// IMAGE PROCESSING
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/photo.hpp>
//#include <opencv2/photo/cuda.hpp>

// POINT CLOUD PROCESSING
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/print.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// POSE ESTIMATION
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>

// UTILITIES
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include <ctime>
#include <vector>
#include <math.h>
#include <numeric> 
#include <algorithm>

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI IntensityType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::PointNormal NormalPointType;

class DataManagement
{
	private:
	
		cv::Mat tvec;
		cv::Mat rvec;
		cv::Mat camera_matrix;
		cv::Mat dist_coeffs;
		cv::Mat frame;
		cv::Mat annoted_frame_;
		cv::Mat refframe;
		cv::Mat annoted_ref_frame;
		float aruco_size = 0.08/2;
		float desc_match_thresh_;
		int icp_max_iter_;
		float icp_corr_distance_;
		int min_marker_ = 1;
		int reduced_resolution_factor_;
		
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		std::vector <int> point_id;
		std::vector <int> cloud_index;
		std::vector <int> correspondence_point_;
		std::vector <int> correspondence_database_;
		std::vector < std::vector < float > > feature_desc, database_desc_, test_desc_;
		std::vector < std::vector < int > > feature_desc_index, database_desc_index_, test_desc_index_;
		std::vector <cv::Point2f> pixel_position_;
		std::vector <int> reduced_resolution_x_range_, reduced_resolution_y_range_;
		std::string package_path_;
		int index_count = 0;
		int frame_height, frame_width;
		bool transformation_ready = false;
		bool image_ready = false;
		bool image_ready2 = false;
		bool image_ready3 = false;
		bool cloud_ready = false;
		bool pixel_point_ready = false;
		bool reading_pixel_point = false;
		bool parameters_ready = false;
		bool camera_parameters_ready = false;
		bool database_desc_ready_ = false;
		bool highlight_cloud_ready_ = false;
		bool reduced_resolution_ = false;
		pcl::PointCloud<PointType>::Ptr cloud, highlight_cloud_, model_cloud_;
		
		//FOR TRACKING
		std::vector < std::vector <double> > all_marker_position;
		std::vector < std::vector <double> > all_cloud_mp;
		std::vector < std::vector <double> > all_unloaded_cloud_mp;
		std::vector <cv::Point2f> all_marker_pixel;
		std::map <int,std::vector < std::vector <double> > > tracking_position;
		std::map <int, std::vector<cv::Point2f> > tracking_pixel;
		cv::Mat tracking_frame;
		int frame_id;
		bool computecostready = false;
		
		std::ofstream text_file_;
		bool text_file_opened_ = false;
		
	public:
	
		void setParameters(double h, double w, std::string p);
		void setCameraParameters(cv::Mat c, cv::Mat d);
		void setPixelPointReady();
		void setDescMatchThreshold(float thresh);
		void setIcpParameters(int iterations_, float threshold_);
		void setMinMarkers(int i);
		void setReducedResolution(int f);
		
		void loadTransform(cv::Mat t, cv::Mat r);
		void loadFrame(cv::Mat f);
		void loadRefframe(cv::Mat f);
		void loadCloud(pcl::PointCloud<PointType>::Ptr &c);
		void loadPixelPoint(cv::Point2f p, int id, double width, double height, double range);
		void loadDatabaseDescriptors(std::vector < std::vector < int > > index_vector, std::vector < std::vector < float > > element_vector);
		void loadDatabaseDescriptors(pcl::PointCloud<PointType>::Ptr &c);
		// To be deleted
		void loadTestDescriptors();
		
		void getTransform(cv::Mat& t, cv::Mat& r);
		bool getFrame(cv::Mat& f);
		bool getRawFrame(cv::Mat& f);
		bool getCloud(pcl::PointCloud<PointType>::Ptr &r);
		bool getHighlightCloud(pcl::PointCloud<PointType>::Ptr &c);
		bool getPixelPoint(cv::Point2f &p, int n);
		bool getPointCloudIndex(int &index, int n);
		bool getPointIndexSize(int &n);
		bool getCloudAndImageLoadStatus();
		bool getImageLoadStatus();
		bool getDatabaseDescriptor(int n, std::vector<float> &v);
		bool getCorrespondence(std::vector<int> &scene_corrs, std::vector<int> &database_corrs);
		bool markerTracking(double duration/*,std::vector < std::vector <float> > cloud_data*/);
		bool getPath(int n, std::vector < std::vector <double> > &all_position);
		bool gettrackinglist(int &size, std::vector<int> &allsize, std::vector < std::vector <double> > &all_position);
		bool getTrackingFrame(cv::Mat& f);
		double cost_function(std::vector<double> object_a, std::vector<double> object_c, double v);
		double factorial(double n);
		
		/* Documentation: Determining Matching Descriptors
		 * Determines if there are matching descriptors between a scene and those stored in a database.
		 * Requires that pixel_point_ready_ and database_desc_ready_ to be toggled true.
		 * When one match is found:
		 * 1. Code breaks.
		 * 2. Corresponding scene and database marker indexes are stored in: correspondence_point_ and correspondence_database_.
		 * 3. Label Marker Function is called.
		 * */
		bool getMatchingDescriptor();
		
		/* Documentation: Descriptor Computation
		 * After the detectMarkers function is called, the point_id and cloud_index vectors would be loaded.
		 * The descriptor computation would calculate the euclidean distance between each point and all of its surrounding points.
		 * These distance will form the elements of the descriptor of each point.
		 * The descriptors are stored into vectors: feature_desc_index and feature_desc. 
		 * Where feature_desc contains the distances and feature_desc_index contains the associated marker scene index.
		 * */
		void computeDescriptors();
		
		bool computePoseEstimate(Eigen::Matrix4f &estimated_pose_, float gc_size_, int gc_threshold_);
		
		/* Documentation: Descriptor's Elements Arrangement
		 * Prior to performing a match, the elements withina  descriptor needs to be sorted according to magnitude, from smallest to largest.
		 * The Insertion sort algorithm is used to perform this task.
		 * */
		void arrangeDescriptorsElements(std::vector < std::vector < int > > &index, std::vector < std::vector < float > > &desc);
		
		void clearPixelPoints();
		void clearDescriptors();
		void clearFrameAndCloud();
		
		bool statusTransform();
		
		void labelMarkers();
		
		void printDescriptors(std::vector < std::vector < int > >, std::vector < std::vector < float > > desc);
		
		bool detectMarkers(int blur_param_, int hsv_target_, int hsv_threshold_ , int contour_area_min_, int contour_area_max, double contour_ratio_min_, double contour_ratio_max_, double contour_length_ratio_min_,  double contour_length_ratio_max_, bool aruco_detection_);
		
		void openTextFile();
		void closeTextFile();
		void writeDescriptorToFile();
		
		/* Documenation: Circular Marker Detection
		 * This function receives an image and processes it to determine if circular markers are located within the scene.
		 * Markers located would have its pixel position recorded.
		 * If there are three or more markers located within a scene, the pixel_point_ready parameter would be set to true, enabling
		 * the computation of descriptors and subsequent functions associated with descriptors.
		 * */
		bool circleEstimation (cv::Mat& input_image, int blur_param_, int hsv_target_, int hsv_threshold_ , int contour_area_min_, int contour_area_max_,  double contour_ratio_min_, double contour_ratio_max_, double contour_length_ratio_min_, double contour_length_ratio_max_){
			
			int marker_count_ = 0;
			annoted_frame_ = input_image.clone();
			tracking_frame = input_image.clone();
			
			
			//#1 Get Image Shape Parameters
			//std::cout << "Step 1: Getting Image Shape Parameters" << std::endl;
			int row = input_image.rows;
			int col = input_image.cols;
			//cv::imwrite(package_path_ + "/pose_estimation_frames/original_image.png", input_image);
			
			//#2 Median Blur Image
			std::cout<<" bluring image"<<std::endl;
			cv::Mat image_blurred;
			cv::medianBlur(input_image, image_blurred, blur_param_);
			
			//#2 fast Mean Denoising Colour
			//cv::Mat image_denoising;
			//cv::fastNlMeansDenoisingColored(input_image, image_denoising, 3, 3, 7, 21);
			
			//cv::imwrite(package_path_ + "/pose_estimation_frames/blurred_image.png", image_blurred);
			
			//#3 Apply HSV Filtering
			std::cout<<" HSV filtering " <<std::endl;
			cv::Mat image_hsv;
			cv::Mat image_hsv_filtered;
			cv::Mat image_hsv_filtered_g;
			cv::cvtColor(image_blurred, image_hsv, CV_BGR2HSV);
			//cv::cvtColor(image_denoising, image_hsv, CV_BGR2HSV);
			cv::inRange(image_hsv,cv::Scalar(hsv_target_ - hsv_threshold_,40,40), cv::Scalar(hsv_target_ + hsv_threshold_,255,255),image_hsv_filtered);
			cv::inRange(image_hsv,cv::Scalar(32,20,20), cv::Scalar(85,255,255),image_hsv_filtered_g);
			//cv::imwrite(package_path_ + "/pose_estimation_frames/hsv_image.png", image_hsv);
			//cv::imwrite(package_path_ + "/pose_estimation_frames/hsv_filtered_image.png", image_hsv_filtered);
			//cv::imshow("HSVmask Viewer", image_hsv_filtered);
			cv::imshow("HSVmask Viewer g", image_hsv_filtered_g);
			// applying erosion and dilation
			int erosion_elem = 0;
			int erosion_size = 1;
			int erosion_type;
			int dilation_elem = 0;
			int dilation_size = 1;
			int dilation_type;
			//int const max_elem = 2;
			//int const max_kernel_size = 21;
			std::cout<<" erode and dilation "<<std::endl;
			cv::Mat erosion_dst, dilation_dst;
			
			if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
			else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
			else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }
			
			if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
			else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
			else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
			
			
			cv::Mat e_element = cv::getStructuringElement( erosion_type, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
			cv::Mat d_element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );
			
			cv::erode( image_hsv_filtered, erosion_dst, e_element );
			cv::dilate( erosion_dst, dilation_dst, d_element );
	
			
			cv::imshow("dilate Viewer", dilation_dst);
			
			//#4 Find Contours
			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			std::vector<std::vector<cv::Point> > contours_g;
			std::vector<cv::Vec4i> hierarchy_g;
			//cv::findContours(image_hsv_filtered, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			cv::findContours(dilation_dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			cv::findContours(image_hsv_filtered_g, contours_g, hierarchy_g, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			std::cout<<"contours found "<<contours.size()<<std::endl;

			
			//#5 Filter Unnecessary Contours
			

				
			//find in image
			for (int i = 0 ; i < contours.size() ; i++){
					double contour_area = cv::contourArea(contours[i]);
					double contour_length = cv::arcLength(cv::Mat(contours[i]),true);
					if((contour_area < contour_area_max_) && (contour_area > contour_area_min_)){
                        //std::cout<<"contour in area range "<<std::endl;
                        
                        //all contours in area range
                        cv::drawContours(annoted_frame_, contours, i, cv::Scalar(255, 0, 0), 1, 8);

                        bool marker_confirmed_ = false;
                        
						
						//#6 Check for Child Contours and green
						for (int j = i; j < hierarchy.size() ; j++){
							if((hierarchy[j][3]==i) && (hierarchy[j][2]==-1)){	
								double child_area_ = cv::contourArea(contours[j]);
								double contour_ratio_ = contour_area / child_area_;
								double contour_length_child_ = cv::arcLength(cv::Mat(contours[j]),true);
								double contour_length_ratio_ = contour_length / contour_length_child_;
								//checking contour value
								//std::cout<<"contour_ratio and length ratio is "<< contour_ratio_<<" and "<<contour_length_ratio_<<std::endl;
								if((contour_ratio_max_ >= contour_ratio_) && (contour_ratio_min_ <= contour_ratio_) && (contour_length_ratio_max_>= contour_length_ratio_) && (contour_length_ratio_min_<=contour_length_ratio_)){
									std::vector<std::vector<cv::Point> > con_out = std::vector<std::vector<cv::Point> >(1, contours[i]);
									std::vector<std::vector<cv::Point> > con_in = std::vector<std::vector<cv::Point> >(1, contours[j]);
									//std::cout<< "area " << contour_area <<endl;
									cv::Moments m_out = cv::moments(con_out[0], false);
									cv::Moments m_in = cv::moments(con_in[0], false);
									cv::Point2f p_out = cv::Point2f((int)(m_out.m10/m_out.m00) , (int)(m_out.m01/m_out.m00));
									cv::Point2f p_in = cv::Point2f((int)(m_in.m10/m_in.m00) , (int)(m_in.m01/m_in.m00));
									double deta_x = p_out.x - p_in.x;
									double deta_y = p_out.y - p_in.y;
									
									cv::RotatedRect ellipse;
									ellipse = cv::fitEllipse(cv::Mat(contours[i]));
									double e_x = ellipse.center.x;
									double e_y = ellipse.center.y;
									double e_width = ellipse.size.width/2.0;
									double e_height = ellipse.size.width/2.0;
									//double e_deta_x = p_out.x - e_x;
									//double e_deta_y = p_out.y - e_y;
									//double pi = 3.1415926535897;
									//double e_area = (pi*ellipse.size.width*ellipse.size.height)/4.0;
									//double e_area_ratio = e_area/contour_area;
									
									//#7 CHECKING FOR CENTER DIFF
									if ((deta_x <= 1) && (deta_x >= -1) && (deta_y <= 1) && (deta_y >= -1)){
										//CHECK FOR GREEN
										for (int k = 0 ; k < contours_g.size() ; k++){
											double contour_area_g = cv::contourArea(contours_g[k]);
											if((contour_area_g < contour_area_max_) && (contour_area_g > 10)){
												
												std::vector<std::vector<cv::Point> > con = std::vector<std::vector<cv::Point> >(1, contours_g[k]);
												cv::Moments m_g = cv::moments(con[0], false);
												cv::Point2f p_g = cv::Point2f((int)(m_g.m10/m_g.m00) , (int)(m_g.m01/m_g.m00));
												double deta_x_g = p_out.x - p_g.x;
												double deta_y_g = p_out.y - p_g.y;
												//#8 CHECKING FOR DIFF FOR GREEN AND PURPLE CENTER
												//std::cout<< "deta_x "<< deta_x<<" deta_y "<<deta_y<<std::endl; 
												if ((deta_x_g <= 3) && (deta_x_g >= -3) && (deta_y_g <= 3) && (deta_y_g >= -3)){
													//std::cout<<"marker confirmed "<<endl;
													marker_confirmed_ = true;
												}
												/*
												//#8FINDING GREEN NEAR
												for (int g = 0 ; g < contours_g[k].size() ; g++){
													cv::Point2f p_g =contours_g[k][g];
													double deta_x_g = p_out.x - p_g.x;
													double deta_y_g = p_out.y - p_g.y;
													if ((std::abs(deta_x_g) < e_width) && (std::abs(deta_x_g) < e_height) && (std::abs(deta_y_g) < e_width) && (std::abs(deta_y_g) <e_width)){
														//std::cout<<"marker confirmed "<<endl;
														marker_confirmed_ = true;
													}
													
												}
												*/
												
											}
										}
									}
								}
							}
						}
						
						//#7 Compute Centroid and give temporary ID
						if(marker_confirmed_){
							//~ std::cout << "Id: " << marker_count_ << ", area: " << contour_area << std::endl;
							std::vector<std::vector<cv::Point> > con = std::vector<std::vector<cv::Point> >(1, contours[i]);
							cv::RotatedRect ellipse;
							ellipse = cv::fitEllipse(cv::Mat(contours[i]));
							double width = ellipse.size.width;
							double height = ellipse.size.height;
							//double contour_area = cv::contourArea(contours[i]);
							double pi = 3.1415926535897;
							double range = sqrt(contour_area/pi);
							if (range>0.0){
								cv::Moments m = cv::moments(con[0], false);
								cv::Point2f p = cv::Point2f((int)(m.m10/m.m00) , (int)(m.m01/m.m00));
								cv::drawContours(annoted_frame_, con, -1, cv::Scalar(0, 255, 0), 1, 8);
								cv::circle(input_image, p, 1, cv::Scalar(0, 0, 255), -1, 8, 0);
								cv::circle(annoted_frame_, p, 1, cv::Scalar(0, 0, 255), -1, 8, 0);
								//std::cout<<" center point"<< p <<std::endl;
								std::stringstream convert;
								convert << marker_count_;
								std::string s;
								s = convert.str();
								//~ cv::putText(input_image, s, p, CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, false);
								std::cout<<"getting ready to load point "<<endl;
								//while(cloud_ready != true){
								//}
								loadPixelPoint(p, marker_count_, width, height,range);
								marker_count_++;
							}
						}
					}
			}
			//~ cv::imwrite(package_path_ + "/pose_estimation_frames/contour_marked_image.png", input_image);
			
			//#8 Return True if sufficient markers are present to make pose estimate
			if (point_id.size() >= min_marker_){
				setPixelPointReady();
				return true;
			}else{
				return false;
			}
		}
	
		// ARUCO Marker Detection function not used
		bool arucoPoseEstimation(cv::Mat& input_image, int id, cv::Mat& tvec, cv::Mat& rvec, cv::Mat& mtx, cv::Mat& dist, bool draw_axis){
			// Contextual Parameters
			//std::cout << std::endl << "Pose estimation called..." << std::endl;
			float aruco_square_size = aruco_size*2;
			bool marker_found = false;
			std::vector< int > marker_ids;
			std::vector< std::vector<cv::Point2f> > marker_corners, rejected_candidates;
			cv::Mat gray;
			
			cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
			cv::aruco::detectMarkers(gray, dictionary, marker_corners, marker_ids);	
			clearPixelPoints();
			std::cout << "Number of markers detected: " << marker_ids.size() << std::endl;
			if (marker_ids.size() > 0){
				for (int i = 0 ; i < marker_ids.size() ; i++){
					std::cout << "Marker ID found: " << marker_ids[i] << std::endl;
					
					std::vector< std::vector<cv::Point2f> > single_corner(1);
					single_corner[0] = marker_corners[i];
					
					for (int j = 0; j < 4; j++){
						//loadPixelPoint(marker_corners[i][j], marker_ids[i]);
					}
					
					cv::aruco::estimatePoseSingleMarkers(single_corner, aruco_square_size, mtx, dist, rvec, tvec);
					if (draw_axis && camera_parameters_ready){
						std::cout << "Drawing markers and axis" << std::endl;
						cv::aruco::drawDetectedMarkers(input_image, marker_corners, marker_ids);
						cv::aruco::drawAxis(input_image, mtx, dist, rvec, tvec, aruco_square_size/2);
					}
				}
				setPixelPointReady();
				marker_found = true;
			}
			else{
				std::cout << "No markers detected" << std::endl;
			}
			
			return marker_found;
		}

		void computeHighlightCloud(){
			ROS_DEBUG("DM: Computing Highlight Cloud");
			pcl::PointCloud<PointType>::Ptr c (new pcl::PointCloud<PointType> ());
			ROS_DEBUG_STREAM("DM: Cloud index size: " << cloud_index.size());
			for(int i=0; i<cloud_index.size(); i++){
				if(cloud->points[cloud_index[i]].x>0 || cloud->points[cloud_index[i]].x<=0){
					c->points.push_back(cloud->points[cloud_index[i]]);
				}else{
					ROS_DEBUG("DM: NAN point, not added to highlight cloud");
				}
			}
			highlight_cloud_ = c;
			highlight_cloud_ready_ = true;
			ROS_DEBUG("DM: Finish computing highlight cloud");
		}
};

void DataManagement::setParameters(double h, double w, std::string p){
	package_path_ = p;
	frame_height = (int) h;
	frame_width = (int) w;
	
	ROS_INFO_STREAM("DM: Parameters set. W = " << frame_width << " H = " << frame_height);
	
	parameters_ready = true;
}

void DataManagement::setCameraParameters(cv::Mat c, cv::Mat d){
	camera_matrix = c;
	dist_coeffs = d;
	camera_parameters_ready = true;
}

void DataManagement::setPixelPointReady(){
		pixel_point_ready = true;
}

void DataManagement::setDescMatchThreshold(float thresh){
	desc_match_thresh_ = thresh;
	ROS_INFO_STREAM("DM: Descriptor Match Threshold set, threshold: " << thresh);
}

void DataManagement::setIcpParameters(int iterations_, float threshold_){
	icp_max_iter_=iterations_;
	icp_corr_distance_=threshold_;
	ROS_INFO_STREAM("DM: ICP Parameters Set, iterations: " << iterations_ << " threshold: " << threshold_);
}

void DataManagement::setMinMarkers(int i){
	min_marker_ = i;
	ROS_INFO_STREAM("DM: Minimum markers set, min: " << i);
}

void DataManagement::setReducedResolution(int i){
	reduced_resolution_ = true;
	reduced_resolution_factor_ = i;
	reduced_resolution_x_range_.push_back(-1);
	reduced_resolution_x_range_.push_back(0);
	reduced_resolution_x_range_.push_back(1);
	reduced_resolution_x_range_.push_back(0);
	reduced_resolution_y_range_.push_back(0);
	reduced_resolution_y_range_.push_back(-1);
	reduced_resolution_y_range_.push_back(0);
	reduced_resolution_y_range_.push_back(1);
	ROS_INFO_STREAM("DM: Reduced Resolution Set, factor: " << i );
}

void DataManagement::loadTransform(cv::Mat t, cv::Mat r){
	tvec = t;
	rvec = r;
	transformation_ready = true;
}

void DataManagement::loadFrame(cv::Mat f){
	if (!image_ready){
		frame = f.clone();
		image_ready = true;
	}
}

void DataManagement::loadRefframe(cv::Mat f){
	if (!image_ready2){
		refframe = f.clone();
		std::cout<<"showing refframe"<<std::endl;
		//cv::imshow("image", refframe);
		image_ready2 = true;
	}
}

void DataManagement::loadCloud(pcl::PointCloud<PointType>::Ptr &c){
	if (!cloud_ready){
		cloud = c;
		cloud_ready = true;
	}
}

// Also computes corresponding cloud index with the given pixel position
void DataManagement::loadPixelPoint(cv::Point2f p, int id, double width, double height,double range){
	int index_temp;
	int index_temp_2;
	double range_x = width/2;
	double range_y = height/2;
	double range_x2 = width/5;
	double range_y2 = height/5;
	//double dist = sqrt(pow (range_x,2) + pow (range_y,2));
	double dist = range;
	std::vector < std::vector <double> > all_position;
	std::vector <double> sample_position(12);
	std::vector <double> avenge_position(12);
	std::vector <double> total_position(12);
	if (!reduced_resolution_){
		ROS_DEBUG("DM: Loading Pixel Point, no reduced resolution");
		//index_temp = (p.x)*frame_height + p.y
		//index_temp = (frame_width-p.y) * frame_width + p.x;;
		index_temp = (p.y) * frame_width + p.x;
		//~ ROS_DEBUG_STREAM("DM: Index computed: " << index_temp);
		if (cloud->points[index_temp].x>0 || cloud->points[index_temp].x<=0 ){
			cloud_index.push_back(index_temp);
			point_id.push_back(id);
			pixel_position_.push_back(p);
			sample_position[0] = cloud->points[index_temp].x;
			sample_position[1] = cloud->points[index_temp].y;
			sample_position[2] = cloud->points[index_temp].z;
			sample_position[8] = p.x;
			sample_position[9] = p.y;
			sample_position[11] = dist;
			all_cloud_mp.push_back(sample_position);
			index_count++;
			std::cout<<"points loaded "<<std::endl;
		}
		else{
			
			all_position.clear();
			for (int i=0; i<(range_x2-1); i++){
				for (int j=0; j<(range_y2-1); j++){
					int x = p.x + i;
					int y = p.y + j;
					int n_x = p.x - i;
					int n_y = p.y - j;
					index_temp = (y) * frame_width + x;
					index_temp_2 = (n_y) * frame_width + n_x;
					if (cloud->points[index_temp].x>0 || cloud->points[index_temp].x<=0 ){
						sample_position[0] = cloud->points[index_temp].x;
						sample_position[1] = cloud->points[index_temp].y;
						sample_position[2] = cloud->points[index_temp].z;
						all_position.push_back(sample_position);
					}
					if (cloud->points[index_temp_2].x>0 || cloud->points[index_temp_2].x<=0 ){
						sample_position[0] = cloud->points[index_temp_2].x;
						sample_position[1] = cloud->points[index_temp_2].y;
						sample_position[2] = cloud->points[index_temp_2].z;
						all_position.push_back(sample_position);
					}
				}
			}
			int sample_size = all_position.size();
			for (int i=0; i<sample_size; i++){
				sample_position = all_position[i];
				total_position[0] = total_position[0] + sample_position[0];
				total_position[1] = total_position[1] + sample_position[1];
				total_position[2] = total_position[2] + sample_position[2];
			}
			if (sample_size > 0){
				//ADD POINT
				avenge_position[0] = total_position[0]/((double)(sample_size));
				avenge_position[1] = total_position[1]/((double)(sample_size));
				avenge_position[2] = total_position[2]/((double)(sample_size));
				avenge_position[8] = p.x;
				avenge_position[9] = p.y;
				avenge_position[11] = dist;
				all_cloud_mp.push_back(avenge_position);
				index_temp = (p.y) * frame_width + p.x;
				cloud_index.push_back(index_temp);
				point_id.push_back(id);
				pixel_position_.push_back(p);
				index_count++;
				std::cout<<"avenging points "<<std::endl;
			}
			//~ ROS_DEBUG_STREAM("DM: Pixel point not added, probable NAN point, x: " << cloud->points[index_temp].x);
			
		}
	}
	else{
		int temp_width_ = p.x/reduced_resolution_factor_;
		int temp_height_ = p.y/reduced_resolution_factor_;
		//index_temp = temp_width_ * frame_heigth / reduced_resolution_factor_+ temp_height_;
		index_temp = temp_height_ * frame_width / reduced_resolution_factor_ + temp_width_;
		//~ ROS_DEBUG_STREAM("DM: Original point: " << p.x << " x " << p.y << " Converted: " << temp_width_ << " x " << temp_height_ << " Index: " << index_temp);
		cloud_index.push_back(index_temp);
		point_id.push_back(id);
		pixel_position_.push_back(p);
		index_count++;
	}

}

void DataManagement::loadDatabaseDescriptors(std::vector < std::vector < int > > index_vector, std::vector < std::vector < float > > element_vector){
	database_desc_index_.swap(index_vector);
	database_desc_.swap(element_vector);
	database_desc_ready_ = true;
	ROS_DEBUG("Database Descriptors Loaded");
	arrangeDescriptorsElements(database_desc_index_, database_desc_);
	printDescriptors(database_desc_index_, database_desc_);
}

// To be deleted
void DataManagement::loadTestDescriptors(){
	std::vector < int > index;
	std::vector < float > desc;
	
	index.push_back(1);
	index.push_back(2);
	desc.push_back(0.08);
	desc.push_back(0.2);
	test_desc_.push_back(desc);
	test_desc_index_.push_back(index);
	loadDatabaseDescriptors(test_desc_index_, test_desc_);
}

void DataManagement::loadDatabaseDescriptors(pcl::PointCloud<PointType>::Ptr &c){
	ROS_DEBUG("Computing Database Descriptors from Cloud");
	model_cloud_ = c;
	ROS_DEBUG_STREAM("Size of cloud: " << c->points.size());
	for(int i=0; i<c->points.size(); i++){
		//~ std::cout << "i: " << i << std::endl;
		std::vector<float> temp_desc;
		std::vector<int> temp_index;
		for (int j=0; j<c->points.size(); j++){
			//~ std::cout << "j: " << j << std::endl;
			if (i!=j){
				float delta_x = c->points[i].x - c->points[j].x;
				float delta_y = c->points[i].y - c->points[j].y;
				float delta_z = c->points[i].z - c->points[j].z;
				float s = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));
				temp_desc.push_back(s);
				temp_index.push_back(j);
			}
		}
		database_desc_index_.push_back(temp_index);
		database_desc_.push_back(temp_desc);		
	}
	database_desc_ready_ = true;
	ROS_INFO("Database Descriptors Loaded");
	arrangeDescriptorsElements(database_desc_index_, database_desc_);
	printDescriptors(database_desc_index_, database_desc_);
}

void DataManagement::getTransform(cv::Mat& t, cv::Mat& r){
	t = tvec;
	r = rvec;
}

bool DataManagement::getFrame(cv::Mat& f){
	if (image_ready){
		f = annoted_frame_.clone();
		return true;
	}
	else
		return false;
}

bool DataManagement::getRawFrame(cv::Mat& f){
	if (image_ready){
		f = frame.clone();
		return true;
	}
	else
		return false;
}

bool DataManagement::getCloud(pcl::PointCloud<PointType>::Ptr &r){
	if (cloud_ready){
		r = cloud;
		return true;
	}
	else{
		return false;
	}
}

bool DataManagement::getHighlightCloud(pcl::PointCloud<PointType>::Ptr &c){
	
	if (highlight_cloud_ready_){
		ROS_DEBUG("Returning Highlight Cloud");
		c = highlight_cloud_;
		highlight_cloud_ready_ = false;
		return true;
	}
	else{
		ROS_DEBUG("Highlight Cloud Size Zero");
		return false;
	}
}

//used
bool DataManagement::getPixelPoint(cv::Point2f &p, int n){
	if (pixel_point_ready){
		p = pixel_position_[n];
		//std::stringstream ss;
		//ss<<(n+1);
		//std::string s = ss.str(); 
		//cv::putText(annoted_frame_, s, p, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 1);
		//pixel_point_ready = false;
		return true;
	}
	else{
		return false;
	}
}

// Not used
bool DataManagement::getPointCloudIndex(int &index, int n){
	if (parameters_ready && pixel_point_ready){
		index = cloud_index[n];
		//~ std::cout << "Point cloud index returned: " << index << std::endl;
		return true;
	}
	else{
		return false;
	}
}

// Not used
bool DataManagement::getPointIndexSize(int &n){
		//~ std::cout << "Point index size returned: " << n << std::endl;
		if (point_id.size()>0 && pixel_point_ready){
			n = point_id.size();
			return true;
		}
		else{
			n=0;
			return false;
		}
}

bool DataManagement::getCloudAndImageLoadStatus(){
	if(image_ready && cloud_ready){
		return true;
	}
	else{
		return false;
	}
}

bool DataManagement::getImageLoadStatus(){
	return image_ready;
}

// Not used
bool DataManagement::getDatabaseDescriptor(int n, std::vector<float> &v){
	if(n < database_desc_.size()){
		v = database_desc_[n];
		return true;
	}else{
		return false;
	}
}

bool DataManagement::getCorrespondence(std::vector<int> &scene_corrs, std::vector<int> &database_corrs){
	if (correspondence_point_.size() > 0){
		scene_corrs = correspondence_point_;
		database_corrs = correspondence_database_;
		return true;
	}
	else{
		return false;
	}
}

bool DataManagement::markerTracking(double duration/*,std::vector < std::vector <float> > cloud_data*/){
	/*ALL PARAMETERS USED
	
	tracking_position
	tracking_pixel
	all_marker_position
	all_marker_pixel
	all_cloud_mp
	all_unloaded_cloud_mp
	pixel_position_ (x,y)
	frame_id
	DELCARE HERE
	marker_data (x,y,z vx,vy,vz,t,f,px,py,maxdist,range)
	
	*/
	bool tracked = false;
	bool reset = false;
	std::vector <double> marker_position(12);
	std::vector < double > object_b_cost(3);
	std::vector < double > best_object_b(3);
	int temp_frame_id =0;
	std::vector < std::vector < double > > list_objects_b_cost;
	double vx;
	double vy;
	double vz;
	double object_time = duration;
	double duration_time;
	int track_id = 0;
	int object_id = 0;
	double highest_cost = 0.0;
	double lowest_cost = 100;
	double highest_dist = 0.0;
	double cost_q = 0.0;
	double cost_w = 0.0;
	double total_cost = 0.0;
	int k =0;
	int l = 0;
	int global_best = 0;
	int counter =0;
	int kick_id =0;
	int block_counter = 0;
	int kick_counter = 0;
	int track_size = 0;
	double v = 0.0;
	bool block = false;
	std::vector < int> track_blocker;//for blocking tracks
	std::vector < int> cost_blocker; //for blocking cost
	int object_list_id;
	int track_blocker2;
	std::vector < int > all_object_id;
	std::vector <int> block_list;//for blocking object
	int num_object;
	int b_object;
	
	
	//TESTING
	//all_cloud_mp = cloud_data;
	
	if (all_cloud_mp.size() > 0){
		std::cout<<"cloud confirm "<<std::endl;
		frame_id = frame_id + 1;
		//FIRST LOADING
		if ((tracking_position.size() == 0)){
			std::cout<<"first tracking "<<std::endl;
			//start = std::clock();
			std::cout<<"cloud size "<<all_cloud_mp.size()<<std::endl;
			for (int i=0; i<all_cloud_mp.size(); i++){
				//std::vector < std::vector < float > > ().swap(all_marker_position);
				//std::vector <cv::Point2f> ().swap(all_marker_pixel);
				//all_marker_pixel.clear();
				all_marker_position.clear();
				all_cloud_mp[i][6] = object_time;
				marker_position[0] = all_cloud_mp[i][0];
				marker_position[1] = all_cloud_mp[i][1];
				marker_position[2] = all_cloud_mp[i][2];
				marker_position[3] = 0;
				marker_position[4] = 0;
				marker_position[5] = 0;
				marker_position[7] = frame_id;
				marker_position[6] = all_cloud_mp[i][6];
				marker_position[8] = all_cloud_mp[i][8];
				marker_position[9] = all_cloud_mp[i][9];
				marker_position[11] = all_cloud_mp[i][11];
				all_marker_position.push_back(marker_position);
				//all_marker_pixel.push_back(pixel_position_[i]);
				tracking_position[i+1] = all_marker_position;
				//tracking_pixel[i+1] = all_marker_pixel;
				//all_marker_pixel.clear();
				all_marker_position.clear();
				std::cout<<"data"<< marker_position[0] <<std::endl;
			}
			std::cout<<"track size "<<tracking_position.size()<<std::endl;
			std::cout<<"tracked is true "<<std::endl;
			tracked = true;
			
		}
		//AFTER FIRST LOADING
		
		else{
			if(all_cloud_mp.size() > 0){
				num_object = all_cloud_mp.size();
				std::cout<<"cloud size "<<all_cloud_mp.size()<<std::endl;
				//time_now = std::clock();
				//duration = ( time_now - start ) / (double) CLOCKS_PER_SEC;
				//start = std::clock();
				object_time = duration;
				std::cout<<"time "<<object_time<<std::endl;
				//UPDATING CLOUD DATA FOR MAX DISTANCE
				highest_cost = 0.0;
				for (int j=0; j<tracking_position.size(); j++){
					all_marker_position = tracking_position[j+1];
					for (int i=0; i<all_cloud_mp.size(); i++){
						marker_position[8] = all_marker_position[all_marker_position.size()-1][8];
						marker_position[9] = all_marker_position[all_marker_position.size()-1][9];
						//marker_position[2] = all_marker_position[all_marker_position.size()-1][2];
						double delta_x = marker_position[8]-all_cloud_mp[i][8];
						double delta_y = marker_position[9]-all_cloud_mp[i][9];
						//double delta_z = marker_position[2]-all_cloud_mp[i][2];
						double maxdist = sqrt(pow (delta_x,2) + pow (delta_y,2) /*+ pow (delta_z,2)*/);
						if (maxdist >= highest_dist){
							highest_dist = maxdist;
							std::cout<<"highest dist "<<maxdist<<std::endl;
						}
					
					}
					
				}
				for (int j=0; j<tracking_position.size(); j++){
					all_marker_position = tracking_position[j+1];
					all_marker_position[all_marker_position.size()-1][10] = highest_dist;
					tracking_position[j+1] = all_marker_position;
				}
				all_marker_position.clear();
				
				std::cout<<"correction step"<<std::endl;
				// CORRECTION STEP
				for (int i=0; i<all_cloud_mp.size(); i++){
					// updating cloud mp data
					all_cloud_mp[i][7] = frame_id;
					all_cloud_mp[i][6] = object_time;
					all_cloud_mp[i][5] = 0;
					all_cloud_mp[i][4] = 0;
					all_cloud_mp[i][3] = 0;
					//FIND BEST MATCH POINT IN A TRACK
					/*
					
					std::cout<<"finding match point"<<std::endl;
					for (int j=0; j<tracking_position.size(); j++){
						all_marker_position = tracking_position[j+1];
						highest_cost = 0.0;
						track_id = 0;
						object_id = 0;
						//FINDING HIGHEST COST IN TRACK
						for (int b=0; b<all_marker_position.size(); b++){
							cost_q = 0.0;
							cost_w = 0.0;
							total_cost = 0.0;
							k =0;
							while(k<=(b-1)){
								computecostready = true;
								v = (float)(k + 1);
								cost_w = cost_w + cost_function(all_marker_position[k],all_marker_position[k+1],v);
								k++;
							}
							std::cout<<"finding highest cost"<<std::endl;
							computecostready = true;
							v = (float)(b+1);
							cost_q = cost_function(all_marker_position[b],all_cloud_mp[i],v);
							total_cost = cost_q + cost_w;
							std::cout<<"total cost "<<total_cost<<std::endl;
							if (total_cost >= highest_cost){
								highest_cost = total_cost;
								track_id = j;
								object_id = b;
							}
						}
						std::cout<<"highest cost found"<<std::endl;
						object_b_cost[0] =highest_cost;
						object_b_cost[1] =track_id;
						object_b_cost[2] =object_id;
						list_objects_b_cost.push_back(object_b_cost);	
					}
					//FIND BEST GLOBAL MATCH POINT FOR ALL TRACKS
					std::cout<<"finding best global match point"<<std::endl;
					float highest_cost = 0.0;
					for (int p=0; p<list_objects_b_cost.size(); p++){
						if (list_objects_b_cost[p][0] > highest_cost){
							best_object_b[0] = list_objects_b_cost[p][0];
							best_object_b[1] = list_objects_b_cost[p][1];
							best_object_b[2] = list_objects_b_cost[p][2];
						}
					}
					list_objects_b_cost.clear();
					l = tracking_position[best_object_b[1]+1].size();
					//CHECK IF NEED MAKE CORRECTION EDGE
					temp_frame_id =0;
					global_best = best_object_b[2];
					if ( global_best < (l-1)){
						//MAKE CORRECTION EDGE
						std::cout<<"making correction edge"<<std::endl;
						temp_frame_id = tracking_position[best_object_b[1]+1][best_object_b[2]][7];
						for (int j=0; j<tracking_position.size(); j++){
							all_marker_position = tracking_position[j+1];
							counter =0;
							for (int b=0; b<all_marker_position.size(); b++){
								if (all_marker_position[b][7] > temp_frame_id){
									//ADD TO UNLOADED LIST
									all_unloaded_cloud_mp.push_back(all_marker_position[b]);
									counter++;
								}	
							}
							//KICK POINTS FROM TRACK
							std::cout<<"removing points from track"<<std::endl;
							kick_id = all_marker_position.size() - counter;
							all_marker_position.erase(all_marker_position.begin()+ kick_id, all_marker_position.begin()+ all_marker_position.size());
							tracking_position[j+1] = all_marker_position;
						}
						//CREATING AND ADD THE CORRECTED EDGE
						std::cout<<"creating correction edge"<<std::endl;
						all_marker_position = tracking_position[best_object_b[1]+1];
						marker_position = all_marker_position[all_marker_position.size()-1];
						if (object_time !=0){
							vx = (all_cloud_mp[i][0] - marker_position[0])/object_time; 
							vy = (all_cloud_mp[i][1] - marker_position[1])/object_time; 
							vz = (all_cloud_mp[i][2] - marker_position[2])/object_time;
							all_cloud_mp[i][3] = vx;
							all_cloud_mp[i][4] = vy;
							all_cloud_mp[i][5] = vz;
						}
						
						all_marker_position.push_back(all_cloud_mp[i]);
						//BLOCK THE ADD POINT FROM REPEAT
						block_list.push_back(i);
						track_blocker2 = best_object_b[1]+1;
						tracking_position[best_object_b[1]+1] = all_marker_position;
					}
					//NOTHING GO TO EXTENTION
					else{
					
					}
					
					*/ 
					
				}
				//KICK ADD POINT FROM CLOUD MP
				block_counter = 0;
				
				for (int b=0; b<block_list.size(); b++){
					std::cout<<"removing point from cloud mp"<<std::endl;
					all_cloud_mp.erase(all_cloud_mp.begin()+ (block_list[b]- block_counter));
					block_counter++;
				}
				block_list.clear();
				//EXTENTION STEP
				int frame_counter = 1;
				while (all_cloud_mp.size()>0){
					//FIND AND EXTEND FOR ALL POINTS IN UNLOAD CLOUD
					
					all_object_id.clear(); 
					std::cout<<"extention step"<<std::endl;
					while (all_unloaded_cloud_mp.size()>0){
						for(int u=0; u<all_unloaded_cloud_mp.size(); u++){
							//FINDING POINT IN SAME FRAME
							if (all_unloaded_cloud_mp[u][7] == (temp_frame_id + frame_counter)){
								std::cout<<"finding best point for track from uncloud"<<std::endl;
								track_id =0;
								object_id =0;
								highest_cost = 0.0;
								for (int j=0; j<tracking_position.size(); j++){
									for (int br=0; br<track_blocker.size();br++){
										if((j+1) == track_blocker[br]){
											block = true;
										}
									}
									if (block == false){
										cost_q = 0.0;
										all_marker_position = tracking_position[j+1];
										computecostready = true;
										v = (double)(all_marker_position.size());
										cost_q = cost_function(all_marker_position[all_marker_position.size()-1],all_unloaded_cloud_mp[u],v);
										if (cost_q >= highest_cost){
											highest_cost = cost_q;
											track_id = j+1;
											object_id = u;
										}
									}
									block = false;
								}
								if ((track_id) !=0){
									//ADD OBJECT TO TRACK
									std::cout<<"adding point to track"<<std::endl;
									all_marker_position = tracking_position[track_id];
									all_marker_position.push_back(all_unloaded_cloud_mp[object_id]);
									tracking_position[track_id] = all_marker_position;
									all_object_id.push_back(object_id);
									track_blocker.push_back(track_id);
								}
							}
							
						}
						//CLEAR TRACK BLOCKER
						track_blocker.clear();
						//REMOVE OBJECT FROM UNCLOUD LIST
						kick_counter = 0;
						for (int o=0; o<all_object_id.size(); o++){
							std::cout<<"kicking point from uncloud"<<std::endl;
							all_unloaded_cloud_mp.erase(all_unloaded_cloud_mp.begin() + (all_object_id[o]- kick_counter));
							kick_counter++;
						}
						
						all_object_id.clear();
						frame_counter++;
					}
					if (all_unloaded_cloud_mp.size() == 0){
						//FIND BEST MATCH AND EXTEND FOR ALL POINTS IN CURRENT CLOUD MP
						std::cout<<"finding match point from cloud mp"<<std::endl;
						std::cout<<"track size "<<tracking_position.size()<<std::endl;
						//FOR NEW OBJECT
						/*
						if (num_object > tracking_position.size()){
							std::cout<<"new object"<<std::endl;
							for(int j=0; j<tracking_position.size(); j++){
								highest_cost = 0.0;
								track_id = 0;
								object_id = 0;
									std::cout<<"finding highest cost for cloud mp"<<std::endl;
									std::cout<<"cloud size "<<all_cloud_mp.size()<<std::endl;
									for (int c =0; c<all_cloud_mp.size(); c++){
										for (int br=0; br<block_list.size();br++){
											if((c == block_list[br]) || ( (j+1)==track_blocker2) ){
												block = true;
											}
										}
										if (block == false){
											all_marker_position = tracking_position[j+1];
											cost_q = 0.0;
											computecostready = true;
											v= (double)(all_marker_position.size());
											cost_q = cost_function(all_marker_position[all_marker_position.size()-1],all_cloud_mp[c],v);
											std::cout<<"cost q"<<cost_q<<std::endl;
											if (cost_q >= highest_cost){
												highest_cost = cost_q;
												track_id = j+1;
												object_id = c;
											}
										}
										block = false;
									}
									//CREATING EDGE ADDING POINT FROM CLOUD MP TO TRACK
									if ((track_id) !=0){
										std::cout<<"adding point to track"<<std::endl;
										all_marker_position = tracking_position[track_id];
										marker_position = all_marker_position[all_marker_position.size()-1];
										if (duration_time != 0){
											vx = (all_cloud_mp[object_id][0] - marker_position[0])/duration_time; 
											vy = (all_cloud_mp[object_id][1] - marker_position[1])/duration_time; 
											vz = (all_cloud_mp[object_id][2] - marker_position[2])/duration_time;
											all_cloud_mp[object_id][3] = vx;
											all_cloud_mp[object_id][4] = vy;
											all_cloud_mp[object_id][5] = vz;
										}
										std::cout<<"testing adding to track object "<<object_id<<std::endl;
										all_marker_position.push_back(all_cloud_mp[object_id]);
										std::cout<<"testing adding to track"<<track_id<<std::endl;
										tracking_position[track_id] = all_marker_position;
										//REMOVING POINT FROM CLOUD MP ADDING TO LIST
										std::cout<<"testing counter list from cloud mp"<<std::endl;
										block_list.push_back(object_id);
									}
							}
							//REMOVING POINT FROM CLOUD MP
							block_counter = 0;
							for (int b=0; b<block_list.size(); b++){
								std::cout<<"removing point from cloud mp"<<std::endl;
								all_cloud_mp.erase(all_cloud_mp.begin()+ (block_list[b]- block_counter));
								block_counter++;
							}
							block_list.clear();
							//CHECK FOR ANY REMAINING POINTS IN CURRENT CLOUD
							if (all_cloud_mp.size() > 0){
								std::cout<<"creating track for remainining point in cloud"<<std::endl;
								track_size = tracking_position.size();
								for(int c =0; c<all_cloud_mp.size(); c++){
									all_marker_position.clear();
									all_marker_position.push_back(all_cloud_mp[c]);
									tracking_position[track_size +1 +c] = all_marker_position;
									all_marker_position.clear();
								}
								all_cloud_mp.clear();
							}
						}
						*/
						//ALL OBJECT
						if(tracking_position.size()>0){
							std::cout<<"for all object"<<std::endl;
							block_list.clear();
							track_blocker.clear();
							//CREATING TRACKS FOR NEW OBJECTS, CHECK THE CLOSEST POINT
							for (int c =0; c<all_cloud_mp.size(); c++){
								lowest_cost = 100;
								for(int j=0; j<tracking_position.size(); j++){
									for (int br=0; br<track_blocker.size();br++){
										if((j+1) == track_blocker[br]){
											block = true;
										}
									}
									if (block ==false){
										all_marker_position = tracking_position[j+1];
										double diff_x = all_cloud_mp[c][8] - tracking_position[j+1][all_marker_position.size()-1][8];
										double diff_y = all_cloud_mp[c][9] - tracking_position[j+1][all_marker_position.size()-1][9];
										double dist = sqrt(pow (diff_x,2) + pow (diff_y,2));
										if ((dist <= lowest_cost)){
											lowest_cost = dist;
											track_id = j+1;
											object_id =c;
										}	
									}
									block = false;
								}
								if ((lowest_cost >=100)){
									track_size = tracking_position.size();
									all_marker_position.clear();
									all_marker_position.push_back(all_cloud_mp[c]);
									tracking_position[track_size +1] = all_marker_position;
									block_list.push_back(object_id);
									track_blocker.push_back(track_id);
									all_marker_position.clear();
									std::cout<<"creating track for points in cloud"<<std::endl;
								}	
							}
							
							//FINDING SCORE FOR EACH OBJECT
							for (int c =0; c<all_cloud_mp.size(); c++){
								highest_cost = 0.0;
								track_id = 0;
								object_id = 0;
								for (int br=0; br<block_list.size();br++){
									if(c == block_list[br]){
										block = true;
									}
								}
								if (block == false){
									std::cout<<"finding highest cost for cloud mp"<<std::endl;
									std::cout<<"cloud size "<<all_cloud_mp.size()<<std::endl;
									for(int j=0; j<tracking_position.size(); j++){
										for (int br=0; br<track_blocker.size();br++){
											if((j+1) == track_blocker[br]){
												block = true;
											}
										}
										if (block == false){
											all_marker_position = tracking_position[j+1];
											//SAVING SCORE
											cost_q = 0.0;
											computecostready = true;
											v= (double)(all_marker_position.size());
											cost_q = cost_function(all_marker_position[all_marker_position.size()-1],all_cloud_mp[c],v);
											std::cout<<"cost q"<<cost_q<<" track_id "<< (j+1) <<std::endl;
											highest_cost = cost_q;
											track_id = j+1;
											object_id = c;
											
											std::cout<<"highest cost save"<<std::endl;
											object_b_cost[0] =highest_cost;
											object_b_cost[1] =track_id;
											object_b_cost[2] =object_id;
											list_objects_b_cost.push_back(object_b_cost);
										}
										block = false;
									}	
								}
								block = false;
									
							}
							block_counter = 0;
							//FINDING BEST MATCH FOR TRACK
							while(list_objects_b_cost.size()!=0){
								//BEST SCORE FOR TRACK
								
								for(int j=0; j<tracking_position.size(); j++){
									highest_cost = 0.0;
									track_id = 0;
									b_object = 0;
									for (int br=0; br<track_blocker.size();br++){
										if((j+1) == track_blocker[br]){
											block = true;
										}
									}
									
									if(block==false){
										//FINDING BEST FOR TRACK
										for (int i=0; i<list_objects_b_cost.size();i++){
											//SCORE OF THE SAME TRACK
											if(list_objects_b_cost[i][1]==(j+1)){
												for (int br=0; br<block_list.size();br++){
													if(list_objects_b_cost[i][2] == block_list[br]){
														block = true;
													}
												}
												if (block ==false){
													if ((list_objects_b_cost[i][0] >= highest_cost)){
														highest_cost = list_objects_b_cost[i][0];
														//track_id = list_objects_b_cost[i][1];
														b_object = list_objects_b_cost[i][2];
													}
												}
												block = false;	
											}
										}
										//FINDING BEST FOR OBJECT
										for (int c =0; c<all_cloud_mp.size(); c++){
											highest_cost = 0.0;
											track_id = 0;
											object_id = 0;
											for (int br=0; br<block_list.size();br++){
												if(c == block_list[br]){
													block = true;
												}
											}
											
											if ((block==false)){
												for (int i=0; i<list_objects_b_cost.size();i++){
													if(list_objects_b_cost[i][2]==(c)){
														for (int br=0; br<track_blocker.size();br++){
															if(list_objects_b_cost[i][1] == track_blocker[br]){
																block = true;
															}
														}
														
														if ((list_objects_b_cost[i][0] >= highest_cost)&&(block==false)){
															if (list_objects_b_cost[i][0] == highest_cost){
																std::cout<<"two same cost"<<std::endl;
																double time_diff =0;
																time_diff = tracking_position[track_id][tracking_position[track_id].size()-1][6] - tracking_position[list_objects_b_cost[i][1]][tracking_position[list_objects_b_cost[i][1]].size()-1][6];
																if (time_diff > 0){
																	
																}
																else if (time_diff <0){
																	highest_cost = list_objects_b_cost[i][0];
																	track_id = list_objects_b_cost[i][1];
																	object_id = list_objects_b_cost[i][2];
																}
															}
															else{
																highest_cost = list_objects_b_cost[i][0];
																track_id = list_objects_b_cost[i][1];
																object_id = list_objects_b_cost[i][2];
															}
														}
														block = false;
													}
												}
												std::cout<<"b_object "<<b_object<<" object_id "<<object_id<<" track_id "<<track_id<<" j+1 "<<(j+1)<<std::endl;
												//CREATING EDGE ADDING POINT FROM CLOUD MP TO TRACK IF THEY MATCH
												if (((track_id) !=0) && (b_object == object_id) && (track_id==(j+1)) ){
													std::cout<<"adding point to track"<<std::endl;
													all_marker_position = tracking_position[track_id];
													marker_position = all_marker_position[all_marker_position.size()-1];
													duration_time = all_cloud_mp[object_id][6] -marker_position[6];
													if (duration_time != 0){
														vx = (all_cloud_mp[object_id][8] - marker_position[8])/duration_time; 
														vy = (all_cloud_mp[object_id][9] - marker_position[9])/duration_time; 
														//vz = (all_cloud_mp[object_id][2] - marker_position[2])/duration_time;
														all_cloud_mp[object_id][3] = vx;
														all_cloud_mp[object_id][4] = vy;
														//all_cloud_mp[object_id][5] = vz;
													}
													std::cout<<"testing adding to track object "<<object_id<<std::endl;
													all_marker_position.push_back(all_cloud_mp[object_id]);
													std::cout<<"testing adding to track "<<track_id<<std::endl;
													tracking_position[track_id] = all_marker_position;
													//REMOVING POINT FROM CLOUD MP ADDING TO LIST
													std::cout<<"testing counter list from cloud mp"<<std::endl;
													block_list.push_back(object_id);
													track_blocker.push_back(track_id);
													block_counter++;
												}
											}
											block =false;
										}
									}
									block=false;
								}
								
								
								//CHECK FOR ANY REMAINING POINTS IN CURRENT CLOUD
								if (track_blocker.size()==tracking_position.size()){
									if (all_cloud_mp.size() > block_list.size()){
										std::cout<<"creating track for remainining point in cloud"<<std::endl;
										track_size = tracking_position.size();
										for(int c =0; c<all_cloud_mp.size(); c++){
											for (int b=0; b<block_list.size(); b++){
												if((c == block_list[b])){
													block = true;
												}
											}
											if (block == false){
												all_marker_position.clear();
												all_marker_position.push_back(all_cloud_mp[c]);
												tracking_position[track_size +1] = all_marker_position;
												all_marker_position.clear();
											}
											block = false;
										}
										list_objects_b_cost.clear();
										all_cloud_mp.clear();
										block_list.clear();
										track_blocker.clear();
									}
									else{
										list_objects_b_cost.clear();
										all_cloud_mp.clear();
										block_list.clear();
										track_blocker.clear();
									}
								}
								//CONDITION FOR FINISHING EXTENSION
								else if(all_cloud_mp.size() == block_list.size()){
									list_objects_b_cost.clear();
									all_cloud_mp.clear();
									block_list.clear();
									track_blocker.clear();
								}
								
							}
						}
						//all_object_id.clear();	
					}
					
				}	
			}
			tracked = true;
		}
		all_cloud_mp.clear(); 
		if (tracked == true){
			std::cout<< " starting to show marker "<<std::endl;
			for (int i=0; i<tracking_position.size(); i++){
				//all_marker_pixel = tracking_pixel[i+1];
				//cv::Point2f p = all_marker_pixel[all_marker_pixel.size() -1];
				all_marker_position = tracking_position[i+1];
				cv::Point2f p;
				p.x = all_marker_position[all_marker_position.size()-1][8];
				p.y = all_marker_position[all_marker_position.size()-1][9];
				std::stringstream convert;
				convert << i+1;
				std::string s;
				s = convert.str();
				//cv::putText(tracking_frame, s, p, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 3, 8, false);
				//cv::circle(tracking_frame, p, 1, cv::Scalar(0, 0, 255), -1, 8, 0);
				//TESTING
				
				if(all_marker_position.size()>1){
					cv::Point2f p1;
					cv::Point2f p2;
					for (int j=0; j<(all_marker_position.size()/2); j++){
						p1.x = all_marker_position[all_marker_position.size()-(2+j)][8];
						p1.y = all_marker_position[all_marker_position.size()-(2+j)][9];
						p2.x = all_marker_position[all_marker_position.size()-(1+j)][8];
						p2.y = all_marker_position[all_marker_position.size()-(1+j)][9];
						cv::line(frame, p1,p2,cv::Scalar(255, 0, 255),1,8,0);
					}
				}
				
				cv::putText(frame , s, p, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 3, 8, false);
				cv::circle(frame , p, 1, cv::Scalar(0, 0, 255), -1, 8, 0);
				annoted_frame_=frame.clone();
				//cv::putText(annoted_frame_, s, p, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 3, 8, false);
				//cv::circle(annoted_frame_, p, 1, cv::Scalar(0, 0, 255), -1, 8, 0);
				//all_marker_pixel.clear();
				all_marker_position.clear();
				std::cout<< " showing marker "<<std::endl;
				image_ready3 = true;
			}
			return true;
		}
		else{
			return false;
		}
	}
	else
		std::cout<< " no marker to track "<<std::endl;
		return true;
	
	
}

double DataManagement::factorial(double n){
		double pi = 3.1415926535897;
		if (int(n) !=n){
			if(n > 0.5){
				return n * factorial(n - 1);
			}
			else{
				return sqrt(pi);
			}
		}
		else{
			if(n > 1){
				return n * factorial(n - 1);
			}
			else{
				return 1.0;
			}
		}
}

double DataManagement::cost_function(std::vector<double> object_a, std::vector<double> object_c, double v){
	/* List for parameters
	 * object a and object c
	 * est object a  
	 * d of object c - object a and est d of est object a - object a
	 * duration t from a to c
	 * similar vector
	 * probabilistic p or the cost
	 * and number of match object
	 */
	//std::vector <float> marker_a(10);
	//std::vector <float> marker_c(10);
	std::vector <double> d(3);
	std::vector <double> est_d(3);
	double dot_d;
	double t;
	double duration;
	std::vector <double> sim_v(3);
	double proba_p;
	double proba_p_d;
	double r_fact;
	double pi = 3.1415926535897;
	double dist_a;
	double sim_v_pow;
	double sim_d_pow;
	v=20.0;
	std::cout<<"finding cost"<<std::endl;
	if(computecostready == true){
		duration = std::abs(object_c[6]-object_a[6]);
		d[0] = object_c[8] - object_a[8];
		d[1] = object_c[9] - object_a[9];
		//d[2] = object_c[2] - object_a[2];
		//t=object_a[6];
		est_d[0] = (object_a[8] + object_a[3]*duration)- object_a[8];
		est_d[1] = (object_a[9] + object_a[4]*duration) - object_a[9];
		//est_d[2] = (object_a[2] + object_a[5]*duration) - object_a[2];
		
		
		std::cout<<"duration "<<duration<<std::endl;
		double dist = sqrt(pow (d[0],2) + pow (d[1],2) /*+ pow (d[2],2)*/);
		double est_dist = sqrt(pow (est_d[0],2) + pow (est_d[1],2) /*+ pow (est_d[2],2)*/);
		
		dot_d = est_d[0]*d[0] + est_d[1]*d[1] /*+ est_d[2]*d[2]*/;
		dist_a = object_a[10];
		std::cout<<"dot_d "<<dot_d<<std::endl;
		std::cout<<"dist "<<dist<<std::endl;
		std::cout<<"est_dist "<<est_dist<<std::endl;
		std::cout<<"dist_a "<<dist_a<<std::endl;
		std::cout<<"range "<<object_a[11]<<std::endl;
		
		std::cout<<"computing similar vel"<<std::endl;
		//TO CHECK FOR NAN
		
		if ((dist_a >0) || (dist_a < 0)){
		}
		else{
			dist_a = 1;
		}
		
		sim_v[0] = 1.0 - (dot_d)/((std::abs(dist))*(std::abs(est_dist)));
		sim_v[1] = /*(std::abs(duration))*/(std::abs(std::abs(dist) - std::abs(est_dist))/(std::abs(dist) + std::abs(est_dist)));
		sim_v[2] = (std::abs(duration))*(std::abs(dist)/(std::abs(dist_a)));
		
		//TO CHECK FOR NAN
		if ((sim_v[0] >=0) || (sim_v[0] < 0)){
			if((dist >object_a[11]) && (est_dist == 0)){
				sim_v[0] = 0.0;
			}
			else if ((dist <= object_a[11]) && (est_dist == 0)){
				sim_v[0] = 0.0;
			}
			else if ((dist <= object_a[11]) && (est_dist != 0)){
				sim_v[0] = 0.0;
			}
			
			
		}
		else{
			if((dist >object_a[11]) && (est_dist == 0)){
				sim_v[0] = 0.0;
			}
			else if ((dist <= object_a[11]) && (est_dist == 0)){
				sim_v[0] = 0.0;
			}
			else if ((dist <= object_a[11]) && (est_dist != 0)){
				sim_v[0] = 0.0;
			}
			
		}
		if ((sim_v[1] >=0) || (sim_v[1] < 0)){
			if((dist >object_a[11]) && (est_dist == 0)){
				est_dist = 1;
				sim_v[1] = /*(std::abs(duration))*/(std::abs(std::abs(dist) - std::abs(est_dist))/(std::abs(dist) + std::abs(est_dist)));
			}
			else if ((dist <= object_a[11]) && (est_dist == 0)){
				sim_v[1] = 0.0;
			}
			else if ((dist <= object_a[11]) && (est_dist != 0)){
				sim_v[1] = 0.0;
			}
		}
		else{
			if ((dist <= object_a[11]) && (est_dist == 0)){
				sim_v[1] = 0.0;
			}
			
		}
		if ((sim_v[2] >=0) || (sim_v[2] < 0)){
		}
		else{
			sim_v[2] = 0;
		}
		
		std::cout<<"sim vel "<<sim_v[0]<<std::endl;
		std::cout<<"sim vel2 "<<sim_v[1]<<std::endl;
		std::cout<<"sim vel3 "<<sim_v[2]<<std::endl;
		std::cout<<"factorial"<<factorial(((v+1.0)/2.0))<<"v "<<v<<std::endl;
		sim_v_pow =(pow(sim_v[0],2) + pow(sim_v[1],2) /*+ pow(duration,2)*/ );
		sim_d_pow = pow(sim_v[2],2);
		std::cout<<"computing probailistic p"<<std::endl;
		proba_p = (factorial(((v+1.0)/2.0))/(sqrt(v*pi)*factorial(v/2.0))) * (1.0/pow ((1.0+(sim_v_pow/v)) ,((v+1.0)/2.0)));
		proba_p_d = (factorial(((v+1.0)/2.0))/(sqrt(v*pi)*factorial(v/2.0))) * (1.0/pow ((1.0+(sim_d_pow/v)) ,((v+1.0)/2.0)));
		std::cout<<"probailistic p"<<proba_p<<"and p_d"<<proba_p_d<<std::endl;
		if ((proba_p >=0) || (proba_p < 0)){
			computecostready = false;
			return (proba_p);
		}
		else{
			computecostready = false;
			std::cout<<"probailistic p is a nan point"<<std::endl;
			return 0.0;
		}
	}
	else{
		std::cout<<" no objects given "<<std::endl;
		return 0.0;
	}
}

bool DataManagement::getPath(int n, std::vector < std::vector <double> > &all_position){
	
	if (tracking_position[n].size() >0){
		all_position = tracking_position[n];
		return true;
	}
	else{
		return false;
	}
	
}
bool DataManagement::gettrackinglist(int &size, std::vector<int> &allsize, std::vector < std::vector <double> > &all_position){
	
	if (tracking_position.size() >0){
		size = tracking_position.size();
		for (int i=0; i<tracking_position.size(); i++){
			allsize.push_back(tracking_position[i+1].size());
			all_marker_position = tracking_position[i+1];
			for (int j=0; j<all_marker_position.size(); j++){
				all_position.push_back(all_marker_position[j]);
			}
		}
		return true;
	}
	else{
		return false;
	}
}

bool DataManagement::getTrackingFrame(cv::Mat& f){
	if (image_ready3){
		f = tracking_frame.clone();
		return true;
	}
	else
		return false;
}

bool DataManagement::getMatchingDescriptor(){
	if (pixel_point_ready && database_desc_ready_){
		ROS_DEBUG("DM: Get Matching Descriptors...");
		std::vector < int > match_point_;
		std::vector < int > match_database_;
		bool match_found_ = false;
		for (int n=0; n<feature_desc_index.size(); n++){
			int match_count_ = 0;
			for (int m=0; m<database_desc_index_.size(); m++){
				ROS_DEBUG_STREAM("DM: Checking feature: " << n << " with Database: " << m);
				std::vector < int > ().swap(match_point_);
				std::vector < int > ().swap(match_database_);
				match_point_.push_back(n);
				match_database_.push_back(m);
				int j = 0;
				int count = 0;
				for (int i=0; i< feature_desc_index[n].size() ; i++){
					if ((feature_desc[n][i]>=database_desc_[m][j] - desc_match_thresh_) && (feature_desc[n][i]<=database_desc_[m][j] + desc_match_thresh_)){
						ROS_DEBUG("DM: Element Matched");
						match_point_.push_back(feature_desc_index[n][i]);
						match_database_.push_back(database_desc_index_[m][j]);
						j++;
						count++;
					}
					else if (feature_desc[n][i] >= database_desc_[m][j] + desc_match_thresh_){
						ROS_DEBUG("DM: Next database element");
						j++;
					}
					else{
						ROS_DEBUG("DM: Not a match");
						break;
					}
				}
				if (count == 3){
					ROS_DEBUG("DM: Match Found");
					match_found_ = true;
					break;
				}
			}
			if (match_found_)
				break;
		}
		if (match_found_){
			ROS_DEBUG("DM: Descriptor Match Found");
			correspondence_point_.swap(match_point_);
			correspondence_database_.swap(match_database_);
			labelMarkers();
		}else{
			ROS_DEBUG("No Match Found");
		}
	}else if(!pixel_point_ready){
		ROS_DEBUG("Pixel points not ready, cannot determine match");
	}
	else if(!database_desc_ready_){
		ROS_DEBUG("Database Descriptors not loaded, cannot determine match");
	}
}

void DataManagement::computeDescriptors(){
	if(pixel_point_ready && cloud_ready){
		int n = point_id.size();
		ROS_DEBUG_STREAM("DM: Number of features detected: " << n);
		for (int i = 0 ; i < n ; i++){
			std::vector<float> temp_desc;
			std::vector<int> temp_index;
			for (int j = 0 ; j < n ; j++){
				if(i!=j){
					float delta_x = cloud->points[cloud_index[i]].x - cloud->points[cloud_index[j]].x;
					float delta_y = cloud->points[cloud_index[i]].y - cloud->points[cloud_index[j]].y;
					float delta_z = cloud->points[cloud_index[i]].z - cloud->points[cloud_index[j]].z;
					float s = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));
					temp_desc.push_back(s);
					temp_index.push_back(point_id[j]);
				}
			}
			feature_desc_index.push_back(temp_index);
			feature_desc.push_back(temp_desc);
		}
		ROS_DEBUG("DM: Finished computing descriptors... ");
		//arrangeDescriptorsElements(feature_desc_index, feature_desc);
		printDescriptors(feature_desc_index, feature_desc);
	}
	else if(!pixel_point_ready){
		ROS_DEBUG("Pixel points not ready, cannot compute descriptor");
	}
	else if(!cloud_ready){
		ROS_DEBUG("Point cloud not ready, cannot compute descriptor");
	}
}

bool DataManagement::computePoseEstimate(Eigen::Matrix4f &estimated_pose_, float gc_size_, int gc_threshold_){
	bool pose_found_ = false;
	if (correspondence_point_.size()>2){
		ROS_DEBUG("DM: Begin Pose Estimation");
				
		//STEP 1: SET CORRESPONDENCE
		pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
		ROS_DEBUG_STREAM("DM: Number of Correspodnences: " << correspondence_point_.size());
		for (int i=0; i<correspondence_point_.size(); i++){
			pcl::Correspondence corr (correspondence_database_[i], correspondence_point_[i], 0);
			ROS_DEBUG_STREAM("DM: Scene: " << correspondence_point_[i] << " Model: " << correspondence_database_[i]);
			model_scene_corrs->push_back (corr);
		}
		
		//STEP 2: PERFORM GEOMETRIC CONSISTENCY GROUPING
		ROS_DEBUG("DM: Begin GCG");
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
		std::vector<pcl::Correspondences> clustered_corrs;  
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize (gc_size_);
		gc_clusterer.setGCThreshold (gc_threshold_);
		gc_clusterer.setInputCloud (model_cloud_);
		gc_clusterer.setSceneCloud (highlight_cloud_);
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);
		ROS_DEBUG_STREAM("DM: Model instances found: " << rototranslations.size ());        
		if (rototranslations.size ()== 0){
			ROS_DEBUG("DM: No instance found");
		}
		else{
			// STEP 3: ICP
			pcl::PointCloud<PointType>::Ptr icp_cloud_ (new pcl::PointCloud<PointType> ());
			pcl::transformPointCloud (*model_cloud_, *icp_cloud_ , rototranslations[0]);
			pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaximumIterations (icp_max_iter_);
      icp.setMaxCorrespondenceDistance (icp_corr_distance_);
      icp.setInputTarget (highlight_cloud_);
      icp.setInputSource (icp_cloud_);
      pcl::PointCloud<PointType>::Ptr icp_registered_ (new pcl::PointCloud<PointType>);
      icp.align (*icp_registered_);
      Eigen::Matrix4f icp_pose_ = rototranslations[0].block<4,4>(0,0);
      if (icp.hasConverged ())
      {
        ROS_DEBUG_STREAM("DM: Instance Aligned");
				estimated_pose_ = icp.getFinalTransformation().cast<float>()*icp_pose_;
				pose_found_ = true;
      }
      else
      {
        ROS_DEBUG("DM: Not Aligned!");
      }
			ROS_DEBUG("DM: ICP Completed");    
		}
		
	}else{
		ROS_DEBUG("DM: Insufficient points to perform pose estimation");	
	}
	return pose_found_;
}

void DataManagement::arrangeDescriptorsElements(std::vector < std::vector < int > > &index, std::vector < std::vector < float > > &	desc){
	 
	ROS_DEBUG("DM: Sorting descriptors according to magnitude...");
	ROS_DEBUG_STREAM("DM: Number of features: " << desc.size());
	if(desc.size()==1){
		ROS_DEBUG("Unable to compute descriptors, only one feature");
	}else{
		for (int n=0; n < desc.size() ; n++){
			bool lowest_score = true;
			int front_of_index;
			std::vector <float> sorted_desc;
			std::vector <int> sorted_index;
			sorted_desc.push_back(desc[n][0]);
			sorted_index.push_back(index[n][0]);
			for (int i=1; i<desc[n].size(); i++){    
				lowest_score = true;  
				for (int  j=0 ; j < sorted_desc.size() ; j++){   
					if(desc[n][i] <= sorted_desc[j]){
						front_of_index = j;
						lowest_score = false;
						break;
					}       
				}
				if (lowest_score==false){
					sorted_desc.insert(sorted_desc.begin() + front_of_index, desc[n][i]);     
					sorted_index.insert(sorted_index.begin() + front_of_index, index[n][i]);           
				}
				else if(lowest_score==true){
					sorted_desc.push_back(desc[n][i]);
					sorted_index.push_back(index[n][i]);
				}
			}
			index[n].swap(sorted_index);
			desc[n].swap(sorted_desc);
		}
		ROS_DEBUG("DM: Descriptors Sorted...");
	}
}

void DataManagement::clearPixelPoints(){
	std::vector <int> ().swap(point_id);
	std::vector <int> ().swap(cloud_index);
	std::vector <int> ().swap(correspondence_point_);
	std::vector <int> ().swap(correspondence_database_);
	std::vector <cv::Point2f> ().swap(pixel_position_);
	index_count = 0;
	pixel_point_ready = false;
}

void DataManagement::clearDescriptors(){
	std::vector < std::vector < float > > ().swap(feature_desc);
	std::vector < std::vector < int > > ().swap(feature_desc_index);
	ROS_DEBUG("Descriptors cleared");
}

void DataManagement::clearFrameAndCloud(){
	image_ready = false;
	image_ready2 = false;
	image_ready3 = false;
	cloud_ready = false;
}

bool DataManagement::statusTransform(){
	return transformation_ready;
}

void DataManagement::labelMarkers(){
	for(int i=0; i<correspondence_point_.size(); i++){
		for(int j=0; j<point_id.size(); j++){
			if(point_id[j]==correspondence_point_[i]){
				std::stringstream convert;
				convert << correspondence_database_[i];
				std::string s;
				s = convert.str();
				cv::putText(annoted_frame_, s, pixel_position_[j], CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, false);
			}
		}
	}
}

void DataManagement::printDescriptors(std::vector < std::vector < int > > index, std::vector < std::vector < float > > desc){
	for( int i = 0; i < index.size() ; i++){
		std::stringstream convert;
		std::string s;
		convert << " Descriptor (";
		convert << i;
		convert << ") : ";
		for (int j = 0 ; j < desc[i].size() ; j++){
			convert << desc[i][j];
			convert << "(";
			convert << index[i][j];
			convert << "), ";
		}
		s = convert.str();
		ROS_INFO_STREAM(s);
	}
}

bool DataManagement::detectMarkers(int blur_param_, int hsv_target_, int hsv_threshold_ , int contour_area_min_, int contour_area_max , double contour_ratio_min_, double contour_ratio_max_, double contour_length_ratio_min_, double contour_length_ratio_max_, bool aruco_detection_){
	if(image_ready){
		bool marker_found = false;
		if(aruco_detection_){
			marker_found = arucoPoseEstimation(frame, 0, tvec, rvec, camera_matrix, dist_coeffs, true);
		}else{
			//cv::imshow("image", refframe);
			std::cout <<"start circleEstimation"<< std::endl;
			
			marker_found = circleEstimation(frame, blur_param_, hsv_target_, hsv_threshold_ , contour_area_min_, contour_area_max, contour_ratio_min_, contour_ratio_max_, contour_length_ratio_min_, contour_length_ratio_max_);
			//marker_found = circleEstimation2(frame, refframe, blur_param_, hsv_target_, hsv_threshold_ , contour_area_min_, contour_area_max, contour_ratio_min_, contour_ratio_max_, contour_length_ratio_min_, contour_length_ratio_max_);
			//cv::imshow("ref image", annoted_ref_frame);
				
		}
		
		
		if (marker_found){
			computeHighlightCloud();
		}
		
		return marker_found;
	}else{
		ROS_DEBUG("Image not ready, cannot detect markers");
		return false;
	}
}

void DataManagement::openTextFile(){
	ROS_INFO_STREAM("DM: Opening Text File in " << package_path_ << "/example.txt");
	std::string text_file_path_ = package_path_ + "/example.txt";
	text_file_.open (text_file_path_.c_str());
	if(text_file_.is_open()){
		ROS_INFO("DM: Text file open checked");
		text_file_opened_=true;
	}
	else{
		ROS_INFO("DM: Text file not opened");
	}
}

void DataManagement::closeTextFile(){
  if(text_file_opened_){
		ROS_INFO("DM: Closing Text File");
		text_file_.close();
		text_file_opened_=false;
	}else{
		ROS_INFO("DM: Cannot close text file, not opened initially");
	}
}

void DataManagement::writeDescriptorToFile(){
	ROS_INFO("DM: Writing Descriptor to Text File");
	for(int i=0; i<correspondence_point_.size(); i++){
		if(correspondence_database_[i]==0){
			for(int j=0; j<feature_desc[correspondence_point_[i]].size(); j++){
				text_file_ << feature_desc[correspondence_point_[i]][j];
				if(j==(feature_desc[correspondence_point_[i]].size()-1)){
					text_file_ << "\n";
				}else{
					text_file_ << ",";
				}
			}
			break;
		}
	}
}

