#include "darknet_ros/YoloROSTracker.hpp"

// libraries for pose estimation
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include <vector>
#include <math.h>
#include <opencv2/features2d.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

// libraries for file streams
#include <iostream>
#include <fstream>
#include <sstream>

// library for utc time for logging
//#include <ctime>

#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif


// %%% Parameters for pose estimation %%%


  // flag for activating pose estimation
bool do_pose_estimate_left, do_pose_estimate_right; // activated by the cameras
bool do_mono = true; // activated by the user
bool do_stereo = true; // activated by the user

  // marker = model outputs, bbox = assigned when doing pose estimation
cv::Mat marker1_left, marker2_left, marker1_right, marker2_right, inputImage_l, inputImage_r, bbox_left_1, bbox_left_2, bbox_right_1, bbox_right_2; //update
int imageWidth = 1280;
int imageHeight = 1024;

  // bbox coordinates copies (maximum two bboxes from each camera view)
float xmin_left_1, ymin_left_1, xmin_left_2, ymin_left_2, xmin_right_1, ymin_right_1, xmin_right_2, ymin_right_2;
float xmin_left, ymin_left, xmin_right, ymin_right; // remove later

  // aruco marker detection parameters 
std::vector< int > markerIds_l_1, markerIds_l_2, markerIds_r_1, markerIds_r_2; 
std::vector< int > markerIds_l, markerIds_r; //remove later
std::vector< std::vector<cv::Point2f> > markerCorners_l_1, markerCorners_l_2, rejectedCandidates_l_1, rejectedCandidates_l_2, markerCorners_r_1, markerCorners_r_2, rejectedCandidates_r_1, rejectedCandidates_r_2, markerCorners_r, rejectedCandidates_r; //update
std::vector< std::vector<cv::Point2f> > markerCorners_l, rejectedCandidates_l; //remove later

cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

//text files to log pose estimate
const char *path_log_all="/home/oysteinvolden/Desktop/logging/log_all.txt";
std::ofstream log_all(path_log_all);
const char *path_stereo_m1="/home/oysteinvolden/Desktop/logging/log_stereo_m1.txt";
std::ofstream log_stereo_m1(path_stereo_m1); 
const char *path_stereo_m2="/home/oysteinvolden/Desktop/logging/log_stereo_m2.txt";
std::ofstream log_stereo_m2(path_stereo_m2); 
const char *path_mono_left_m1="/home/oysteinvolden/Desktop/logging/log_mono_left_m1.txt";
std::ofstream log_mono_left_m1(path_mono_left_m1);
const char *path_mono_left_m2="/home/oysteinvolden/Desktop/logging/log_mono_left_m2.txt";
std::ofstream log_mono_left_m2(path_mono_left_m2); 
const char *path_mono_right_m1="/home/oysteinvolden/Desktop/logging/log_mono_right_m1.txt";
std::ofstream log_mono_right_m1(path_mono_right_m1); 
const char *path_mono_right_m2="/home/oysteinvolden/Desktop/logging/log_mono_right_m2.txt";
std::ofstream log_mono_right_m2(path_mono_right_m2); 


//publish cam measurement to ros
//std_msgs::String msg;
//std::stringstream ss;

int ColumnOfNewImage;
int RowsOfNewImage;


namespace darknet_ros {

YoloROSTracker::YoloROSTracker(ros::NodeHandle nh2)
  : nodeHandle(nh2),
    imageTransport(nodeHandle)
  {
    ROS_INFO("[YoloROSDetector] Node started.");
 
    if(!readParameters()) ros::requestShutdown();
    initROS();
    initdarknet();
    ros::Rate rate(5); //set freq for loop
    time_stamp_start = std::chrono::steady_clock::now(); //define relative start time - measure for time stamp
    curr_time = std::time(nullptr); //utc time start
    while(true){
      captureThread();
      trackThread();
      darknetThread();
      publishResult();
      if(!ros::ok()){
	log_all.close(); //close log file for each shutdown
	log_stereo_m1.close(); 
	log_stereo_m2.close();
	log_mono_left_m1.close();
	log_mono_left_m2.close();
	log_mono_right_m1.close();
	log_mono_right_m2.close();
	break;
      }
      rate.sleep();
      ros::spinOnce();
    }
  }
YoloROSTracker::~YoloROSTracker(){
    exit_flag = true;
    if(t_cap.joinable()) t_cap.join();
    if(t_detect.joinable()) t_detect.join();
}

bool YoloROSTracker::readParameters()
    {
    // Load common parameters.
    nodeHandle.param("image_view/enable_opencv", viewImage_, true);
    nodeHandle.param("image_view/wait_key_delay", waitKeyDelay_, 3);
    nodeHandle.param("image_view/enable_console_output", enableConsoleOutput_, false);

    // Check if Xserver is running on Linux.
    if (XOpenDisplay(NULL)) {
      // Do nothing!
      ROS_INFO("[YoloObjectDetector] Xserver is running.");
    } else {
      ROS_INFO("[YoloObjectDetector] Xserver is not running.");
      viewImage_ = false;
    }
    // Set vector sizes.
    nodeHandle.param("yolo_model/detection_classes/names", classLabels,
                      std::vector<std::string>(0));
    return true;
  }
void YoloROSTracker::initROS()
  {
  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  std::string detectionBBoxTopicName; //bbox
  std::string detectionBBoxTopicName2; //bbox
  std::string bboxCoordTopicName; //bbox
  std::string bboxCoordTopicName2; //bbox
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;
  std::string cameraMeasurementTopicName; //cam measurements

  nodeHandle.param("subscribers/camera_reading/topic", cameraTopicName,
                  std::string("/camera/image_raw"));
  nodeHandle.param("subscribers/bbox_reading/topic", detectionBBoxTopicName,
                  std::string("/darknet_ros/bbox_image")); // bbox subscriber
  nodeHandle.param("subscribers/bbox_reading_2/topic", detectionBBoxTopicName2,
                  std::string("/darknet_ros/bbox_image_2")); // bbox subscriber
  nodeHandle.param("subscribers/bbox_coord_1/topic", bboxCoordTopicName,
                  std::string("/darknet_ros/bbox_coord_1")); // bbox coord subscriber
  nodeHandle.param("subscribers/bbox_coord_2/topic", bboxCoordTopicName2,
                  std::string("/darknet_ros/bbox_coord_2")); // bbox coord subscriber
  nodeHandle.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle.param("publishers/object_detector/topic", objectDetectorTopicName,
                  std::string("found_object"));
  nodeHandle.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle.param("publishers/bounding_boxes/topic", boundingBoxesTopicName,
                  std::string("bounding_boxes"));
  nodeHandle.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle.param("publishers/detection_image/topic", detectionImageTopicName,
                    std::string("detection_image"));
  nodeHandle.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle.param("publishers/detection_image/latch", detectionImageLatch, true);
  nodeHandle.param("publishers/camera_measurements/topic", cameraMeasurementTopicName, std::string("camera measurements"));

  imageSubscriber = imageTransport.subscribe(cameraTopicName, cameraQueueSize,
                                             &YoloROSTracker::cameraCallback, this);
  
  bboxSubscriber = imageTransport.subscribe(detectionBBoxTopicName, 1,
                                            &YoloROSTracker::cameraCallback2, this); // bbox subscriber - marker1

  bboxSubscriber2 = imageTransport.subscribe(detectionBBoxTopicName2, 1,
		  			    &YoloROSTracker::cameraCallback3, this); // bbox subscriber - marker2

  bboxCoord = nodeHandle.subscribe(bboxCoordTopicName, 1, &YoloROSTracker::bboxCoordCallback, this); // bbox coord subscriber

  bboxCoord2 = nodeHandle.subscribe(bboxCoordTopicName2, 1, &YoloROSTracker::bboxCoordCallback2, this); // bbox coord subscriber 

  boundingBoxesPublisher = nodeHandle.advertise<darknet_ros_msgs::BoundingBoxes>(
                                             boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
                                             detectionImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                             detectionImageQueueSize,
                                             detectionImageLatch);

  detectionImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                              detectionImageQueueSize,
                                              detectionImageLatch);

  cameraMeasurementPublisher = nodeHandle.advertise<std_msgs::String>(cameraMeasurementTopicName, 1000); //publish camera measurements to ros

}
void YoloROSTracker::initdarknet()
  {
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  nodeHandle.param("yolo_model/threshold/value", thresh, (float)0.3);
  nodeHandle.param("yolo_model/weight_file/name", weightsModel, std::string("yolov3.weights"));
  nodeHandle.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;

  nodeHandle.param("yolo_model/config_file/name", configModel, std::string("yolov3.cfg"));
  nodeHandle.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;

  dataPath = darknetFilePath_;
  dataPath += "/data";

  detector = new Detector(configPath, weightsPath);
  detector->nms = 0.02;
 #ifdef TRACK_OPTFLOW
  detector->wait_stream = true;
 #endif
  passed_flow_frames = 0;
  exit_flag = false;
  consumed = true;
  fps_det_counter = 0;
  fps_cap_counter = 0;
  current_cap_fps = 0;
  current_det_fps = 0;
  }
void YoloROSTracker::darknetThread(){
  if (!t_detect.joinable()){
  t_detect = std::thread([&]() {
    auto current_image = det_image;
    consumed = true;
    ROS_INFO("Started darknet thread");
    while (current_image.use_count() > 0 && !exit_flag) {
           ROS_INFO("Reference Count > 0");
           auto result = detector->detect_resized(*current_image, frame_size.width, frame_size.height, thresh, false);
           ++fps_det_counter;
           std::unique_lock<std::mutex> lock(mtx);
           thread_result_vec = result;
           consumed = true;
           ROS_INFO("Started darknet thread");
           cv_detected.notify_all();
           if (detector->wait_stream) while (consumed && !exit_flag) cv_pre_tracked.wait(lock);
           current_image = det_image;
      }
    });
  }
}
void YoloROSTracker::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
  {
  ROS_DEBUG("[YoloROSDetector] ROS image received.");

  cv_bridge::CvImagePtr cv_ptr;

  try {
    ROS_INFO("Callback Called");
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cap_frame = cv_ptr->image.clone();
    frame_size = cap_frame.size();
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  return;
}

// callback for bounding box image

void YoloROSTracker::cameraCallback2(const sensor_msgs::ImageConstPtr& msg)
  {
  ROS_DEBUG("[YoloROSDetector] ROS box image received.");

  cv_bridge::CvImagePtr cv_ptr2;

  try {
    ROS_INFO("Callback bbox Called");
    cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    marker1_right = cv_ptr2->image.clone(); 
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  return;
}

// callback for bounding box image

void YoloROSTracker::cameraCallback3(const sensor_msgs::ImageConstPtr& msg)
  {
  ROS_DEBUG("[YoloROSDetector] ROS box image received.");

  cv_bridge::CvImagePtr cv_ptr3;

  try {
    ROS_INFO("Callback bbox Called");
    cv_ptr3 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    marker2_right = cv_ptr3->image.clone();
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  return;
}


 // callback for bounding box coordinates for marker1
 // extract x and y coordinates from the right camera

 void YoloROSTracker::bboxCoordCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
  {
   int i = 0;
   for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
     {
       //bboxCoordinates.push_back(*it);
	if(i%2 == 0){
	  xmin_right_1 = *it;
	}
	else{
	  ymin_right_1 = *it;
	}
       i++;
     }
   return;
 }

// callback for bounding box coordinates for marker2

void YoloROSTracker::bboxCoordCallback2(const std_msgs::Float32MultiArray::ConstPtr& array)
  {
   int i = 0;
   for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
     {
        if(i%2 == 0){
          xmin_right_2 = *it;
        }
        else{
          ymin_right_2 = *it;
        }
       i++;
     }
   return;
 }


// used for stereo vision measurements

double YoloROSTracker::findMedian(double a[], int n){

  std::sort(a, a + n);

  if(n % 2 != 0){
    return (double)a[n/2];
  }
  return (double)(a[(n - 1)/2] + a[n / 2]) / 2.0;

}



void YoloROSTracker::captureThread(){
  t_cap = std::thread([&]() { ros::spinOnce(); });
  ++cur_time_extrapolate;
  if(t_cap.joinable()){
    t_cap.join();
    ++fps_cap_counter;
    cur_frame = cap_frame.clone();
  }
}
void YoloROSTracker::trackThread(){
  if(consumed){
    ROS_INFO("Inside tracking");
      std::unique_lock<std::mutex> lock(mtx);
      det_image = detector->mat_to_image_resize(cur_frame);
      auto old_result_vec = detector->tracking_id(result_vec);
      auto detected_result_vec = thread_result_vec;
      result_vec = detected_result_vec;
 #ifdef TRACK_OPTFLOW
      if(track_optflow_queue.size() > 0){
        cv::Mat first_frame = track_optflow_queue.front();
        tracker_flow.update_tracking_flow(track_optflow_queue.front(), result_vec);
        while (track_optflow_queue.size() > 1) {
          track_optflow_queue.pop();
          result_vec = tracker_flow.tracking_flow(track_optflow_queue.front(), true);
        }
      track_optflow_queue.pop();
      passed_flow_frames = 0;

      result_vec = detector->tracking_id(result_vec);
      auto tmp_result_vec = detector->tracking_id(detected_result_vec, false);
      extrapolate_coords.new_result(tmp_result_vec, old_time_extrapolate);
      old_time_extrapolate = cur_time_extrapolate;
      }
 #else
      result_vec = detector->tracking_id(result_vec);
      extrapolate_coords.new_result(result_vec, cur_time_extrapolate - 1);
 #endif
    for (auto &i : old_result_vec) {
      auto it = std::find_if(result_vec.begin(), result_vec.end(),
                [&i](bbox_t const& b) { return b.track_id == i.track_id && b.obj_id == i.obj_id; });
      bool track_id_absent = (it == result_vec.end());
      if (track_id_absent)
        if (i.frames_counter-- > 1)
            result_vec.push_back(i);
        else it->frames_counter = std::min((unsigned)3, i.frames_counter + 1);
    }

 #ifdef TRACK_OPTFLOW
    tracker_flow.update_cur_bbox_vec(result_vec);
    result_vec = tracker_flow.tracking_flow(cur_frame, true);
 #endif
    consumed = false;
    ROS_INFO("Consumder Done");
    cv_pre_tracked.notify_all();
  }
}
void YoloROSTracker::publishResult(){
  if (!cur_frame.empty()) {
    steady_end = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(steady_end - steady_start).count() >= 1) {
        current_det_fps = fps_det_counter;
        current_cap_fps = fps_cap_counter;
        steady_start = steady_end;
        fps_det_counter = 0;
        fps_cap_counter = 0;
      }
 #ifdef TRACK_OPTFLOW
    ++passed_flow_frames;
    track_optflow_queue.push(cur_frame.clone());
    result_vec = tracker_flow.tracking_flow(cur_frame);
    extrapolate_coords.update_result(result_vec, cur_time_extrapolate, true);
 #endif

    // define compression parameters for bboxes to be saved
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(3);

    auto result_vec_draw = result_vec;
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };
    for(auto &i : result_vec){
      boundingBox.Class = classLabels[i.obj_id];
      boundingBox.prob = i.prob;
      boundingBox.x = i.x;
      boundingBox.y = i.y;
      boundingBox.w = i.w;
      boundingBox.h = i.h;
      boundingBoxes.bounding_boxes.push_back(boundingBox);
      //cv::Scalar color = obj_id_to_color(i.obj_id);
      //cv::rectangle(cur_frame, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
      if (classLabels.size() > i.obj_id) {
	    
	    /*
	    // expand window to ensure that the whole marker is included
            cv::Rect ROI1(i.x*0.91, i.y*0.91, i.w*2.2, i.h*2.1);
	    cv::Rect ROI2(i.x*0.93, i.y*0.91, i.w*2.2, i.h*2.1);

	    
            if(classLabels[i.obj_id] == "m1"){
		marker1_left = cur_frame(ROI1);
		xmin_left_1 = i.x*0.91;
                ymin_left_1 = i.y*0.91;
		//OBS: consider to publish and subscribe for left bounding box as well
                cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/bbox-left_1.png", marker1_left, compression_params);
   
            }

            if(classLabels[i.obj_id] == "m2"){
		marker2_left = cur_frame(ROI2);
		xmin_left_2 = i.x*0.93;
                ymin_left_2 = i.y*0.91;
                //OBS: consider to publish and subscribe for left bounding box as well
                cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/bbox-left_2.png", marker2_left, compression_params);

            }
	    */
	
	          cv::Rect ROI1(i.x*(1 - i.w*0.001), i.y*(1 - i.h*0.001), i.w*2.5, i.h*2.3);
            cv::Rect ROI2(i.x*(1 - i.w*0.001), i.y*(1 - i.h*0.001), i.w*2.5, i.h*2.3);

            
            if(classLabels[i.obj_id] == "m1"){
                marker1_left = cur_frame(ROI1);
                xmin_left_1 = i.x*(1 - i.w*0.001);
                ymin_left_1 = i.y*(1 - i.h*0.001);
                //OBS: consider to publish and subscribe for left bounding box as well
                cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/bbox-left_1.png", marker1_left, compression_params);
   
            }

            if(classLabels[i.obj_id] == "m2"){
                marker2_left = cur_frame(ROI2);
                xmin_left_2 = i.x*(1 - i.w*0.001);
                ymin_left_2 = i.y*(1 - i.h*0.001);
                //OBS: consider to publish and subscribe for left bounding box as well
                cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/bbox-left_2.png", marker2_left, compression_params);

            }
 
	    

        }
    }

    
 
    // %%% Decide if pose estimation should be performed for right/left camera %%%
    

   
    // right camera
    if(marker1_right.empty() && marker2_right.empty()){
      do_pose_estimate_right = false;
    }
    else{
        if(!marker1_right.empty()){
	  bbox_right_1 = marker1_right;	
          marker1_right.release();
        }
        if(!marker2_right.empty()){
          bbox_right_2 = marker2_right;
          marker2_right.release();
        }
      do_pose_estimate_right = true;
    }

    

    // left camera
    if(marker1_left.empty() && marker2_left.empty()){
      //ROS_INFO("empty left bbox");
      do_pose_estimate_left = false; 
    }
    else{
	if(!marker1_left.empty()){
	  bbox_left_1 = cv::imread("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/bbox-left_1.png", 0);
	}
	if(!marker2_left.empty()){
	  bbox_left_2 = cv::imread("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/bbox-left_2.png", 0);
	}
      do_pose_estimate_left = true;   
    }


    // %%% STEREO VISION CODE %%%

    if(do_pose_estimate_left && do_pose_estimate_right && do_stereo){
            
      // only do detection when the same marker is visible from left and right camera

      // %% part1: aruco marker detection
      if(!bbox_left_1.empty() && !bbox_right_1.empty()){
        cv::aruco::detectMarkers(bbox_left_1, dictionary, markerCorners_l_1, markerIds_l_1, parameters, rejectedCandidates_l_1);
	cv::aruco::detectMarkers(bbox_right_1, dictionary, markerCorners_r_1, markerIds_r_1, parameters, rejectedCandidates_r_1);
      }
      if(!bbox_left_2.empty() && !bbox_right_2.empty()){
        cv::aruco::detectMarkers(bbox_left_2, dictionary, markerCorners_l_2, markerIds_l_2, parameters, rejectedCandidates_l_2);
	cv::aruco::detectMarkers(bbox_right_2, dictionary, markerCorners_r_2, markerIds_r_2, parameters, rejectedCandidates_r_2);
      }
  
      // Assume the corners of each marker comes in the same order (no need for matching strategies

      std::vector<cv::KeyPoint> keyPoints_left_1, keyPoints_right_1, keyPoints_left_2, keyPoints_right_2;

      if( (markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0) && (markerIds_r_1.size() > 0 && markerCorners_r_1.size() > 0) ){
	for(int i = 0; i < 4; i++){
          keyPoints_left_1.push_back(cv::KeyPoint(markerCorners_l_1[0][i], 1.f));
          keyPoints_right_1.push_back(cv::KeyPoint(markerCorners_r_1[0][i], 1.f));
        }
      }

      if( (markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0) && (markerIds_r_2.size() > 0 && markerCorners_r_2.size() > 0) ){
        for(int i = 0; i < 4; i++){
          keyPoints_left_2.push_back(cv::KeyPoint(markerCorners_l_2[0][i], 1.f));
          keyPoints_right_2.push_back(cv::KeyPoint(markerCorners_r_2[0][i], 1.f));
        }
      }

 		    
      // projection matrices -  computed from stereoRectify() with camera calibration parameters (stereo calibrator) - same as used for monocular	

      //cv::Mat P1(3,4, CV_64F, double(0));
      //P1.at<double>(0,0) = 382.935;
      //P1.at<double>(0,2) = 323.684;
      //P1.at<double>(1,1) = 382.935;
      //P1.at<double>(1,2) = 260.191;
      //P1.at<double>(2,2) = 1;

      //cv::Mat P2(3,4, CV_64F, double(0));
      //P2.at<double>(0,0) = 382.935;
      //P2.at<double>(0,2) = 323.684;
      //P2.at<double>(0,3) = -199574; 
      //P2.at<double>(1,1) = 382.935;
      //P2.at<double>(1,2) = 260.191;
      //P2.at<double>(2,2) = 1;

      /*
      cv::Mat P1(3,4, CV_64F, double(0));
      P1.at<double>(0,0) = 798.775;
      P1.at<double>(0,2) = 670.609;
      P1.at<double>(1,1) = 798.775;
      P1.at<double>(1,2) = 525.289;
      P1.at<double>(2,2) = 1;

      cv::Mat P2(3,4, CV_64F, double(0));
      P2.at<double>(0,0) = 798.775;
      P2.at<double>(0,2) = 670.609;
      P2.at<double>(0,3) = -495032;
      P2.at<double>(1,1) = 798.775;
      P2.at<double>(1,2) = 525.289;
      P2.at<double>(2,2) = 1;
	*/

      cv::Mat P1(3,4, CV_64F, double(0));
      P1.at<double>(0,0) = 774.51; //765.87
      P1.at<double>(0,2) = 647.368;
      P1.at<double>(1,1) = 774.51; //765.87
      P1.at<double>(1,2) = 520.382;
      P1.at<double>(2,2) = 1;

      cv::Mat P2(3,4, CV_64F, double(0));
      P2.at<double>(0,0) = 774.51; //
      P2.at<double>(0,2) = 647.368;
      P2.at<double>(0,3) = -474608.643369; //-474608.643369
      P2.at<double>(1,1) = 774.51; //
      P2.at<double>(1,2) = 520.382;
      P2.at<double>(2,2) = 1;



      // %%% Transform points relative to the whole image and triangulate %%%
      
      cv::Mat pointsMat_l_1(2, 4, CV_64F);
      cv::Mat pointsMat_r_1(2, 4, CV_64F);
      cv::Mat pointsMat_l_2(2, 4, CV_64F);
      cv::Mat pointsMat_r_2(2, 4, CV_64F);

      CvPoint3D64f point3D_1, point3D2_1, point3D3_1, point3D4_1; //marker1
      CvPoint3D64f point3D_2, point3D2_2, point3D3_2, point3D4_2; //marker2

      if( (markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0) && (markerIds_r_1.size() > 0 && markerCorners_r_1.size() > 0) ){
        
	for(int i = 0; i < 4; i++){
          pointsMat_l_1.at<double>(0,i) = keyPoints_left_1[i].pt.x + xmin_left_1;
          pointsMat_l_1.at<double>(1,i) = keyPoints_left_1[i].pt.y + ymin_left_1;
          pointsMat_r_1.at<double>(0,i) = keyPoints_right_1[i].pt.x + xmin_right_1;
          pointsMat_r_1.at<double>(1,i) = keyPoints_right_1[i].pt.y + ymin_right_1;
	}

	// triangulate the four pair of points - marker1
        cv::Mat pnts3D_1(4, 4, CV_64F); //output 3D points from marker1
        cv::triangulatePoints(P1, P2, pointsMat_l_1, pointsMat_r_1, pnts3D_1);

	// calculate 3D world points relative to left camera
        //CvPoint3D64f point3D_1, point3D2_1, point3D3_1, point3D4_1; //marker1

	//marker1
        point3D_1.x = pnts3D_1.at<double>(0, 0)/pnts3D_1.at<double>(3, 0);
        point3D_1.y = pnts3D_1.at<double>(1, 0)/pnts3D_1.at<double>(3, 0);
        point3D_1.z = pnts3D_1.at<double>(2, 0)/pnts3D_1.at<double>(3, 0);
        point3D2_1.x = pnts3D_1.at<double>(0, 1)/pnts3D_1.at<double>(3, 1);
        point3D2_1.y = pnts3D_1.at<double>(1, 1)/pnts3D_1.at<double>(3, 1);
        point3D2_1.z = pnts3D_1.at<double>(2, 1)/pnts3D_1.at<double>(3, 1);
        point3D3_1.x = pnts3D_1.at<double>(0, 2)/pnts3D_1.at<double>(3, 2);
        point3D3_1.y = pnts3D_1.at<double>(1, 2)/pnts3D_1.at<double>(3, 2);
        point3D3_1.z = pnts3D_1.at<double>(2, 2)/pnts3D_1.at<double>(3, 2);
        point3D4_1.x = pnts3D_1.at<double>(0, 3)/pnts3D_1.at<double>(3, 3);
        point3D4_1.y = pnts3D_1.at<double>(1, 3)/pnts3D_1.at<double>(3, 3);
        point3D4_1.z = pnts3D_1.at<double>(2, 3)/pnts3D_1.at<double>(3, 3);
      }



      if( (markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0) && (markerIds_r_2.size() > 0 && markerCorners_r_2.size() > 0) ){
        
	for(int i = 0; i < 4; i++){
          pointsMat_l_2.at<double>(0,i) = keyPoints_left_2[i].pt.x + xmin_left_2;
          pointsMat_l_2.at<double>(1,i) = keyPoints_left_2[i].pt.y + ymin_left_2;
          pointsMat_r_2.at<double>(0,i) = keyPoints_right_2[i].pt.x + xmin_right_2;
          pointsMat_r_2.at<double>(1,i) = keyPoints_right_2[i].pt.y + ymin_right_2;
        }

	cv::Mat pnts3D_2(4, 4, CV_64F); //output 3D points from marker2
        cv::triangulatePoints(P1, P2, pointsMat_l_2, pointsMat_r_2, pnts3D_2);

	// calculate 3D world points relative to left camera - marker2
	//CvPoint3D64f point3D_2, point3D2_2, point3D3_2, point3D4_2; //marker2

	//marker2
        point3D_2.x = pnts3D_2.at<double>(0, 0)/pnts3D_2.at<double>(3, 0);
        point3D_2.y = pnts3D_2.at<double>(1, 0)/pnts3D_2.at<double>(3, 0);
        point3D_2.z = pnts3D_2.at<double>(2, 0)/pnts3D_2.at<double>(3, 0);
        point3D2_2.x = pnts3D_2.at<double>(0, 1)/pnts3D_2.at<double>(3, 1);
        point3D2_2.y = pnts3D_2.at<double>(1, 1)/pnts3D_2.at<double>(3, 1);
        point3D2_2.z = pnts3D_2.at<double>(2, 1)/pnts3D_2.at<double>(3, 1);
        point3D3_2.x = pnts3D_2.at<double>(0, 2)/pnts3D_2.at<double>(3, 2);
        point3D3_2.y = pnts3D_2.at<double>(1, 2)/pnts3D_2.at<double>(3, 2);
        point3D3_2.z = pnts3D_2.at<double>(2, 2)/pnts3D_2.at<double>(3, 2);
        point3D4_2.x = pnts3D_2.at<double>(0, 3)/pnts3D_2.at<double>(3, 3);
        point3D4_2.y = pnts3D_2.at<double>(1, 3)/pnts3D_2.at<double>(3, 3);
        point3D4_2.z = pnts3D_2.at<double>(2, 3)/pnts3D_2.at<double>(3, 3);
      }


      // transform points to middle point of marker and find median - more robust for outliers
      
      // TODO: more correct transformation with rodrigues - will not work under waves outside? 
      
      // marker1
      if( (markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0) && (markerIds_r_1.size() > 0 && markerCorners_r_1.size() > 0) ){
      	
	/*	
	std::cout << std::endl;
      	std::cout << "%%% 3D world point middle of MARKER1 relative to left camera %%%" << std::endl;
      	std::cout << "mp x1: " << point3D_1.x + 131.4 << " mp y1: " << point3D_1.y + 131.4 << " mp z1: " << point3D_1.z << std::endl;
      	std::cout << "mp x2: " << point3D2_1.x - 131.4 << " mp y2: " << point3D2_1.y + 131.4 << " mp z2: " << point3D2_1.z << std::endl;
      	std::cout << "mp x3: " << point3D3_1.x - 131.4 << " mp y3: " << point3D3_1.y - 131.4 << " mp z3: " << point3D3_1.z << std::endl;
      	std::cout << "mp x4: " << point3D4_1.x + 131.4 << " mp y4: " << point3D4_1.y - 131.4 << " mp z4: " << point3D4_1.z << std::endl;
	std::cout << std::endl;
	*/

	// median value for marker1
        double x_med_1, y_med_1, z_med_1;
        double x_1[] = { point3D_1.x + 131.4, point3D2_1.x - 131.4, point3D3_1.x - 131.4, point3D4_1.x + 131.4};
        double y_1[] =  { point3D_1.y + 131.4, point3D2_1.y + 131.4, point3D3_1.y - 131.4, point3D4_1.y - 131.4 };
        double z_1[] = { point3D_1.z, point3D2_1.z, point3D3_1.z, point3D4_1.z };
        int n_1 = sizeof(x_1) / sizeof(x_1[0]);
        x_med_1 = findMedian(x_1,n_1);
        y_med_1 = findMedian(y_1,n_1);
        z_med_1 = findMedian(z_1,n_1);
        //std::cout << "x_median m1: " << x_med_1 << " y median m1: " << y_med_1 << " z median m1: " << z_med_1 << std::endl;
	

	//log file
	time_stamp_end = std::chrono::steady_clock::now();
	
	std::tm *tm_gmt = std::gmtime(&curr_time);

	//format: utc time - relative time - radius/length - length_x_z - x_med - y_med - z_med - x_1 - y_1 - z_1         measured in seconds and mm

	log_stereo_m1 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(x_med_1*0.001,2) + pow(y_med_1*0.001,2) + pow(z_med_1*0.001,2) ) << " " << sqrt( pow(x_med_1*0.001,2) + pow(z_med_1*0.001,2) ) << " " << x_med_1*0.001 << " " << y_med_1*0.001 << " " << z_med_1*0.001 << std::endl;
        log_all << "s - left - m1, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(x_med_1*0.001,2) + pow(y_med_1*0.001,2) + pow(z_med_1*0.001,2) ) << " length_x_z: " << sqrt( pow(x_med_1*0.001,2) + pow(z_med_1*0.001,2) ) <<  " x: "  << x_med_1*0.001 << " y: " << y_med_1*0.001 << " z: " << z_med_1*0.001 << std::endl;
     
  
	//ss <<  "stereo - left - m1, utc start time: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(x_med_1*0.001,2) + pow(y_med_1*0.001,2) + pow(z_med_1*0.001,2) ) << " x: "  << x_med_1*0.001 << " y: " << y_med_1*0.001 << " z: " << z_med_1*0.001 << std::endl;

	//msg.data = ss.str();
	//cameraMeasurementPublisher.publish(msg);

	//delete measurements - ensures that old measurements will not be printed again
        // Not neccessary - will be deleted when out of scope
      }
	//marker2
      if( (markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0) && (markerIds_r_2.size() > 0 && markerCorners_r_2.size() > 0) ){

	/*
	std::cout << std::endl;
	std::cout << "%%% 3D world point middle of MARKER2 relative to left camera %%%" << std::endl;
        std::cout << "mp x1: " << point3D_2.x + 131.4 << " mp y1: " << point3D_2.y + 131.4 << " mp z1: " << point3D_2.z << std::endl;
        std::cout << "mp x2: " << point3D2_2.x - 131.4 << " mp y2: " << point3D2_2.y + 131.4 << " mp z2: " << point3D2_2.z << std::endl;
        std::cout << "mp x3: " << point3D3_2.x - 131.4 << " mp y3: " << point3D3_2.y - 131.4 << " mp z3: " << point3D3_2.z << std::endl;
        std::cout << "mp x4: " << point3D4_2.x + 131.4 << " mp y4: " << point3D4_2.y - 131.4 << " mp z4: " << point3D4_2.z << std::endl;
	std::cout << std::endl;
	*/

	// median value for marker2
        double x_med_2, y_med_2, z_med_2;
        double x_2[] = { point3D_2.x + 131.4, point3D2_2.x - 131.4, point3D3_2.x - 131.4, point3D4_2.x + 131.4 };
        double y_2[] =  { point3D_2.y + 131.4, point3D2_2.y + 131.4, point3D3_2.y - 131.4, point3D4_2.y - 131.4 };
        double z_2[] = { point3D_2.z, point3D2_2.z, point3D3_2.z, point3D4_2.z };
        int n_2 = sizeof(x_2) / sizeof(x_2[0]);
        x_med_2 = findMedian(x_2,n_2);
        y_med_2 = findMedian(y_2,n_2);
        z_med_2 = findMedian(z_2,n_2);
        //std::cout << "x_median m1: " << x_med_2 << " y median m1: " << y_med_2 << " z median m1: " << z_med_2 << std::endl;

        //log file
	
	//publish cam measurement to ros
	std_msgs::String msg;
	std::stringstream ss;

        time_stamp_end = std::chrono::steady_clock::now();
	std::tm *tm_gmt = std::gmtime(&curr_time);

	log_stereo_m2 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(x_med_2*0.001,2) + pow(y_med_2*0.001,2) + pow(z_med_2*0.001,2) ) << " " << sqrt( pow(x_med_2*0.001,2) + pow(z_med_2*0.001,2) )  << " " << x_med_2*0.001  << " " << y_med_2*0.001  << " " << z_med_2*0.001  << " " << std::endl;
	log_all << "s - left - m2, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(x_med_2*0.001,2) + pow(y_med_2*0.001,2) + pow(z_med_2*0.001,2) ) << " length_x_z: " << sqrt( pow(x_med_2*0.001,2) + pow(z_med_2*0.001,2) ) << " x: "  << x_med_2*0.001 << " y: " << y_med_2*0.001 << " z: " << z_med_2*0.001 << std::endl;

	ss << "s - l - m2, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " l: " << sqrt( pow(x_med_2*0.001,2) + pow(y_med_2*0.001,2) + pow(z_med_2*0.001,2) ) << " l_x_z: " << sqrt( pow(x_med_2*0.001,2) + pow(z_med_2*0.001,2) )  << " x: "  << x_med_2*0.001 << " y: " << y_med_2*0.001 << " z: " << z_med_2*0.001;
	//ss << '\n';
	msg.data = ss.str();
        cameraMeasurementPublisher.publish(msg);

	//delete measurements - ensures that old measurements will not be printed again
	// not neccessary - will be deleted when out of scope
      }
      

    }
	


    // %%% MONOCULAR VISION CODE %%%


    if((do_pose_estimate_left || do_pose_estimate_right) && do_mono){


      // %% part1 - load calibration parameters %% - try to remove this away from loop

 	/*	
      cv::Mat K1(3,3, CV_32F); //cv::DataType<float>::type
      K1.at<float>(0,0) = 381.0888;
      K1.at<float>(0,2) = 324.4502;
      K1.at<float>(1,1) = 381.1861;
      K1.at<float>(1,2) = 258.2286;
      K1.at<float>(2,2) = 1;

      cv::Mat D1(1,5, CV_32F);
      D1.at<float>(0,0) = -0.0641; // k1 - radial
      D1.at<float>(0,1) = 0.1569;// k2
      D1.at<float>(0,2) = 0;// p1 - tangential
      D1.at<float>(0,3) = -0.002; // p2
      D1.at<float>(0,4) = -0.1106; // k3
	

      cv::Mat K2(3,3, CV_32F);
      K2.at<float>(0,0) = 379.5845;
      K2.at<float>(0,2) = 325.2762;
      K2.at<float>(1,1) = 380.1453;
      K2.at<float>(1,2) = 262.6230;
      K2.at<float>(2,2) = 1;

      cv::Mat D2(1,5, CV_32F);
      D2.at<float>(0,0) = -0.0701; // k1 
      D2.at<float>(0,1) = 0.1749; // k2
      D2.at<float>(0,2) = -0.0022; // p1
      D2.at<float>(0,3) = -0.0018; // p2
      D2.at<float>(0,4) = -0.1071; // k3
	*/

      cv::Mat K1(3,3, CV_32F); //cv::DataType<float>::type
      K1.at<float>(0,0) = 759.9205;
      K1.at<float>(0,2) = 659.1680;
      K1.at<float>(1,1) = 761.0753;
      K1.at<float>(1,2) = 517.9815;
      K1.at<float>(2,2) = 1;

      cv::Mat D1(1,5, CV_32F);
      D1.at<float>(0,0) = -0.0578; // k1 - radial
      D1.at<float>(0,1) = 0.1415;// k2
      D1.at<float>(0,2) = 0;// p1 - tangential
      D1.at<float>(0,3) = 0.0012; // p2
      D1.at<float>(0,4) = -0.0854; // k3


      cv::Mat K2(3,3, CV_32F);
      K2.at<float>(0,0) = 758.7863;
      K2.at<float>(0,2) = 658.4538;
      K2.at<float>(1,1) = 759.6578;
      K2.at<float>(1,2) = 526.6109;
      K2.at<float>(2,2) = 1;

      cv::Mat D2(1,5, CV_32F);
      D2.at<float>(0,0) = -0.0655; // k1 
      D2.at<float>(0,1) = 0.1647; // k2
      D2.at<float>(0,2) = 0; // p1
      D2.at<float>(0,3) = 0; // p2
      D2.at<float>(0,4) = -0.1095; // k3


      
      std::vector <cv::Point2f> bbox_left, bbox_right; // move this outside? 

      std::vector < cv::Vec3d > rvecs_l_1, tvecs_l_1, rvecs_l_2, tvecs_l_2, rvecs_r_1, tvecs_r_1, rvecs_r_2, tvecs_r_2;
      std::vector < cv::Vec3d > rvecs_l, tvecs_l, rvecs_r, tvecs_r; // remove later
      	
      // %% part2 - marker detection of right and/or left bbox + pose estimation
      
      // weakness: pose estimation of right and left bbox is not performed excactly at the same time. 
      // But dont need to launch the two scripts at the same time. 
      // Another approach is to simply publish pose estimation results from right camera in the other node  
      
      if(do_pose_estimate_left){
	
	// marker detection 
	if(!bbox_left_1.empty()){
      	cv::aruco::detectMarkers(bbox_left_1, dictionary, markerCorners_l_1, markerIds_l_1, parameters, rejectedCandidates_l_1);
	}
	if(!bbox_left_2.empty()){
        cv::aruco::detectMarkers(bbox_left_2, dictionary, markerCorners_l_2, markerIds_l_2, parameters, rejectedCandidates_l_2);
        }

	
	// find corners relative to full image
	std::vector <cv::Point2f> bbox_1_left, bbox_2_left;
	std::vector < std::vector<cv::Point2f> > markerCorners_l_full_1, markerCorners_l_full_2;
        markerCorners_l_full_1.push_back(std::vector<cv::Point2f>());
	markerCorners_l_full_2.push_back(std::vector<cv::Point2f>());

	if(markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0){
	  bbox_1_left.push_back(cv::Point2f(xmin_left_1, ymin_left_1));
          for(int j = 0; j < 4; j++){
            markerCorners_l_full_1[0].push_back(cv::Point2f(markerCorners_l_1[0][j].x + bbox_1_left[0].x, markerCorners_l_1[0][j].y + bbox_1_left[0].y)); 
          }
        }

	if(markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0){
	  bbox_2_left.push_back(cv::Point2f(xmin_left_2, ymin_left_2));
          for(int j = 0; j < 4; j++){
            markerCorners_l_full_2[0].push_back(cv::Point2f(markerCorners_l_2[0][j].x + bbox_2_left[0].x, markerCorners_l_2[0][j].y + bbox_2_left[0].y));
          }
        }


	// pose estimation
	
	if(markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0){
	  cv::aruco::estimatePoseSingleMarkers(markerCorners_l_full_1, 0.2628, K1, D1, rvecs_l_1, tvecs_l_1);
	}

	if(markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0){
          cv::aruco::estimatePoseSingleMarkers(markerCorners_l_full_2, 0.2628, K1, D1, rvecs_l_2, tvecs_l_2);
        }

	
      }

      if(do_pose_estimate_right){

	// marker detection
        if(!bbox_right_1.empty()){
	  ROS_INFO("test 1");
          cv::aruco::detectMarkers(bbox_right_1, dictionary, markerCorners_r_1, markerIds_r_1, parameters, rejectedCandidates_r_1);
        }
        if(!bbox_right_2.empty()){
	  ROS_INFO("test 2");
          cv::aruco::detectMarkers(bbox_right_2, dictionary, markerCorners_r_2, markerIds_r_2, parameters, rejectedCandidates_r_2);
        }


	// find corners relative to full image
        std::vector <cv::Point2f> bbox_1_right, bbox_2_right;
        std::vector < std::vector<cv::Point2f> > markerCorners_r_full_1, markerCorners_r_full_2;
        markerCorners_r_full_1.push_back(std::vector<cv::Point2f>());
        markerCorners_r_full_2.push_back(std::vector<cv::Point2f>());

        if(markerIds_r_1.size() > 0 && markerCorners_r_1.size() > 0){
          bbox_1_right.push_back(cv::Point2f(xmin_right_1, ymin_right_1));
          for(int j = 0; j < 4; j++){
            markerCorners_r_full_1[0].push_back(cv::Point2f(markerCorners_r_1[0][j].x + bbox_1_right[0].x, markerCorners_r_1[0][j].y + bbox_1_right[0].y));
          }
        }

        if(markerIds_r_2.size() > 0 && markerCorners_r_2.size() > 0){
          bbox_2_right.push_back(cv::Point2f(xmin_right_2, ymin_right_2));
          for(int j = 0; j < 4; j++){
            markerCorners_r_full_2[0].push_back(cv::Point2f(markerCorners_r_2[0][j].x + bbox_2_right[0].x, markerCorners_r_2[0][j].y + bbox_2_right[0].y));
          }
        }


	// pose estimation

        if(markerIds_r_1.size() > 0 && markerCorners_r_1.size() > 0){
	  //ROS_INFO("test 3");
          cv::aruco::estimatePoseSingleMarkers(markerCorners_r_full_1, 0.2628, K1, D1, rvecs_r_1, tvecs_r_1);
        }

        if(markerIds_r_2.size() > 0 && markerCorners_r_2.size() > 0){
	  //ROS_INFO("test 4");
          cv::aruco::estimatePoseSingleMarkers(markerCorners_r_full_2, 0.2628, K1, D1, rvecs_r_2, tvecs_r_2);
        }



      }

      // %% part3: log measurements to file - one of the following cases will occur %%

      // case1: pose estimate from both cameras

      if(do_pose_estimate_left && do_pose_estimate_right){
        // log pose estimate and time stamp from both cameras
	
	if(markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0){
          for(int i = 0; i < tvecs_l_1.size(); i++){
            time_stamp_end = std::chrono::steady_clock::now(); // relative time
	    std::tm *tm_gmt = std::gmtime(&curr_time); //utc time
	    log_mono_left_m1 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][1],2) + pow(tvecs_l_1[i][2],2) ) << " " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][2],2) ) << " " <<  tvecs_l_1[i][0] << " " << tvecs_l_1[i][1] << " " << tvecs_l_1[i][2] << " " << rvecs_l_1[i][0] << " " << rvecs_l_1[i][1] << " " << rvecs_l_1[i][2] << std::endl;
            log_all << "m - left - m1, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][1],2) + pow(tvecs_l_1[i][2],2) ) << " length_x_z: " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][2],2) )  << " x: " <<  tvecs_l_1[i][0] << " y: " << tvecs_l_1[i][1] << " z: " << tvecs_l_1[i][2] << " yaw: " << rvecs_l_1[i][0] << " pitch: " << rvecs_l_1[i][1] << " roll: " << rvecs_l_1[i][2] << std::endl;
          }
        }

        if(markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0){
          for(int i = 0; i < tvecs_l_2.size(); i++){
            time_stamp_end = std::chrono::steady_clock::now(); // relative time
	    std::tm *tm_gmt = std::gmtime(&curr_time); //utc time
    	    log_mono_left_m2 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][1],2) + pow(tvecs_l_2[i][2],2) ) << " " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][2],2) ) << " " << tvecs_l_2[i][0] << " " << tvecs_l_2[i][1] << " " << tvecs_l_2[i][2] << " " << rvecs_l_2[i][0] << " " << rvecs_l_2[i][1] << " " << rvecs_l_2[i][2] << std::endl;
            log_all << "m - left - m2, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][1],2) + pow(tvecs_l_2[i][2],2) ) << " length_x_z: " << sqrt( pow(tvecs_l_2[i][0],2) +  pow(tvecs_l_2[i][2],2) ) << " x: " <<  tvecs_l_2[i][0] << " y: " << tvecs_l_2[i][1] << " z: " << tvecs_l_2[i][2] << " yaw: " << rvecs_l_2[i][0] << " pitch: " << rvecs_l_2[i][1] << " roll: " << rvecs_l_2[i][2] << std::endl;
          }
        }

	if(markerIds_r_1.size() > 0 && markerCorners_r_1.size() > 0){
          for(int i = 0; i < tvecs_r_1.size(); i++){
            time_stamp_end = std::chrono::steady_clock::now(); // relative time
	    std::tm *tm_gmt = std::gmtime(&curr_time); //utc time
	    log_mono_right_m1 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(tvecs_r_1[i][0],2) + pow(tvecs_r_1[i][1],2) + pow(tvecs_r_1[i][2],2) ) << " " << sqrt( pow(tvecs_r_1[i][0],2) + pow(tvecs_r_1[i][2],2) ) << " " << tvecs_r_1[i][0] << " " << tvecs_r_1[i][1] << " " << tvecs_r_1[i][2] << " " << rvecs_r_1[i][0] << " " << rvecs_r_1[i][1] << " " << rvecs_r_1[i][2] << std::endl;
            log_all << "m - right - m1, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(tvecs_r_1[i][0],2) + pow(tvecs_r_1[i][1],2) + pow(tvecs_r_1[i][2],2) ) << " length_x_z: " <<  sqrt( pow(tvecs_r_1[i][0],2) + pow(tvecs_r_1[i][2],2) ) << " x: " <<  tvecs_r_1[i][0] << " y: " << tvecs_r_1[i][1] << " z: " << tvecs_r_1[i][2] << " yaw: " << rvecs_r_1[i][0] << " pitch: " << rvecs_r_1[i][1] << " roll: " << rvecs_r_1[i][2] << std::endl;
          }
        }

        if(markerIds_r_2.size() > 0 && markerCorners_r_2.size() > 0){
          for(int i = 0; i < tvecs_r_2.size(); i++){
            time_stamp_end = std::chrono::steady_clock::now(); // relative time
	    std::tm *tm_gmt = std::gmtime(&curr_time); //utc time
	    log_mono_right_m2 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(tvecs_r_2[i][0],2) + pow(tvecs_r_2[i][1],2) + pow(tvecs_r_2[i][2],2) ) << " " << sqrt( pow(tvecs_r_2[i][0],2) + pow(tvecs_r_2[i][2],2) ) << " " << tvecs_r_2[i][0] << " " << tvecs_r_2[i][1] << " " << tvecs_r_2[i][2] << " " << rvecs_r_2[i][0] << " " << rvecs_r_2[i][1] << " " << rvecs_r_2[i][2] << std::endl;
            log_all << "m - right - m2, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(tvecs_r_2[i][0],2) + pow(tvecs_r_2[i][1],2) + pow(tvecs_r_2[i][2],2) ) << " length_x_z: " << sqrt( pow(tvecs_r_2[i][0],2) + pow(tvecs_r_2[i][2],2) ) << " x: " <<  tvecs_r_2[i][0] << " y: " << tvecs_r_2[i][1] << " z: " << tvecs_r_2[i][2] << " yaw: " << rvecs_r_2[i][0] << " pitch: " << rvecs_r_2[i][1] << " roll: " << rvecs_r_2[i][2] << std::endl;
          }
        }
        log_all << std::endl;
	

      }

     

      // case2: only left and not right camera
      if(do_pose_estimate_left && !(do_pose_estimate_right)){

        if(markerIds_l_1.size() > 0 && markerCorners_l_1.size() > 0){
	  for(int i = 0; i < tvecs_l_1.size(); i++){
            time_stamp_end = std::chrono::steady_clock::now(); // relative time
	    std::tm *tm_gmt = std::gmtime(&curr_time); //utc time
            log_mono_left_m1 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][1],2) + pow(tvecs_l_1[i][2],2) ) << " " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][2],2) ) << " " <<  tvecs_l_1[i][0] << " " << tvecs_l_1[i][1] << " " << tvecs_l_1[i][2] << " " << rvecs_l_1[i][0] << " " << rvecs_l_1[i][1] << " " << rvecs_l_1[i][2] << std::endl;
            log_all << "m - left - m1, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][1],2) + pow(tvecs_l_1[i][2],2) ) << " length_x_z: " << sqrt( pow(tvecs_l_1[i][0],2) + pow(tvecs_l_1[i][2],2) ) << " x: " <<  tvecs_l_1[i][0] << " y: " << tvecs_l_1[i][1] << " z: " << tvecs_l_1[i][2] << " yaw: " << rvecs_l_1[i][0] << " pitch: " << rvecs_l_1[i][1] << " roll: " << rvecs_l_1[i][2] << std::endl;
          }
	}

	if(markerIds_l_2.size() > 0 && markerCorners_l_2.size() > 0){
	  for(int i = 0; i < tvecs_l_2.size(); i++){
            time_stamp_end = std::chrono::steady_clock::now(); // relative time
	    std::tm *tm_gmt = std::gmtime(&curr_time); // utc time
            log_mono_left_m2 << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][1],2) + pow(tvecs_l_2[i][2],2) ) << " " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][2],2) ) << " " << tvecs_l_2[i][0] << " " << tvecs_l_2[i][1] << " " << tvecs_l_2[i][2] << " " << rvecs_l_2[i][0] << " " << rvecs_l_2[i][1] << " " << rvecs_l_2[i][2] << std::endl;
            log_all << "m - left - m2, utc: " << tm_gmt->tm_hour << ":" << tm_gmt->tm_min << ":" << tm_gmt->tm_sec << " t: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " length: " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][1],2) + pow(tvecs_l_2[i][2],2) ) << " lengtt_x_z: " << sqrt( pow(tvecs_l_2[i][0],2) + pow(tvecs_l_2[i][2],2) ) << " x: " <<  tvecs_l_2[i][0] << " y: " << tvecs_l_2[i][1] << " z: " << tvecs_l_2[i][2] << " yaw: " << rvecs_l_2[i][0] << " pitch: " << rvecs_l_2[i][1] << " roll: " << rvecs_l_2[i][2] << std::endl;
          }
	}
	log_all << std::endl;
	
      }

      /*
      // case3: only right and not left camera - TODO: enable this case - by now, this never happens
      if(do_pose_estimate_right && !(do_pose_estimate_left)){
	for(int i = 0; i < tvecs_l.size(); i++){
	  time_stamp_end = std::chrono::steady_clock::now(); // measure time for each pose estimate
          log_left << "m: " << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " <<  tvecs_r[i][0] << " " << tvecs_r[i][1] << " " << tvecs_r[i][2] << " " << rvecs_r[i][0] << " " << rvecs_r[i][1] << " " << rvecs_r[i][2] << std::endl;
	}
	//log_left << std::endl;
      }
      */

      //release latest measurements - ensures that old measurements will not be printed again
      rvecs_l_1.clear();
      tvecs_l_1.clear();
      rvecs_l_2.clear();
      tvecs_l_2.clear();
      rvecs_r_1.clear();
      tvecs_r_1.clear();
      rvecs_r_2.clear();
      tvecs_r_2.clear();
 
	
    }  


    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps);
        putText(cur_frame, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }
    boundingBoxesPublisher.publish(boundingBoxes);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cur_frame).toImageMsg();
    detectionImagePublisher.publish(msg);
    ROS_INFO("Published");

    }
}

}
