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

// libraries for file streams
#include <iostream>
#include <fstream>


#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif


// %%% Parameters for pose estimation %%%


  // flag for activating pose estimation
bool do_pose_estimate = false; 

  // input/output image and resolution
cv::Mat marker1_right, marker2_right, bbox_right_1, bbox_right_2, inputImage_l, inputImage_r; //update
int imageWidth = 1280;
int imageHeight = 1024;

  // bbox coordinates copies
float xmin_left, ymin_left, xmin_right_1, ymin_right_1, xmin_right_2, ymin_right_2;
float xmin_right, ymin_right;

// publish bbox coordinates for each marker
std_msgs::Float32MultiArray bboxArray_1, bboxArray_2;

  // aruco marker detection parameters 
std::vector< int > markerIds_l, markerIds_r;
std::vector< std::vector<cv::Point2f> > markerCorners_l, rejectedCandidates_l, markerCorners_r, rejectedCandidates_r;
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
cv::Ptr<cv::aruco::Dictionary> dictionary_r = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);


//file to log pose estimate
const char *path="/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/log_right.txt";
std::ofstream log_right(path); //open in constructor


namespace darknet_ros {

YoloROSTracker::YoloROSTracker(ros::NodeHandle nh)
  : nodeHandle(nh),
    imageTransport(nodeHandle)
  {
    ROS_INFO("[YoloROSDetector] Node started.");
    time_stamp_start = std::chrono::steady_clock::now(); //define start time - measure for time stamp

    if(!readParameters()) ros::requestShutdown();
    initROS();
    initdarknet();
    ros::Rate rate(5);
    while(true){ 
      captureThread();
      trackThread();
      darknetThread();
      publishResult();
      if(!ros::ok()){
	log_right.close();
      	break;
      }
      rate.sleep();
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
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  std::string detectionBBoxTopicName1; //bbox
  std::string detectionBBoxTopicName2; //bbox
  std::string bboxCoordTopicName1; //bbox
  std::string bboxCoordTopicName2; //bbox
  int detectionImageQueueSize;
  bool detectionImageLatch;


  nodeHandle.param("subscribers/camera_reading/topic", cameraTopicName,
                  std::string("/camera/image_raw"));
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
  
  // bbox param
  nodeHandle.param("publishers/bbox_coord_1/topic", bboxCoordTopicName1, std::string("bbox_coordinates_1"));
  nodeHandle.param("publishers/bbox_coord_2/topic", bboxCoordTopicName2, std::string("bbox_coordinates_2"));

  // bbox param
  nodeHandle.param("publishers/bbox_image_1/topic", detectionBBoxTopicName1,
                    std::string("bbox_image_1"));
  nodeHandle.param("publishers/bbox_image_2/topic", detectionBBoxTopicName2,
                    std::string("bbox_image_2")); 

  nodeHandle.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle.param("publishers/detection_image/latch", detectionImageLatch, true);

  imageSubscriber = imageTransport.subscribe(cameraTopicName, cameraQueueSize,
                                             &YoloROSTracker::cameraCallback, this);

  boundingBoxesPublisher = nodeHandle.advertise<darknet_ros_msgs::BoundingBoxes>(
                                             boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
                                             detectionImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                             detectionImageQueueSize,
                                             detectionImageLatch);

  detectionImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(detectionImageTopicName,
                                              detectionImageQueueSize,
                                              detectionImageLatch);

  // bbox publisher
  detectionBBoxPublisher_1 = nodeHandle.advertise<sensor_msgs::Image>(detectionBBoxTopicName1,
                                              detectionImageQueueSize,
                                              detectionImageLatch);
  // bbox publisher
  detectionBBoxPublisher_2 = nodeHandle.advertise<sensor_msgs::Image>(detectionBBoxTopicName2,
                                              detectionImageQueueSize,
                                              detectionImageLatch);

  // bbox coord publisher
  bboxCoordPublisher_1 = nodeHandle.advertise<std_msgs::Float32MultiArray>(bboxCoordTopicName1, detectionImageQueueSize, detectionImageLatch);
  // bbox coord publisher
  bboxCoordPublisher_2 = nodeHandle.advertise<std_msgs::Float32MultiArray>(bboxCoordTopicName2, detectionImageQueueSize, detectionImageLatch);
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
    compression_params.push_back(3); //adjust this parameter?

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
            cv::Rect ROI1(i.x*0.89, i.y*0.91, i.w*2.2, i.h*2.1);
	    cv::Rect ROI2(i.x*0.91, i.y*0.91, i.w*2.2, i.h*2.1);

	    if(classLabels[i.obj_id] == "m1"){
                marker1_right = cur_frame(ROI1);
		//ROS_INFO("test 1");
                xmin_right_1 = i.x*0.89;
                ymin_right_1 = i.y*0.91;
		cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/marker1_right.png", marker1_right, compression_params);
                // send bbox image + coordinates over ROS - will be used for stereo
                sensor_msgs::ImagePtr msg_bbox_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", marker1_right).toImageMsg(); //mono?
                detectionBBoxPublisher_1.publish(msg_bbox_1);
		bboxArray_1.data.clear();
            	bboxArray_1.data.push_back(xmin_right_1);
            	bboxArray_1.data.push_back(ymin_right_1);
            	bboxCoordPublisher_1.publish(bboxArray_1);
            }
	    

            if(classLabels[i.obj_id] == "m2"){
                marker2_right = cur_frame(ROI2);
                //ROS_INFO("test2");
                xmin_right_2 = i.x*0.91;
                ymin_right_2 = i.y*0.91;
		cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/marker2_right.png", marker2_right, compression_params);
                sensor_msgs::ImagePtr msg_bbox_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", marker2_right).toImageMsg(); //mono?
                detectionBBoxPublisher_2.publish(msg_bbox_2);
		bboxArray_2.data.clear();
                bboxArray_2.data.push_back(xmin_right_2);
                bboxArray_2.data.push_back(ymin_right_2);
                bboxCoordPublisher_2.publish(bboxArray_2);
            }
	    */

	    cv::Rect ROI1(i.x*(1 - i.w*0.001), i.y*(1 - i.h*0.001), i.w*2.5, i.h*2.3);
            cv::Rect ROI2(i.x*(1 - i.w*0.001), i.y*(1 - i.h*0.001), i.w*2.5, i.h*2.3);
	   
	    if(classLabels[i.obj_id] == "m1"){
                marker1_right = cur_frame(ROI1);
                //ROS_INFO("test 1");
                xmin_right_1 = i.x*(1 - i.w*0.001);
                ymin_right_1 = i.y*(1 - i.h*0.001);
                cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/marker1_right.png", marker1_right, compression_params);
                // send bbox image + coordinates over ROS - will be used for stereo
                sensor_msgs::ImagePtr msg_bbox_1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", marker1_right).toImageMsg(); //mono?
                detectionBBoxPublisher_1.publish(msg_bbox_1);
                bboxArray_1.data.clear();
                bboxArray_1.data.push_back(xmin_right_1);
                bboxArray_1.data.push_back(ymin_right_1);
                bboxCoordPublisher_1.publish(bboxArray_1);
            }

	    if(classLabels[i.obj_id] == "m2"){
                marker2_right = cur_frame(ROI2);
                //ROS_INFO("test2");
                xmin_right_2 = i.x*(1 - i.w*0.001);
                ymin_right_2 = i.y*(1 - i.h*0.001);
                cv::imwrite("/home/oysteinvolden/darknet_ws/src/darknet_ros/darknet_ros/src/marker2_right.png", marker2_right, compression_params);
                sensor_msgs::ImagePtr msg_bbox_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", marker2_right).toImageMsg(); //mono?
                detectionBBoxPublisher_2.publish(msg_bbox_2);
                bboxArray_2.data.clear();
                bboxArray_2.data.push_back(xmin_right_2);
                bboxArray_2.data.push_back(ymin_right_2);
                bboxCoordPublisher_2.publish(bboxArray_2);
            }


		
        }
    }

    // maybe unable pose estimation here, will only do pose estimation in the other node

    

	/*
    // Only do pose estimation if yolov3 provide a bounding box
    if(inputImage.empty()){
      do_pose_estimate = false; 
    }
    else{
      //inputImage_r = cv::imread("/home/oysteinvolden/darknet_ws2/src/darknet_ros/darknet_ros/src/bbox-left_test.png", 0);
      inputImage_r = cv::imread("/home/oysteinvolden/darknet_ws2/src/darknet_ros/darknet_ros/src/bbox-right_test.png", 0);
      do_pose_estimate = true;   
    }
    */

	
	// need to update the calibration parameters for this to work!

    if(do_pose_estimate){
    
      // %% part1 - marker detection %%
      cv::aruco::detectMarkers(inputImage_r, dictionary_r, markerCorners_r, markerIds_r, parameters, rejectedCandidates_r);
      cv::aruco::drawDetectedMarkers(inputImage_r, markerCorners_r, markerIds_r, cv::Scalar(0,255,0));


      // %% part2 - calibration parameters %%
      cv::Mat K1(3,3, cv::DataType<float>::type);
      K1.at<float>(0,0) = 380.6637;
      K1.at<float>(0,2) = 330.4397;
      K1.at<float>(1,1) = 381.8116;
      K1.at<float>(1,2) = 265.8672;
      K1.at<float>(2,2) = 1;

      cv::Mat D1(1,4, cv::DataType<float>::type);
      D1.at<float>(0,0) = -0.0132; // k1 - radial 
      D1.at<float>(0,1) = 0.0368;// k2 
      D1.at<float>(0,2) = 0;// p1 - tangential
      D1.at<float>(0,3) = 0; // p2
      //D1.at<float>(0,4) = 0; // k3

      cv::Mat K2(3,3, CV_32F);
      K2.at<float>(0,0) = 379.5845;
      K2.at<float>(0,2) = 325.2762;
      K2.at<float>(1,1) = 380.1453;
      K2.at<float>(1,2) = 262.6230;
      K2.at<float>(2,2) = 1;

      cv::Mat D2(1,5, CV_32F);
      D2.at<float>(0,0) = -0.0701; // k1 - CHECK here
      D2.at<float>(0,1) = 0.1749; // k2
      D2.at<float>(0,2) = 0; // p1
      D2.at<float>(0,3) = 0; // p2
      D2.at<float>(0,4) = -0.1071; // k3


      // part 3: %% find corners relative to full image %%

      std::vector <cv::Point2f> bbox_left, bbox_right;
      bbox_right.push_back(cv::Point2f(xmin_right, ymin_right));

      std::vector < std::vector<cv::Point2f> > markerCorners_l_full, markerCorners_r_full;
      markerCorners_r_full.push_back(std::vector<cv::Point2f>());

      if(markerIds_r.size() > 0 && markerCorners_r.size() > 0){
        for(int j = 0; j < 4; j++){
          markerCorners_r_full[0].push_back(cv::Point2f(markerCorners_r[0][j].x + bbox_right[0].x, markerCorners_r[0][j].y + bbox_right[0].y));
          std::cout << "x: " << markerCorners_r_full[0][j].x << " y: " << markerCorners_r_full[0][j].y << std::endl;

        }
      }

	
	
      // %% part 4: pose estimation %%

      std::vector < cv::Vec3d > rvecs_l, tvecs_l, rvecs_r, tvecs_r;

      if(markerIds_r.size() > 0 && markerCorners_r.size() > 0){
        cv::aruco::estimatePoseSingleMarkers(markerCorners_r_full, 0.2628, K2, D2, rvecs_r, tvecs_r);

        for(int i = 0; i < tvecs_r.size(); i++){
          std::cout << "tvec right cam: " << tvecs_r[i] << std::endl;
	  time_stamp_end = std::chrono::steady_clock::now(); 
	  log_right << std::chrono::duration<double>(time_stamp_end - time_stamp_start).count() << " " << tvecs_r[i][0] << " " << tvecs_r[i][1] << " " << tvecs_r[i][2] << " " << rvecs_r[i][0] << " " << rvecs_r[i][1] << " " << rvecs_r[i][2] << std::endl;
        }
        for(int i = 0; i < rvecs_r.size(); i++){
          std::cout << "rvec right cam: " << rvecs_r[i]*(180/3.1459) << std::endl;
        }

        // convert to left camera xyz coordinates via fixed translation
        //std::cout << "tvec left cam: " << tvecs_r[0][0] + 0.5211686 << " " << tvecs_r[0][1] - 0.027276 << " " << tvecs_r[0][2] - 0.002478  << std::endl;
      }


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
