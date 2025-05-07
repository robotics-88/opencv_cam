#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <regex>
#include <opencv2/calib3d.hpp>

#include "camera_calibration_parsers/parse.hpp"
#include <sensor_msgs/image_encodings.hpp>

#define BYTE __uint8_t


namespace opencv_cam
{
  
  // Utility function to check hardware and return preferred GStreamer encoder pipeline
  std::string get_gstreamer_pipeline(const std::string &filename) {
    std::string home = std::getenv("HOME");
    std::string path = home + "/r88_public/device-tree/model";
    std::ifstream model_file(path);
    std::string pipeline;
    if (model_file.is_open()) {
      std::string model;
      std::getline(model_file, model);
      std::cout << "Detected model: " << model << std::endl;
      if (model.find("Orin") != std::string::npos || model.find("Jetson AGX") != std::string::npos) {
        std::cout << "OpenCV_Cam using hardware encoding for Orin/Jetson AGX" << std::endl;
      } 
      // TODO figure out why Nano doesnt have the above encoder, it should be the same as NX
      // else if (model.find("Jetson Nano") != std::string::npos) {
      //   std::cout << "OpenCV_Cam using hardware encoding for Jetson Nano" << std::endl;
      //   encoder = "nvv4l2h264enc";
      // }

      pipeline = "appsrc ! videoconvert ! nvvidconv ! nvv4l2h264enc preset-level=1 insert-sps-pps=true "
                             "! h264parse ! mp4mux ! filesink location=" + filename;
    }
    else {
      pipeline = "appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency "
                             "! h264parse ! mp4mux ! filesink location=" + filename;
    }

    return pipeline;
  }

  // Utility: Determine camera pixel format using v4l2-ctl
  std::string get_camera_pixel_format(const std::string &device) {
    std::string cmd = "v4l2-ctl --device=" + device + " --get-fmt-video 2>&1";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return "";

    char buffer[256];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      output += buffer;
    }
    pclose(pipe);

    std::smatch match;
    if (std::regex_search(output, match, std::regex("Pixel Format:\s+'(\\w+)'"))) {
      std::string fmt = match[1];
      std::cout << "Detected camera pixel format: " << fmt << std::endl;
      return fmt;
    }

    std::cout << "Could not detect pixel format, falling back to MJPEG pipeline." << std::endl;
    return "MJPG"; // fallback
  }

  // Utility function to generate fastest capture pipeline based on hardware and camera format
  std::string get_capture_pipeline(const std::string &device, int width, int height, int fps) {
    std::string home = std::getenv("HOME");
    std::string path = home + "/r88_public/device-tree/model";
    std::ifstream model_file(path);
    std::string model;
    if (model_file.is_open()) {
      std::getline(model_file, model);
    }

    std::string format = get_camera_pixel_format(device);
    std::ostringstream pipeline;

    if (model.find("Orin") != std::string::npos || model.find("Jetson AGX") != std::string::npos || model.find("Jetson Nano") != std::string::npos) {
      if (format == "MJPG") {
        pipeline << "v4l2src device=" << device
                << " ! image/jpeg, width=" << width
                << ", height=" << height
                << ", framerate=" << fps << "/1 "
                << "! jpegdec ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";
      } else {
        pipeline << "v4l2src device=" << device
                << " ! video/x-raw, width=" << width
                << ", height=" << height
                << ", framerate=" << fps << "/1 "
                << "! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";
      }
    } else {
      pipeline << "v4l2src device=" << device
              << " ! image/jpeg, width=" << width
              << ", height=" << height
              << ", framerate=" << fps << "/1 "
              << "! jpegdec ! videoconvert ! appsink";
    }

    std::cout << "Final GStreamer pipeline: " << pipeline.str() << std::endl;
    return pipeline.str();
  }

  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("unsupported encoding type");
    }
  }

  OpencvCamNode::OpencvCamNode(const rclcpp::NodeOptions &options) :
    Node("opencv_cam", options)
    , map_frame_("map")
    , recording_(false)
    , frame_count_(0)
    , written_frame_count_(0)
    , canceled_(false)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

    // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Register for parameter changed. NOTE at this point nothing is done when parameters change.
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Log the current parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
    CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "opencv_cam Parameters", OPENCV_CAM_ALL_PARAMS)

    // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), OPENCV_CAM_ALL_PARAMS)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    // Check if see3cam camera is used
    if (cxt_.camera_frame_id_.find("see3") != std::string::npos) {
      see3cam_flag_ = true;
    } else {
      see3cam_flag_ = false;
    }

    // Open file or device
    if (cxt_.file_) {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_);

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open file %s", cxt_.filename_.c_str());
        return;
      }

      if (cxt_.fps_ > 0) {
        // Publish at the specified rate
        publish_fps_ = cxt_.fps_;
      } else {
        // Publish at the recorded rate
        publish_fps_ = static_cast<int>(capture_->get(cv::CAP_PROP_FPS));
      }
      

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      RCLCPP_INFO(get_logger(), "file %s open, width %g, height %g, publish fps %d",
                  cxt_.filename_.c_str(), width, height, publish_fps_);

      next_stamp_ = now();

    } else {
      std::string pipeline = get_capture_pipeline(cxt_.device_, cxt_.width_, cxt_.height_, cxt_.fps_);
      capture_ = std::make_shared<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
      if (see3cam_flag_) {
        capture_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','1','6',' '));
        capture_->set(cv::CAP_PROP_CONVERT_RGB, false);
      }

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open device %s", cxt_.device_.c_str());
        return;
      }

      if (cxt_.height_ > 0) {
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, cxt_.height_);
      }

      if (cxt_.width_ > 0) {
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, cxt_.width_);
      }

      if (cxt_.fps_ > 0) {
        capture_->set(cv::CAP_PROP_FPS, cxt_.fps_);
      }
      else {
        RCLCPP_ERROR(get_logger(), "fps not set, not starting node");
        return;
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      device_fps_ = capture_->get(cv::CAP_PROP_FPS);

      RCLCPP_INFO(get_logger(), "device %d open, width %g, height %g, device fps %g",
                  cxt_.index_, width, height, device_fps_);
    }

    assert(!cxt_.camera_info_path_.empty()); // readCalibration will crash if file_name is ""
    std::string camera_name;
    if (camera_calibration_parsers::readCalibration(cxt_.camera_info_path_, camera_name, camera_info_msg_)) {
      RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
      camera_info_msg_.header.frame_id = cxt_.camera_frame_id_;
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    } else {
      RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
      camera_info_pub_ = nullptr;
    }

    // Init fisheye
    if (camera_info_msg_.distortion_model == "fisheye") {
      K_fisheye_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info_msg_.k.data())).clone();
      D_fisheye_ = cv::Mat(1, camera_info_msg_.d.size(), CV_64F, const_cast<double*>(camera_info_msg_.d.data())).clone();
      R_fisheye_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info_msg_.r.data())).clone();
      P_fisheye_ = cv::Mat(3, 4, CV_64F, const_cast<double*>(camera_info_msg_.p.data())).clone();
      image_size_ = cv::Size(camera_info_msg_.width, camera_info_msg_.height);
      initFisheyeUndistortMaps();
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    if (see3cam_flag_) {
      image_ir_pub_ = create_publisher<sensor_msgs::msg::Image>("image_ir_raw", 10);
    }
    rectified_image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_rect", 10);


    // Video recorder service
    record_service_ = this->create_service<messages_88::srv::RecordVideo>("~/record", std::bind(&OpencvCamNode::recordVideoCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "Start publishing");

    meas_fps_pub_ = create_publisher<std_msgs::msg::Float32>("meas_fps", 10);

    temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/i2c_temperature", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        latest_temp_ = msg->data;
      });    

    // Publish how many frames received in last second
    meas_fps_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std_msgs::msg::Float32 msg;
      msg.data = frame_count_ - last_frame_count_;
      meas_fps_pub_->publish(msg);
      last_frame_count_ = frame_count_;
    });

    //record_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / device_fps_), std::bind(&OpencvCamNode::writeVideo, this));
  }

  OpencvCamNode::~OpencvCamNode()
  {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void OpencvCamNode::validate_parameters()
  {}

  void OpencvCamNode::initFisheyeUndistortMaps()
  {
    if (!fisheye_maps_initialized_ && camera_info_msg_.distortion_model == "fisheye") {
      cv::fisheye::initUndistortRectifyMap(
        K_fisheye_,
        D_fisheye_,
        R_fisheye_,
        K_fisheye_, // Or P_fisheye_.colRange(0, 3) if you want to project to new intrinsics
        image_size_,
        CV_16SC2,
        map1_,
        map2_);
      fisheye_maps_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialized fisheye undistort maps");
    }
  }


  bool OpencvCamNode::SeparatingRGBIRBuffers(cv::Mat frame, cv::Mat* IRImageCU83, cv::Mat* RGBImageCU83, int *RGBBufferSizeCU83, int *IRBufferSizeCU83)
  {
    uchar* RGBIRBuff = frame.data;
    // int long size = Frame.cols*Frame.rows * 2;

    int Buffcnt = 0;
    int cnt = 0;
    int RGBBufsize = 0;
    //Changed the datatype from UINT32 to uint32_t - commented by Sushanth
    //Reason - compatible for both linux & windows
    uint32_t IrBufsize = 0;
    int rgbcnt = 0, Ircnt = 0;
    BYTE * IRBuff = NULL;
    IRBuff = (BYTE*)malloc(1920 * 1080 * 2);
    int long BuffSize = 3120 * 1080 * 2;

    if (true)
    // if ((Frame.cols == 3120 && Frame.rows == 1080)) // Later bring back handling different dual images
    {
      // int rgbbuffsize = 1920 * 1080 * 2;
      // int irbuffsize = 2592000;

      while (BuffSize > 0)
      {
        if ((RGBIRBuff[Buffcnt] & 0x03) == 0x00)
        {
          memcpy(RGBImageCU83->data + (RGBBufsize), RGBIRBuff + Buffcnt, 3839);
          Buffcnt += 3840;
          RGBBufsize += 3840;
          BuffSize -= 3840;
          rgbcnt += 1;
        }
        else if ((RGBIRBuff[Buffcnt] & 0x03) == 0x03)
        {
          memcpy(IRBuff + (IrBufsize), RGBIRBuff + Buffcnt, 2399);
          IrBufsize += 2400;
          Buffcnt += 2400;
          BuffSize -= 2400;
          Ircnt += 1;
        }
        else
        {
          return 1;
        }
        cnt++;
      }
    }
    // else
    // {
    // 	while (size > 0)
    // 	{
    // 		if ((RGBIRBuff[Buffcnt] & 0x03) == 0x00)
    // 		{
    // 			memcpy(RGBImageCU83->data + (RGBBufsize), RGBIRBuff + Buffcnt, 7679);
    // 			Buffcnt += 7680;
    // 			RGBBufsize += 7680;
    // 			size -= 7680;
    // 			rgbcnt += 1;
    // 		}
    // 		else if ((RGBIRBuff[Buffcnt] & 0x03) == 0x03)
    // 		{
    // 			memcpy(IRBuff + (IrBufsize), RGBIRBuff + Buffcnt, 2399);
    // 			IrBufsize += 2400;
    // 			Buffcnt += 2400;
    // 			size -= 2400;
    // 			Ircnt += 1;
    // 		}
    // 		else
    // 		{
    // 			return 0;
    // 		}
    // 		cnt++;
    // 	}
    // }

    int bufsize_IR = 0;
    Buffcnt = 0;
    while (IrBufsize > 0)
    {
      memcpy(IRImageCU83->data + (bufsize_IR), IRBuff + Buffcnt, 4);
      bufsize_IR += 4;
      Buffcnt += 5;
      IrBufsize -= 5;
    }
    Buffcnt = 0;

    *RGBBufferSizeCU83 = RGBBufsize;
    *IRBufferSizeCU83 = IrBufsize;
    free(IRBuff);
    // IRBuff = NULL; //Added by Sushanth - Assigning IRBuff to NULL when it is freed.
    return 1;
  }

  void OpencvCamNode::loop()
  {
    cv::Mat frame;

    while (rclcpp::ok() && !canceled_.load()) {
      // Read a frame, if this is a device block until a frame is available
      if (!capture_->read(frame)) {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing");
        break;
      }

      auto stamp = now();

      if (see3cam_flag_) {
        handleSee3CamFrame(frame, stamp);
      }
      else {
        handleGenericFrame(frame, stamp);
      }

      // Increment frame count after processing
      frame_count_++;

      // Sleep if required
      if (cxt_.file_) {
        using namespace std::chrono_literals;
        next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
        auto wait = next_stamp_ - stamp;
        if (wait.nanoseconds() > 0) {
          std::this_thread::sleep_for(static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
        }
      }
    }
  }

  void OpencvCamNode::handleSee3CamFrame(cv::Mat &frame, rclcpp::Time stamp) {
    // separate dual image RGB/IR
    cv::Mat RGBImageCU83 = cv::Mat(1080, 1920, CV_8UC2); //allocation
    cv::Mat IRImageCU83 = cv::Mat(1080, 1920, CV_8UC1);
    int RGBBufferSizeCU83, IRBufferSizeCU83 = 0;
    cv::Mat ResultImage;
    if (SeparatingRGBIRBuffers(frame, &IRImageCU83, &RGBImageCU83, &RGBBufferSizeCU83, &IRBufferSizeCU83))
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!RGBImageCU83.empty())
      {
        cvtColor(RGBImageCU83, ResultImage, cv::COLOR_YUV2BGR_UYVY);

        // Avoid copying image message if possible
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

        // Convert OpenCV Mat to ROS Image
        image_msg->header.stamp = stamp;
        image_msg->header.frame_id = cxt_.camera_frame_id_;
        image_msg->height = ResultImage.rows;
        image_msg->width = ResultImage.cols;
        image_msg->encoding = sensor_msgs::image_encodings::BGR8;
        image_msg->is_bigendian = false;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(ResultImage.step);
        image_msg->data.assign(ResultImage.datastart, ResultImage.dataend);
        image_pub_->publish(std::move(image_msg));

        if (last_frame_.empty() || last_frame_.size != ResultImage.size || last_frame_.type() != ResultImage.type()) {
          last_frame_ = ResultImage.clone();
        }
        else {
          ResultImage.copyTo(last_frame_);
        }
      }
      if (!IRImageCU83.empty())
      {
        // Avoid copying image message if possible
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

        // Convert OpenCV Mat to ROS Image
        image_msg->header.stamp = stamp;
        image_msg->header.frame_id = cxt_.camera_frame_id_;
        image_msg->height = IRImageCU83.rows;
        image_msg->width = IRImageCU83.cols;
        image_msg->encoding = sensor_msgs::image_encodings::MONO8;
        image_msg->is_bigendian = false;
        image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(IRImageCU83.step);
        image_msg->data.assign(IRImageCU83.datastart, IRImageCU83.dataend);
        image_ir_pub_->publish(std::move(image_msg));

        if (last_ir_frame_.empty() || last_ir_frame_.size != ResultImage.size || last_ir_frame_.type() != ResultImage.type()) {
          last_ir_frame_ = IRImageCU83.clone();
        }
        else {
          IRImageCU83.copyTo(last_ir_frame_);
        }
      }
    }
    else
    {
      std::cout << "SeparatingRGBIRBuffers failed" << std::endl;
    }
  }

  void OpencvCamNode::handleGenericFrame(cv::Mat &frame, rclcpp::Time stamp) {

    std::lock_guard<std::mutex> lock(frame_mutex_);
    // Avoid copying image message if possible
    sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

    if (!std::isnan(latest_temp_)) {
      std::ostringstream temp_text;
      temp_text << std::fixed << std::setprecision(1) << latest_temp_ << " °C";
      cv::putText(frame, temp_text.str(), cv::Point(20, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 165, 255), 3); // Orange
    }    

    // Convert OpenCV Mat to ROS Image
    image_msg->header.stamp = stamp;
    image_msg->header.frame_id = cxt_.camera_frame_id_;
    image_msg->height = frame.rows;
    image_msg->width = frame.cols;
    image_msg->encoding = mat_type2encoding(frame.type());
    image_msg->is_bigendian = false;
    image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    image_msg->data.assign(frame.datastart, frame.dataend);

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
    static int count = 0;
    RCLCPP_INFO(get_logger(), "%d, %p", count++, reinterpret_cast<std::uintptr_t>(image_msg.get()));
#endif

    // Publish
    image_pub_->publish(std::move(image_msg));
    if (camera_info_pub_) {
      camera_info_msg_.header.stamp = stamp;
      camera_info_pub_->publish(camera_info_msg_);
    }
    if (camera_info_msg_.distortion_model == "fisheye") {
      cv::Mat undistorted;
      cv::remap(frame, undistorted, map1_, map2_, cv::INTER_LINEAR);
      cv::Mat rotated_image;
      cv::Point2f center(undistorted.cols / 2.0, undistorted.rows / 2.0);
      cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, 180, 1.0);
      cv::warpAffine(undistorted, rotated_image, rotation_matrix, undistorted.size());
      last_rect_frame_ = rotated_image.clone();
    
      sensor_msgs::msg::Image::UniquePtr rectified_msg(new sensor_msgs::msg::Image());
      rectified_msg->header.stamp = stamp;
      rectified_msg->header.frame_id = cxt_.camera_frame_id_;
      rectified_msg->height = rotated_image.rows;
      rectified_msg->width = rotated_image.cols;
      rectified_msg->encoding = mat_type2encoding(rotated_image.type());
      rectified_msg->is_bigendian = false;
      rectified_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(rotated_image.step);
      rectified_msg->data.assign(rotated_image.datastart, rotated_image.dataend);
      rectified_image_pub_->publish(std::move(rectified_msg));
    }
    

    // if (last_frame_.empty() || last_frame_.size != frame.size || last_frame_.type() != frame.type()) {
    //   last_frame_ = frame.clone();
    // }
    // else {
    //   frame.copyTo(last_frame_);
    // }

    cv::Mat bgr_frame;
    if (camera_info_msg_.distortion_model == "fisheye") {
      if (frame.channels() == 1) {
        cv::cvtColor(last_rect_frame_, bgr_frame, cv::COLOR_GRAY2BGR);
      } else if (frame.channels() == 4) {
        cv::cvtColor(last_rect_frame_, bgr_frame, cv::COLOR_BGRA2BGR);
      } else {
        bgr_frame = last_rect_frame_;  // Assume already BGR
      }  
    }
    else {
      if (frame.channels() == 1) {
        cv::cvtColor(frame, bgr_frame, cv::COLOR_GRAY2BGR);
      } else if (frame.channels() == 4) {
        cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
      } else {
        bgr_frame = frame;  // Assume already BGR
      }
  
    }
    
    if (recording_) {
      // Add the frame to the queue
      std::lock_guard<std::mutex> lock(writer_mutex_);
      if (frame_queue_.size() < 100) {
        frame_queue_.push(bgr_frame.clone());
      } else {
        RCLCPP_WARN(this->get_logger(), "Frame queue full, dropping frame");
      }
    }
  }

  bool OpencvCamNode::recordVideoCallback(const std::shared_ptr<messages_88::srv::RecordVideo::Request> req,
    std::shared_ptr<messages_88::srv::RecordVideo::Response> resp) {
    bool success;

    if (req->start)
      success = startRecording(req->filename);
    else
      success = stopRecording();

    resp->success = success;
    return success;
  }

  bool OpencvCamNode::startRecording(const std::string &filename) {
    if (recording_)
    {
        RCLCPP_WARN(this->get_logger(), "Already recording!");
        return false;
    }

    // If see3cam, split into two 1920 width images.
    int width;
    if (see3cam_flag_) {
      width = 1920;
    } else {
      width = cxt_.width_;
    }

    std::string pipeline = get_gstreamer_pipeline(filename);
    bool use_pipeline = (pipeline != filename);

    if (use_pipeline) {
      RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline: %s", pipeline.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Using default OpenCV VideoWriter");
    }

    video_writer_.open(pipeline,
                        use_pipeline ? cv::CAP_GSTREAMER : 0,
                        device_fps_,
                        cv::Size(width, cxt_.height_),
                        true);

    if (!video_writer_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video file for writing.");
        return false;
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Started recording to %s", filename.c_str());
    }

    // If see3cam_flag_ set, open a second video writer for the IR image
    if (see3cam_flag_) {
      std::string filename_ir = filename.substr(0, filename.find_last_of(".")) + "_ir.mp4";
      std::string pipeline_ir = get_gstreamer_pipeline(filename_ir);
      bool use_pipeline_ir = (pipeline_ir != filename_ir);

      video_writer_ir_.open(pipeline_ir,
                            use_pipeline_ir ? cv::CAP_GSTREAMER : 0,
                            device_fps_,
                            cv::Size(width, cxt_.height_),
                            false);

      if (!video_writer_ir_.isOpened())
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video file for writing.");
        return false;
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Started recording to %s", filename_ir.c_str());
      }
    }

    recording_ = true;
    stop_writer_thread_ = false;
    writer_thread_ = std::thread(&OpencvCamNode::writerLoop, this);

    // Open the pose file
    std::string pose_filename = filename.substr(0, filename.find_last_of(".")) + "_pose.txt";
    pose_file_.open(pose_filename);

    return true;
  }

  bool OpencvCamNode::stopRecording() {
    if (recording_)
    {
      {std::lock_guard<std::mutex> lock(writer_mutex_);
        stop_writer_thread_ = true;
      }
      if (writer_thread_.joinable()) {
        writer_thread_.join();
      }

      if (video_writer_.isOpened())
        video_writer_.release();
      if (video_writer_ir_.isOpened())
        video_writer_ir_.release();
      if (pose_file_.is_open())
        pose_file_.close();
      recording_ = false;
      written_frame_count_ = 0;
      RCLCPP_INFO(this->get_logger(), "Stopped recording.");
    }
    return true;
  }

  void OpencvCamNode::writeToPoseFile() {
    // Get the current camera pose from the TF tree
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform(map_frame_, cxt_.camera_frame_id_, tf2::TimePointZero);
      rclcpp::Time transform_time(transform_stamped.header.stamp);
      if (now() - transform_time > rclcpp::Duration::from_seconds(0.5))
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Camera TF is older than 0.5 seconds");
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform: %s", ex.what());
      return;
    }

    // TODO make this a service arg
    int camera_id = 1;
    std::string image_name = "image_" + std::to_string(written_frame_count_) + ".png";
    // Write the pose to the pose file
    if (pose_file_.is_open())
    {
      pose_file_  << written_frame_count_ << " ";
      pose_file_  << transform_stamped.transform.rotation.w << " " 
                  << transform_stamped.transform.rotation.x << " "
                  << transform_stamped.transform.rotation.y << " " 
                  << transform_stamped.transform.rotation.z << " ";
      pose_file_  << transform_stamped.transform.translation.x << " " 
                  << transform_stamped.transform.translation.y << " " 
                  << transform_stamped.transform.translation.z << " ";
      pose_file_  << std::to_string(camera_id) << " " << image_name << "\n";
      pose_file_  << "\n"; // Leave blank line between frames
    }
  }

  void OpencvCamNode::writeVideo() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (recording_) {

      // TODO: determine why see3cam write takes so long and therefore causes delays.
      if (video_writer_.isOpened()) {
        if (camera_info_msg_.distortion_model == "fisheye") {
          video_writer_.write(last_rect_frame_);
        } else {
          video_writer_.write(last_frame_);
        }
      }
      if (video_writer_ir_.isOpened()) {
        video_writer_ir_.write(last_ir_frame_);
      }
      // writeToPoseFile();
      written_frame_count_++;
    }
  }

  void OpencvCamNode::writerLoop() {
    cv::Mat last_frame;
    rclcpp::Rate rate(device_fps_);

    while (rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock(writer_mutex_);
        if (!frame_queue_.empty()) {
          last_frame = std::move(frame_queue_.front());
          frame_queue_.pop();
        }
      }
  
      if (!last_frame.empty()) {
        video_writer_.write(last_frame);
      }
  
      if (stop_writer_thread_) break;
      rate.sleep();
    }
  }
} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)