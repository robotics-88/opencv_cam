#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>

#include "camera_calibration_parsers/parse.hpp"
#include <sensor_msgs/image_encodings.hpp>

#define BYTE __uint8_t


namespace opencv_cam
{

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
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.device_, cv::CAP_V4L2);
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

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      double fps = capture_->get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(get_logger(), "device %d open, width %g, height %g, device fps %g",
                  cxt_.index_, width, height, fps);
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

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    if (see3cam_flag_) {
      image_ir_pub_ = create_publisher<sensor_msgs::msg::Image>("image_ir_raw", 10);
    }

    // Video recorder service
    record_service_ = this->create_service<messages_88::srv::RecordVideo>("~/record", std::bind(&OpencvCamNode::recordVideoCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "start publishing");
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

      std::lock_guard<std::mutex> lock(record_mutex_);

      if (see3cam_flag_) {
        // separate dual image RGB/IR
        cv::Mat RGBImageCU83 = cv::Mat(1080, 1920, CV_8UC2); //allocation
        cv::Mat IRImageCU83 = cv::Mat(1080, 1920, CV_8UC1);
        int RGBBufferSizeCU83, IRBufferSizeCU83 = 0;
        cv::Mat ResultImage;
        if (SeparatingRGBIRBuffers(frame, &IRImageCU83, &RGBImageCU83, &RGBBufferSizeCU83, &IRBufferSizeCU83) == 1)
        {
          bool pose_written = false;
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

            if (recording_) {
              video_writer_.write(RGBImageCU83);
              writeToPoseFile(stamp);
              pose_written = true;
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

            if (recording_) {
              video_writer_ir_.write(IRImageCU83);
              if (!pose_written)
                writeToPoseFile(stamp);
            }
          }
        }
        else
        {
          std::cout << "SeparatingRGBIRBuffers failed" << std::endl;
        }
      }
      else {

        // Avoid copying image message if possible
        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

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

        // Record frame to video file
        if (recording_) {
          video_writer_.write(frame);
          writeToPoseFile(stamp);
        }

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

  bool OpencvCamNode::recordVideoCallback(const std::shared_ptr<messages_88::srv::RecordVideo::Request> req,
    std::shared_ptr<messages_88::srv::RecordVideo::Response> resp) {
    bool success;

    // Use lock guard here to ensure thread safety with main loop which operates in its own thread
    std::lock_guard<std::mutex> lock(record_mutex_);
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

    video_writer_.open(filename, 
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
                        (double)cxt_.fps_, 
                        cv::Size(cxt_.width_, cxt_.height_));

    if (!video_writer_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video file for writing.");
        return false;
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Started recording to %s", filename.c_str());
    }

    if (see3cam_flag_) {
      std::string filename_ir = filename.substr(0, filename.find_last_of(".")) + "_ir.mp4";
      video_writer_ir_.open(filename_ir, 
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
        (double)cxt_.fps_, 
        cv::Size(cxt_.width_, cxt_.height_));

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

    // Open the pose file
    std::string pose_filename = filename.substr(0, filename.find_last_of(".")) + "_pose.txt";
    pose_file_.open(pose_filename);

    return true;
  }

  bool OpencvCamNode::stopRecording() {
    if (recording_)
    {
      video_writer_.release();
      if (pose_file_.is_open())
        pose_file_.close();
      recording_ = false;
      frame_count_ = 0;
      RCLCPP_INFO(this->get_logger(), "Stopped recording.");
    }
    return true;
  }

  void OpencvCamNode::writeToPoseFile(rclcpp::Time stamp) {
    // Get the current camera pose from the TF tree
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform(map_frame_, cxt_.camera_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // TODO make this a service arg
    int camera_id = 1;
    std::string image_name = "image_" + std::to_string(frame_count_) + ".png";
    // Write the pose to the pose file
    if (pose_file_.is_open())
    {
      pose_file_  << frame_count_ << " ";
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

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)