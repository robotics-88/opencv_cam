#include "opencv_cam/opencv_cam_node.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <regex>
#include <sstream>

#include "camera_calibration_parsers/parse.hpp"
#include <sensor_msgs/image_encodings.hpp>

#define BYTE __uint8_t

namespace opencv_cam {

// Utility function to check hardware and return preferred GStreamer encoder pipeline
std::string get_gstreamer_pipeline(const std::string &filename) {
    std::string home = std::getenv("HOME");
    std::string path = home + "/r88_public/device-tree/model";
    std::ifstream model_file(path);

    std::string pipeline =
        "appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency "
        "! h264parse ! mp4mux ! filesink location=" +
        filename;

    // If model file includes jetson AGX or Orin, use hardware encoding
    if (model_file.is_open()) {
        std::string model;
        std::getline(model_file, model);
        std::cout << "Detected model: " << model << std::endl;
        if (model.find("Orin") != std::string::npos ||
            model.find("Jetson AGX") != std::string::npos) {
            std::cout << "OpenCV_Cam using hardware encoding for Orin/Jetson AGX" << std::endl;

            pipeline = "appsrc ! videoconvert ! nvvidconv ! nvv4l2h264enc preset-level=1 "
                       "insert-sps-pps=true "
                       "! h264parse ! mp4mux ! filesink location=" +
                       filename;
        }
    }

    return pipeline;
}
// Utility: Determine camera pixel format using v4l2-ctl
std::string get_camera_pixel_format(const std::string &device) {
    std::string cmd = "v4l2-ctl --device=" + device + " --get-fmt-video 2>&1";
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        return "";

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

    if (model.find("Orin") != std::string::npos || model.find("Jetson AGX") != std::string::npos ||
        model.find("Jetson Nano") != std::string::npos) {
        if (format == "MJPG") {
            pipeline << "v4l2src device=" << device << " ! image/jpeg, width=" << width
                     << ", height=" << height << ", framerate=" << fps << "/1 "
                     << "! jpegdec ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";
        } else {
            pipeline << "v4l2src device=" << device << " ! video/x-raw, width=" << width
                     << ", height=" << height << ", framerate=" << fps << "/1 "
                     << "! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";
        }
    } else {
        pipeline << "v4l2src device=" << device << " ! image/jpeg, width=" << width
                 << ", height=" << height << ", framerate=" << fps << "/1 "
                 << "! jpegdec ! videoconvert ! appsink";
    }

    std::cout << "Final GStreamer pipeline: " << pipeline.str() << std::endl;
    return pipeline.str();
}

std::string mat_type2encoding(int mat_type) {
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

OpencvCamNode::OpencvCamNode(const rclcpp::NodeOptions &options)
    : Node("opencv_cam", options),
      map_frame_("map"),
      recording_(false),
      frame_count_(0),
      written_frame_count_(0),
      canceled_(false),
      file_(false),
      filename_(""),
      fps_(0),
      device_(""),
      width_(0),
      height_(0),
      camera_info_path_(""),
      camera_frame_id_("") {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("file", file_);
    this->declare_parameter("filename", filename_);
    this->declare_parameter("fps", fps_);
    this->declare_parameter("device", device_);
    this->declare_parameter("width", width_);
    this->declare_parameter("height", height_);
    this->declare_parameter("camera_info_path", camera_info_path_);
    this->declare_parameter("camera_frame_id", camera_frame_id_);

    this->get_parameter("file", file_);
    this->get_parameter("filename", filename_);
    this->get_parameter("fps", fps_);
    this->get_parameter("device", device_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("camera_info_path", camera_info_path_);
    this->get_parameter("camera_frame_id", camera_frame_id_);

    // Check if see3cam camera is used
    if (camera_frame_id_.find("see3") != std::string::npos) {
        see3cam_flag_ = true;
    } else {
        see3cam_flag_ = false;
    }

    // Open file or device
    if (file_) {
        capture_ = std::make_shared<cv::VideoCapture>(filename_);

        if (!capture_->isOpened()) {
            RCLCPP_ERROR(get_logger(), "Cannot open file %s", filename_.c_str());
            return;
        }

        if (fps_ > 0) {
            // Publish at the specified rate
            publish_fps_ = fps_;
        } else {
            // Publish at the recorded rate
            publish_fps_ = static_cast<int>(capture_->get(cv::CAP_PROP_FPS));
        }

        double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
        double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
        RCLCPP_INFO(get_logger(), "File %s open, width %g, height %g, publish fps %d",
                    filename_.c_str(), width, height, publish_fps_);

        next_stamp_ = now();

    } else {
        std::string pipeline = get_capture_pipeline(device_, width_, height_, fps_);
        capture_ = std::make_shared<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
        if (see3cam_flag_) {
            capture_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', '1', '6', ' '));
            capture_->set(cv::CAP_PROP_CONVERT_RGB, false);
        }

        if (!capture_->isOpened()) {
            RCLCPP_ERROR(get_logger(), "cannot open device %s", device_.c_str());
            return;
        }

        if (height_ > 0) {
            capture_->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        }

        if (width_ > 0) {
            capture_->set(cv::CAP_PROP_FRAME_WIDTH, width_);
        }

        if (fps_ > 0) {
            capture_->set(cv::CAP_PROP_FPS, fps_);
        } else {
            RCLCPP_ERROR(get_logger(), "fps not set, not starting node");
            return;
        }

        double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
        double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
        device_fps_ = capture_->get(cv::CAP_PROP_FPS);

        RCLCPP_INFO(get_logger(), "Device %s open, width %g, height %g, fps %g", device_.c_str(),
                    width, height, device_fps_);
    }

    assert(!camera_info_path_.empty()); // readCalibration will crash if file_name is ""
    if (camera_calibration_parsers::readCalibration(camera_info_path_, camera_name_,
                                                    camera_info_msg_)) {
        RCLCPP_INFO(get_logger(), "Got camera info for '%s'", camera_name_.c_str());
        camera_info_msg_.header.frame_id = camera_frame_id_;
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    } else {
        RCLCPP_ERROR(get_logger(), "Cannot get camera info, will not publish");
        camera_info_pub_ = nullptr;
    }

    // Init fisheye
    if (camera_info_msg_.distortion_model == "fisheye") {
        K_fisheye_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info_msg_.k.data())).clone();
        D_fisheye_ = cv::Mat(1, camera_info_msg_.d.size(), CV_64F,
                             const_cast<double *>(camera_info_msg_.d.data()))
                         .clone();
        R_fisheye_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info_msg_.r.data())).clone();
        P_fisheye_ = cv::Mat(3, 4, CV_64F, const_cast<double *>(camera_info_msg_.p.data())).clone();
        image_size_ = cv::Size(camera_info_msg_.width, camera_info_msg_.height);
        initFisheyeUndistortMaps();
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    rectified_image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_rect", 10);

    // Video recorder service
    trigger_recording_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/trigger_recording", 10,
        std::bind(&OpencvCamNode::triggerRecordingCallback, this, std::placeholders::_1));

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "Start publishing");

    meas_fps_pub_ = create_publisher<std_msgs::msg::Float32>("meas_fps", 10);

    temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/i2c_temperature", 10,
        [this](std_msgs::msg::Float32::SharedPtr msg) { latest_temp_ = msg->data; });

    // Publish how many frames received in last second
    meas_fps_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        std_msgs::msg::Float32 msg;
        msg.data = frame_count_ - last_frame_count_;
        meas_fps_pub_->publish(msg);
        last_frame_count_ = frame_count_;
    });
}

OpencvCamNode::~OpencvCamNode() {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void OpencvCamNode::initFisheyeUndistortMaps() {
    if (!fisheye_maps_initialized_ && camera_info_msg_.distortion_model == "fisheye") {
        cv::fisheye::initUndistortRectifyMap(
            K_fisheye_, D_fisheye_, R_fisheye_,
            K_fisheye_, // Or P_fisheye_.colRange(0, 3) if you want to project to new intrinsics
            image_size_, CV_16SC2, map1_, map2_);
        fisheye_maps_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initialized fisheye undistort maps");
    }
}

void OpencvCamNode::loop() {
    cv::Mat frame;

    while (rclcpp::ok() && !canceled_.load()) {
        // Read a frame, if this is a device block until a frame is available
        if (!capture_->read(frame)) {
            RCLCPP_INFO(get_logger(), "EOF, stop publishing");
            break;
        }

        auto stamp = now();

        handleFrame(frame, stamp);

        // Sleep if required
        if (file_) {
            using namespace std::chrono_literals;
            next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
            auto wait = next_stamp_ - stamp;
            if (wait.nanoseconds() > 0) {
                std::this_thread::sleep_for(
                    static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
            }
        }
    }
}

void OpencvCamNode::handleFrame(cv::Mat &frame, rclcpp::Time stamp) {

    // Avoid copying image message if possible
    sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

    if (!std::isnan(latest_temp_)) {
        std::ostringstream temp_text;
        temp_text << std::fixed << std::setprecision(1) << latest_temp_ << " °C";
        cv::putText(frame, temp_text.str(), cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5,
                    cv::Scalar(0, 165, 255), 3); // Orange
    }

    // Convert OpenCV Mat to ROS Image
    image_msg->header.stamp = stamp;
    image_msg->header.frame_id = camera_frame_id_;
    image_msg->height = frame.rows;
    image_msg->width = frame.cols;
    image_msg->encoding = mat_type2encoding(frame.type());
    image_msg->is_bigendian = false;
    image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    image_msg->data.assign(frame.datastart, frame.dataend);

    // Publish
    image_pub_->publish(std::move(image_msg));
    if (camera_info_pub_) {
        camera_info_msg_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_msg_);
    }

    cv::Mat rec_frame;
    if (camera_info_msg_.distortion_model == "fisheye") {
        cv::Mat undistorted;
        cv::remap(frame, undistorted, map1_, map2_, cv::INTER_LINEAR);
        cv::Mat rotated_image;
        cv::Point2f center(undistorted.cols / 2.0, undistorted.rows / 2.0);
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, 180, 1.0);
        cv::warpAffine(undistorted, rotated_image, rotation_matrix, undistorted.size());
        rec_frame = rotated_image.clone();

        sensor_msgs::msg::Image::UniquePtr rectified_msg(new sensor_msgs::msg::Image());
        rectified_msg->header.stamp = stamp;
        rectified_msg->header.frame_id = camera_frame_id_;
        rectified_msg->height = rotated_image.rows;
        rectified_msg->width = rotated_image.cols;
        rectified_msg->encoding = mat_type2encoding(rotated_image.type());
        rectified_msg->is_bigendian = false;
        rectified_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(rotated_image.step);
        rectified_msg->data.assign(rotated_image.datastart, rotated_image.dataend);
        rectified_image_pub_->publish(std::move(rectified_msg));
    } else {
        rec_frame = frame.clone();
    }

    if (recording_) {

        cv::Mat bgr_frame;
        if (frame.channels() == 1) {
            cv::cvtColor(rec_frame, bgr_frame, cv::COLOR_GRAY2BGR);
        } else if (frame.channels() == 4) {
            cv::cvtColor(rec_frame, bgr_frame, cv::COLOR_BGRA2BGR);
        } else {
            bgr_frame = rec_frame.clone(); // Assume already BGR
        }

        // Add the frame to the queue
        std::lock_guard<std::mutex> lock(writer_mutex_);
        if (frame_queue_.size() < 100) {
            frame_queue_.push(bgr_frame.clone());
        } else {
            RCLCPP_WARN(this->get_logger(), "Frame queue full, dropping frame");
        }
    }

    // Increment frame count after processing
    frame_count_++;
}

void OpencvCamNode::triggerRecordingCallback(const std_msgs::msg::String::SharedPtr msg) {

    if (!msg->data.empty())
        startRecording(msg->data);
    else
        stopRecording();
}

bool OpencvCamNode::startRecording(const std::string data_directory) {
    std::string filename = data_directory + "/" + camera_name_ + "_" + get_time_str() + ".mp4";

    if (recording_) {
        RCLCPP_WARN(this->get_logger(), "Already recording!");
        return false;
    }

    std::string pipeline = get_gstreamer_pipeline(filename);
    bool use_pipeline = (pipeline != filename);

    if (use_pipeline) {
        RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline: %s", pipeline.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Using default OpenCV VideoWriter");
    }

    video_writer_.open(pipeline, use_pipeline ? cv::CAP_GSTREAMER : 0, device_fps_,
                       cv::Size(width_, height_), true);

    if (!video_writer_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video file for writing.");
        return false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Started recording to %s", filename.c_str());
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
    if (recording_) {
        {
            std::lock_guard<std::mutex> lock(writer_mutex_);
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
    try {
        transform_stamped =
            tf_buffer_->lookupTransform(map_frame_, camera_frame_id_, tf2::TimePointZero);
        rclcpp::Time transform_time(transform_stamped.header.stamp);
        if (now() - transform_time > rclcpp::Duration::from_seconds(0.5)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Camera TF is older than 0.5 seconds");
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Could not transform: %s", ex.what());
        return;
    }

    // TODO make this a service arg
    int camera_id = 1;
    std::string image_name = "image_" + std::to_string(written_frame_count_) + ".png";
    // Write the pose to the pose file
    if (pose_file_.is_open()) {
        pose_file_ << written_frame_count_ << " ";
        pose_file_ << transform_stamped.transform.rotation.w << " "
                   << transform_stamped.transform.rotation.x << " "
                   << transform_stamped.transform.rotation.y << " "
                   << transform_stamped.transform.rotation.z << " ";
        pose_file_ << transform_stamped.transform.translation.x << " "
                   << transform_stamped.transform.translation.y << " "
                   << transform_stamped.transform.translation.z << " ";
        pose_file_ << std::to_string(camera_id) << " " << image_name << "\n";
        pose_file_ << "\n"; // Leave blank line between frames
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

        if (stop_writer_thread_)
            break;
        rate.sleep();
    }
}

std::string OpencvCamNode::get_time_str() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::gmtime(&now_time);
    std::stringstream ss;
    ss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)