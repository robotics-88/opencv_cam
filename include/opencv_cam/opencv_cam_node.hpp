#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP

#include <fstream>
#include <queue>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/imgproc.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace opencv_cam {

class OpencvCamNode : public rclcpp::Node {
  public:
    explicit OpencvCamNode(const rclcpp::NodeOptions &options);

    ~OpencvCamNode() override;

    bool stopRecording();

  private:
    // Parameters
    bool file_;                    /* Read from file vs. read from device */
    std::string filename_;         /* Filename */
    int fps_;                      /* Desired frames per second */
    std::string device_;           /* Device name e.g. /dev/video0 */
    int width_;                    /* Device width */
    int height_;                   /* Device height */
    std::string camera_info_path_; /* Camera info path */
    std::string camera_frame_id_;  /* Camera frame id */

    std::thread thread_;
    std::atomic<bool> canceled_;

    std::shared_ptr<cv::VideoCapture> capture_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    std::string camera_name_;
    int publish_fps_;
    double device_fps_;
    rclcpp::Time next_stamp_;
    bool see3cam_flag_;

    // Fisheye
    cv::Mat map1_, map2_;
    bool fisheye_maps_initialized_ = false;
    cv::Mat K_fisheye_, D_fisheye_, R_fisheye_, P_fisheye_;
    cv::Size image_size_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temp_sub_;
    std::atomic<float> latest_temp_{NAN};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_ir_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        rectified_image_pub_; // for fisheye rectification
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr meas_fps_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_recording_sub_;

    rclcpp::TimerBase::SharedPtr meas_fps_timer_;
    rclcpp::TimerBase::SharedPtr record_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::VideoWriter video_writer_;
    cv::VideoWriter video_writer_ir_;

    std::string map_frame_;
    std::ofstream pose_file_;
    std::atomic<int> frame_count_;
    std::atomic<int> written_frame_count_;
    int last_frame_count_;

    bool recording_;
    std::thread writer_thread_;
    std::queue<cv::Mat> frame_queue_;
    bool stop_writer_thread_ = false;
    std::mutex writer_mutex_;

    void initFisheyeUndistortMaps();
    std::string get_time_str();

    void handleFrame(cv::Mat &frame, rclcpp::Time stamp);
    void writeToPoseFile();

    void triggerRecordingCallback(const std_msgs::msg::String::SharedPtr msg);
    bool startRecording(const std::string data_directory);

    void loop();
    void writerLoop();
};

} // namespace opencv_cam

#endif // OPENCV_CAM_HPP
