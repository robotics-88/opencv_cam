#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP

#include <fstream>

#include <opencv2/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "messages_88/srv/record_video.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "opencv_cam/camera_context.hpp"

namespace opencv_cam
{

  class OpencvCamNode : public rclcpp::Node
  {
    CameraContext cxt_;

    std::thread thread_;
    std::atomic<bool> canceled_;

    std::shared_ptr<cv::VideoCapture> capture_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int publish_fps_;
    double device_fps_;
    rclcpp::Time next_stamp_;
    bool see3cam_flag_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_ir_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr meas_fps_pub_;
    rclcpp::Service<messages_88::srv::RecordVideo>::SharedPtr record_service_;

    rclcpp::TimerBase::SharedPtr meas_fps_timer_;
    rclcpp::TimerBase::SharedPtr record_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::VideoWriter video_writer_;
    cv::VideoWriter video_writer_ir_;

    cv::Mat last_frame_;
    cv::Mat last_ir_frame_;

    std::string map_frame_;
    std::ofstream pose_file_;
    std::atomic<int> frame_count_;
    std::atomic<int> written_frame_count_;
    int last_frame_count_;

    bool recording_;

    std::mutex frame_mutex_;

  public:

    explicit OpencvCamNode(const rclcpp::NodeOptions &options);

    ~OpencvCamNode() override;

    bool stopRecording();

  private:

    void validate_parameters();
    bool SeparatingRGBIRBuffers(cv::Mat frame, cv::Mat* IRImageCU83, cv::Mat* RGBImageCU83, int *RGBBufferSizeCU83, int *IRBufferSizeCU83);

    void handleSee3CamFrame(cv::Mat frame, rclcpp::Time stamp);
    void handleGenericFrame(cv::Mat frame, rclcpp::Time stamp);
    void writeToPoseFile();
    void writeVideo();

    bool recordVideoCallback(const std::shared_ptr<messages_88::srv::RecordVideo::Request> req,
      std::shared_ptr<messages_88::srv::RecordVideo::Response> resp);
    bool startRecording(const std::string &filename);
    
    void loop();
  };

} // namespace opencv_cam

#endif //OPENCV_CAM_HPP
