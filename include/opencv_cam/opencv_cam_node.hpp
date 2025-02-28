#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP

#include <opencv2/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "messages_88/srv/record_video.hpp"

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
    rclcpp::Time next_stamp_;
    bool see3cam_flag_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_ir_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Service<messages_88::srv::RecordVideo>::SharedPtr record_service_;

    cv::VideoWriter video_writer_;
    cv::VideoWriter video_writer_ir_;

    bool recording_;

    std::mutex record_mutex_;

  public:

    explicit OpencvCamNode(const rclcpp::NodeOptions &options);

    ~OpencvCamNode() override;

    bool startRecording(const std::string &filename);
    bool stopRecording();

  private:

    void validate_parameters();
    bool SeparatingRGBIRBuffers(cv::Mat frame, cv::Mat* IRImageCU83, cv::Mat* RGBImageCU83, int *RGBBufferSizeCU83, int *IRBufferSizeCU83);

    bool recordVideoCallback(const std::shared_ptr<messages_88::srv::RecordVideo::Request> req,
      std::shared_ptr<messages_88::srv::RecordVideo::Response> resp);

    void loop();
  };

} // namespace opencv_cam

#endif //OPENCV_CAM_HPP
