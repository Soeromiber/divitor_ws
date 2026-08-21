#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace divitor_perception {

class ThermalColormapperNode : public rclcpp::Node {
  public:
    ThermalColormapperNode(const rclcpp::NodeOptions &options);
    void declareAndFetchParameters();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_mapped_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double min_temp_;
    double max_temp_;
};

} // namespace divitor_perception
