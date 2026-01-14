#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

class BMWMonitorParamNode : public rclcpp::Node
{
public:
  BMWMonitorParamNode()
  : Node("bmw_parameters_monitoring")
  {
    // Create a parameter subscriber for monitoring parameter changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    // The remote node whose parameters we want to monitor
    const std::string remote_node_name = "buycar_param_node";

    // Callback for parameter
    // String Parameters
    auto cb_a = [this](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received update: \"%s\" set to %s (type: %s)",
        p.get_name().c_str(),
        p.as_string().c_str(),
        p.get_type_name().c_str());
    };
    cb_handle_a_ = param_subscriber_->add_parameter_callback("car_model", cb_a, remote_node_name);
    cb_handle_b_ = param_subscriber_->add_parameter_callback("car_color", cb_a, remote_node_name);
    cb_handle_c_ = param_subscriber_->add_parameter_callback("bank", cb_a, remote_node_name);
    
    //Integer Parameter
    auto cb_b = [this](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received update: \"%s\" set to %ld (type: %s)",
        p.get_name().c_str(),
        p.as_int(),
        p.get_type_name().c_str());
    };
    cb_handle_d_ = param_subscriber_->add_parameter_callback("budget", cb_b, remote_node_name);
    cb_handle_e_ = param_subscriber_->add_parameter_callback("loan_years", cb_b, remote_node_name);
    cb_handle_f_ = param_subscriber_->add_parameter_callback("downpayment", cb_b, remote_node_name);


    RCLCPP_INFO(this->get_logger(), "Monitoring BMW parameters from '%s'...", remote_node_name.c_str());
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_a_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_b_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_c_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_d_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_e_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_f_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BMWMonitorParamNode>());
  rclcpp::shutdown();
  return 0;
}
