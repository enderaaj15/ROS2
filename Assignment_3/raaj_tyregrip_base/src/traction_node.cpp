#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_loader.hpp>
#include <raaj_tyregrip_base/traction.hpp>
#include <cmath>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"

class TractionNode : public rclcpp::Node
{
public:
  TractionNode() : Node("traction_node")
  {
    // --- Parameter Descriptors ---
    rcl_interfaces::msg::ParameterDescriptor speed_desc;
    speed_desc.name = "speed";
    speed_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 300.0;
    range.step = 10.0;
    speed_desc.floating_point_range.push_back(range);

    // Declare parameters and initialize member variables
    current_weather_name_ = this->declare_parameter<std::string>("weather_plugin", "raaj_tyregrip_plugins::DryWeather");
    current_tyre_name_ = this->declare_parameter<std::string>("tyre_plugin", "raaj_tyregrip_plugins::UHPTyre");
    current_mass_ = this->declare_parameter<double>("mass", 1500.0);
    current_speed_ = this->declare_parameter("speed", 80.0, speed_desc);

    // Plugin loaders
    weather_loader_ = std::make_shared<pluginlib::ClassLoader<raaj_tyregrip_base::Weather>>(
      "raaj_tyregrip_plugins", "raaj_tyregrip_base::Weather");
    tyre_loader_ = std::make_shared<pluginlib::ClassLoader<raaj_tyregrip_base::Tyre>>(
      "raaj_tyregrip_plugins", "raaj_tyregrip_base::Tyre");

    // Load initial plugins
    load_plugins();

    // Watch for parameter changes
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TractionNode::param_callback, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool should_reload_plugins = false;

    for (const auto &param : params) {
      if (param.get_name() == "weather_plugin") {
        current_weather_name_ = param.as_string();
        should_reload_plugins = true;
      } else if (param.get_name() == "tyre_plugin") {
        current_tyre_name_ = param.as_string();
        should_reload_plugins = true;
      } else if (param.get_name() == "mass") {
        current_mass_ = param.as_double();
        should_reload_plugins = true;
      } else if (param.get_name() == "speed") {
        current_speed_ = param.as_double();
        should_reload_plugins = true;
      }
    }

    if (should_reload_plugins) {
        load_plugins();
    }
    
    return result;
  }

  void load_plugins()
  {
    try {
      auto weather_name = current_weather_name_;
      auto tyre_name = current_tyre_name_;
      double mass = current_mass_;
      double speed = current_speed_;

      weather_ = weather_loader_->createSharedInstance(weather_name);
      tyre_ = tyre_loader_->createSharedInstance(tyre_name);

      double mu_weather = weather_->get_multiplier(speed);
      double mu_tyre = tyre_->get_multiplier(speed, weather_->get_type());
      double mu_total = mu_weather * mu_tyre;
      double traction = mu_total * mass * 9.81;

      RCLCPP_INFO(this->get_logger(),
        "✅ Reloaded plugins -> Weather: %s | Tyre: %s | Mass: %.1f kg",
        weather_name.c_str(), tyre_name.c_str(), mass);

      RCLCPP_INFO(this->get_logger(),
        "📊 Speed=%.1f km/h | μ=%.2f | Traction Force=%.1f N",
        speed, mu_total, traction);

      if (mu_total >= 0.8)
        RCLCPP_INFO(this->get_logger(), "✅ SAFE! Plenty of grip");
      else if (mu_total >= 0.5)
        RCLCPP_WARN(this->get_logger(), "⚠️ WARNING! Grip starting to drop");
      else
        RCLCPP_ERROR(this->get_logger(), "❌ CRITICAL DANGER! Grip almost gone");
    }
    catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load plugin: %s", ex.what());
    }
  }

  std::shared_ptr<pluginlib::ClassLoader<raaj_tyregrip_base::Weather>> weather_loader_;
  std::shared_ptr<pluginlib::ClassLoader<raaj_tyregrip_base::Tyre>> tyre_loader_;
  std::shared_ptr<raaj_tyregrip_base::Weather> weather_;
  std::shared_ptr<raaj_tyregrip_base::Tyre> tyre_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  
  //MEMBER VARIABLES TO CACHE CURRENT PARAMETER VALUES
  std::string current_weather_name_;
  std::string current_tyre_name_;
  double current_mass_;
  double current_speed_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TractionNode>());
  rclcpp::shutdown();
  return 0;
}
