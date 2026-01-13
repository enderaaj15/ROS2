#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

namespace comp_raaj_bmwshowroom_srvcli
{

class BuyCarClient : public rclcpp::Node
{
public:
    explicit BuyCarClient(const rclcpp::NodeOptions & options)
    : Node("buycar_client", options)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 BuyCar Client started.");

        // Create service client
        client_ = this->create_client<std_srvs::srv::Trigger>("buycar_service");

        // Subscribe to topic updates
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "buycar_summary",
            10,
            std::bind(&BuyCarClient::topic_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(2s, std::bind(&BuyCarClient::try_call_service, this));
    }

private:
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool called_ = false;

    void try_call_service()
    {
        if (!client_->service_is_ready()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "⏳ Waiting for buycar_service...");
            return;
        }

        if (called_) {
            timer_->cancel();
            return;
        }

        called_ = true;
        RCLCPP_INFO(this->get_logger(), "📞 buycar_service is ready, sending request...");

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // Non-blocking async call — no spin_until_future_complete()
        client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
            {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "✅ Service response:\n%s", response->message.c_str());
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "❌ Service call failed: %s", e.what());
                }
            });
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "📢 Seller's message:\n%s", msg->data.c_str());
    }
};

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(comp_raaj_bmwshowroom_srvcli::BuyCarClient)

