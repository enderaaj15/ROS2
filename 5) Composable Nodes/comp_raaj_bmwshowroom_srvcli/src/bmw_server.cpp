#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

using std::placeholders::_1;
using std::placeholders::_2;

namespace comp_raaj_bmwshowroom_srvcli
{

    class BuyCarServer : public rclcpp::Node
    {
    public:
        explicit BuyCarServer(const rclcpp::NodeOptions & options) : Node("buycar_server", options)
        {
            RCLCPP_INFO(this->get_logger(), "🚗 BMW server started. Ready to take your dream car order.");

            // BMW catalog and allowed values
            car_catalog_ = {
                {"BMW 118i", 150000}, {"BMW 116i", 120000},
                {"BMW 218i Active Tourer", 180000}, {"BMW X1 sDrive18i", 190000},
                {"BMW 218i Gran Coupe Sport", 241000}, {"BMW 320i Sport", 265800},
                {"BMW X3 20 xDrive M Sport", 320800}, {"BMW 530i M Sport", 399800},
                {"BMW 330i M Sport", 340200}, {"BMW 2 Series Gran Coupe", 250000},
                {"BMW X1", 260000}, {"BMW 3 Series Sedan", 300000},
                {"BMW X4", 380000}, {"BMW Z4", 400000}, {"BMW i4", 430000},
                {"BMW i5", 460000}, {"BMW X5", 480000}, {"BMW X6", 550000},
                {"BMW M340i xDrive", 580000}, {"BMW 420i Coupe", 255000},
                {"BMW X2", 280000}, {"BMW 330Li M Sport", 320000}
            };

            allowed_colors_ = {"black", "white", "blue", "silver"};
            bank_interest_ = {{"maybank", 0.035}, {"cimb", 0.037}, {"public_bank", 0.036}, {"rhb", 0.038}};

            // Declare parameters
            this->declare_parameter<int>("budget", 200000);
            this->declare_parameter<int>("downpayment", 0);
            this->declare_parameter<int>("loan_years", 5);
            this->declare_parameter<std::string>("car_model", "BMW 116i");
            this->declare_parameter<std::string>("car_color", "white");
            this->declare_parameter<std::string>("bank", "rhb");

            // Parameter validation
            callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&BuyCarServer::param_callback, this, _1));

            // Service
            service_ = this->create_service<std_srvs::srv::Trigger>(
                "buycar_service",
                std::bind(&BuyCarServer::handle_service, this, _1, _2));

            // Publisher
            publisher_ = this->create_publisher<std_msgs::msg::String>("buycar_summary", 10);

            // Periodic evaluation
            eval_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&BuyCarServer::evaluate_purchase, this));
        }

    private:
        std::map<std::string, int> car_catalog_;
        std::vector<std::string> allowed_colors_;
        std::map<std::string, double> bank_interest_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        rclcpp::TimerBase::SharedPtr eval_timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        // Parameter callback with full validation
        rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params)
        {
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;

            for (const auto &param : params)
            {
                const auto &name = param.get_name();

                if (name == "budget")
                {
                    int budget = param.as_int();
                    if (budget < 120000)
                    {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Budget RM%d is too low (<RM120,000) BMW not cheap la. Please la higher budget abit.", budget);
                        result.successful = false;
                        return result;
                    }

                    std::ostringstream models;
                    models << "Models within budget: ";
                    for (auto &[model, price] : car_catalog_)
                        if (price <= budget)
                            models << model << ", ";
                    RCLCPP_INFO(this->get_logger(), "%s", models.str().c_str());
                }
                else if (name == "car_model")
                {
                    std::string model = param.as_string();
                    if (car_catalog_.find(model) == car_catalog_.end())
                    {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Model '%s' not in catalog. Choose proper model la.", model.c_str());
                        result.successful = false;
                        return result;
                    }
                    RCLCPP_INFO(this->get_logger(), "%s chosen. Sweet choice!", model.c_str());
                }
                else if (name == "car_color")
                {
                    std::string color = param.as_string();
                    std::transform(color.begin(), color.end(), color.begin(), ::tolower);
                    if (std::find(allowed_colors_.begin(), allowed_colors_.end(), color) == allowed_colors_.end())
                    {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Color '%s' not allowed. Choose from black, white, blue, silver.", color.c_str());
                        result.successful = false;
                        return result;
                    }
                    RCLCPP_INFO(this->get_logger(), "Color chosen for your car is %s.", color.c_str());
                }
                else if (name == "loan_years")
                {
                    int years = param.as_int();
                    if (years < 5 || years > 9)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Loan years must be between 5 and 9 ok.");
                        result.successful = false;
                        return result;
                    }
                    RCLCPP_INFO(this->get_logger(), "Loan years set to %d.", years);
                }
                else if (name == "bank")
                {
                    std::string bank = param.as_string();
                    std::transform(bank.begin(), bank.end(), bank.begin(), ::tolower);
                    if (bank_interest_.find(bank) == bank_interest_.end())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Bank '%s' not available. Choose from maybank, cimb, public_bank, rhb.", bank.c_str());
                        result.successful = false;
                        return result;
                    }
                    RCLCPP_INFO(this->get_logger(), "Bank set to %s.", bank.c_str());
                }
                else if (name == "downpayment")
                {
                    int down = param.as_int();
                    if (down < 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Downpayment must be positive. Rejecting.");
                        result.successful = false;
                        return result;
                    }
                    RCLCPP_INFO(this->get_logger(), "Downpayment set to RM%d.", down);
                }
            }

            // Publish summary automatically after valid param set
            auto msg = std_msgs::msg::String();
            msg.data = get_repayment_summary();
            publisher_->publish(msg);

            return result;
        }

        void handle_service(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            std::string summary = get_repayment_summary();
            response->success = true;
            response->message = summary;

            auto msg = std_msgs::msg::String();
            msg.data = summary;
            publisher_->publish(msg);
        }

        void evaluate_purchase()
        {
            std::string summary = get_repayment_summary();
            RCLCPP_INFO(this->get_logger(), "%s", summary.c_str());

            auto msg = std_msgs::msg::String();
            msg.data = summary;
            publisher_->publish(msg);
        }

        std::string get_repayment_summary()
        {
            int budget = this->get_parameter("budget").as_int();
            int down = this->get_parameter("downpayment").as_int();
            int years = this->get_parameter("loan_years").as_int();
            std::string color = this->get_parameter("car_color").as_string();
            std::string model = this->get_parameter("car_model").as_string();
            std::string bank = this->get_parameter("bank").as_string();

            if (car_catalog_.find(model) == car_catalog_.end()) return "❌ Invalid car model.";
            if (bank_interest_.find(bank) == bank_interest_.end()) return "❌ Invalid bank.";
            if (std::find(allowed_colors_.begin(), allowed_colors_.end(), color) == allowed_colors_.end())
                return "❌ Invalid color.";

            int price = car_catalog_[model];

            if (down >= price)
            {
                std::ostringstream err;
                err << "⚠️ Downpayment RM" << down << " must be less than car price RM" << price << ". Rejecting calculation.";
                return err.str();
            }
            if (budget < price)
            {
                std::ostringstream err;
                err << "⚠️ Model " << model << " (RM" << price << ") exceeds budget RM" << budget;
                return err.str();
            }

            double rate = bank_interest_[bank];
            double monthly_rate = rate / 12.0;
            int months = years * 12;
            double loan_amount = price - down;
            double monthly_payment = (loan_amount * monthly_rate) /
                                     (1 - std::pow(1 + monthly_rate, -months));

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            oss << "Monthly repayment for your " << color << " " << model << " over " << years
                << " years via " << bank << ": RM" << monthly_payment
                << "\n🎉 Happy? If yes, you are now the proud owner of a BMW!!!";
            return oss.str();
        }
    };
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(comp_raaj_bmwshowroom_srvcli::BuyCarServer)

