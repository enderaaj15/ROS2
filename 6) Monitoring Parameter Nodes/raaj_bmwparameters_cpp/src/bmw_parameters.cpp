#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

using std::placeholders::_1;

class BuyCarParamNode : public rclcpp::Node
{
public:
    BuyCarParamNode()
        : Node("buycar_param_node")
    {
        RCLCPP_INFO(this->get_logger(), "BMW is ready to be bought. What is your budget? Choose your car model based on your budget. Then choose a bank and loan duration and we will calculate the monthly loan repayment based on the bank interest rates");

        // Full BMW catalog with prices (RM)
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

        bank_interest_ = {
            {"maybank", 0.035},
            {"cimb", 0.037},
            {"public_bank", 0.036},
            {"rhb", 0.038}
        };

        // Declare parameters
        this->declare_parameter<int>("downpayment", 0);
        this->declare_parameter<int>("budget", 200000);
        this->declare_parameter<std::string>("car_model", "BMW 116i");
        this->declare_parameter<std::string>("car_color", "white");
        this->declare_parameter<int>("loan_years", 5);
        this->declare_parameter<std::string>("bank", "rhb");

        // Parameter callback
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BuyCarParamNode::parameter_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Waiting for parameters to be set...");

        // Startup check
        startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&BuyCarParamNode::startup_check, this));

        // Repeating evaluation every 5s
        eval_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&BuyCarParamNode::evaluate_purchase, this));
    }

private:
    std::map<std::string, int> car_catalog_;
    std::vector<std::string> allowed_colors_;
    std::map<std::string, double> bank_interest_;

    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr eval_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    
    bool valid_budget_ = true;
    bool valid_model_ = true;
    bool valid_years_ = true;
    bool valid_bank_ = true;


    // Parameter validation
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : params)
        {
            const auto &name = param.get_name();

            if (name == "budget")
            {
                int value = param.as_int();
                if (value < 120000)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Budget RM%d is too low (<RM120,000) BMW not cheap la. Rejecting parameter.", value);
                    valid_budget_ = false;
                    result.successful = false;
                    return result;
                }
                valid_budget_ = true;
                RCLCPP_INFO(this->get_logger(), "Budget set to RM%d", value);
                std::ostringstream models;
                models << "Models within budget: ";
                for (auto &[model, price] : car_catalog_)
                    if (price <= value)
                        models << model << ", ";
                RCLCPP_INFO(this->get_logger(), "%s", models.str().c_str());
            }
            else if (name == "car_model")
            {
                std::string model = param.as_string();
                if (car_catalog_.find(model) == car_catalog_.end())
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Model '%s' not in catalog. Rejecting parameter.", model.c_str());
                    valid_model_ = false;
                    result.successful = false;
                    return result;
                }
                valid_model_ = true;
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
                RCLCPP_INFO(this->get_logger(), "Color of your ride set to %s..", color.c_str());
            }
            else if (name == "loan_years")
            {
                int years = param.as_int();
                if (years < 5 || years > 9)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Loan years must be between 5 and 9. Rejecting parameter.");
                    valid_years_ = false;
                    result.successful = false;
                    return result;
                }
                valid_years_ = true;
                RCLCPP_INFO(this->get_logger(), "Loan repayment set to %d years.", years);
            }
            else if (name == "bank")
            {
                std::string bank = param.as_string();
                std::transform(bank.begin(), bank.end(), bank.begin(), ::tolower);
                if (bank_interest_.find(bank) == bank_interest_.end())
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Bank '%s' not recognized. Choose from maybank, cimb, public_bank, rhb.", bank.c_str());
                    valid_bank_ = false;
                    result.successful = false;
                    return result;
                }
                valid_bank_ = true;
                RCLCPP_INFO(this->get_logger(), "🏦️ Bank set to %s.", bank.c_str());
            }
            else if (name == "downpayment")
            {
                int down = param.as_int();
                if (down < 0)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Downpayment must be positive. Rejecting parameter.");
                    result.successful = false;
                    return result;
                }
                RCLCPP_INFO(this->get_logger(), "Downpayment set to RM%d.", down);
            }
        }

        return result;
    }

    void startup_check()
    {
        evaluate_purchase();
        startup_timer_->cancel();
    }

    // Repeatedly evaluates and prints
    void evaluate_purchase()
    {
        // Skip individually if any parameter invalid
        if (!valid_budget_) {
            RCLCPP_WARN(this->get_logger(), "❌ Skipping Calc: Budget invalid (<RM120,000) BMW not cheap laaa...🤦‍♂️️");
            return;
        }
        if (!valid_model_) {
            RCLCPP_WARN(this->get_logger(), "❌ Skipping Calc: Car model invalid. Choose model available in the catalog within your budget");
            return;
        }
        if (!valid_years_) {
            RCLCPP_WARN(this->get_logger(), "❌ Skipping Calc: Loan years invalid. (Must be between 5 and 9).");
            return;
        }
        if (!valid_bank_) {
            RCLCPP_WARN(this->get_logger(), "❌ Skipping Calc: Bank invalid. (Choose from maybank, cimb, public_bank, rhb).");
            return;
        }
        
        int budget = this->get_parameter("budget").as_int();
        std::string model = this->get_parameter("car_model").as_string();
        std::string color = this->get_parameter("car_color").as_string();
        int years = this->get_parameter("loan_years").as_int();
        std::string bank = this->get_parameter("bank").as_string();
        int downpayment = this->get_parameter("downpayment").as_int();

        std::transform(color.begin(), color.end(), color.begin(), ::tolower);
        std::transform(bank.begin(), bank.end(), bank.begin(), ::tolower);

        if (car_catalog_.find(model) == car_catalog_.end())
            return;

        int price = car_catalog_[model];

        // Error and Warning msg

        if (downpayment >= price)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Downpayment RM%d must be less than car price RM%d. So rich want to buy it in cash? 💰️💸️", downpayment, price);
            return;
        }

        if (price > budget)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Model '%s' costs RM%d, which exceeds budget RM%d. Ukur baju di badan sendiri 🙃️", model.c_str(), price, budget);
            return;
        }

        double rate = bank_interest_[bank];
        double monthly_rate = rate / 12.0;
        int months = years * 12;
        double loan_amount = price - downpayment;
        double monthly_payment = (loan_amount * monthly_rate) /
                                 (1 - std::pow(1 + monthly_rate, -months));

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << monthly_payment;

        RCLCPP_INFO(this->get_logger(),
                    "Monthly repayment for your %s %s over %d years via %s: RM%s",
                    color.c_str(), model.c_str(), years, bank.c_str(), oss.str().c_str());
        RCLCPP_INFO(this->get_logger(),
                    "🎉 Congratulations! You are now the proud owner of a %s %s!",
                    color.c_str(), model.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuyCarParamNode>());
    rclcpp::shutdown();
    return 0;
}

