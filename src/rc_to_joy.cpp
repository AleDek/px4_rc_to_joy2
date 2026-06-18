#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>

using namespace std;
using namespace std::chrono_literals;

class RcToJoy : public rclcpp::Node
{
public:
    RcToJoy();
    void run();

private:
    void rc_cb(const px4_msgs::msg::RcChannels::SharedPtr data);
    void data_publisher();
    void load_params();
    double to_unit_axes(double pwm);
    double to_unit_buttons(double pwm);

    rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rc_sub;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr _joy_pub;

    std::mutex _mtx;
    bool _first_rc;
    int _n_ch;
    std::vector<double> _pwm;
    std::vector<double> _old_pwm;

    // --- parameters ---
    int _N_axes;
    int _N_buttons;
    std::vector<int64_t> _map_to_axes;        // Use int64_t for ROS2 integer parameters
    std::vector<double> _axes_dir;
    std::vector<int64_t> _map_to_button;      // Use int64_t for ROS2 integer parameters
    std::vector<double> _button_dir;
    std::vector<double> _button_nstates;
    double _pwm_max;
    double _pwm_min;
    double _pwm_deadzone;
    double _pwm_mid;
    double _pwm_delta;
    double _pwm_range;
};

void RcToJoy::load_params()
{
    // Declare parameters with their default values
    this->declare_parameter("pwm_max", 1.0);
    this->declare_parameter("pwm_min", -1.0);
    this->declare_parameter("pwm_deadzone", 1e-3);
    this->declare_parameter("ch_to_axes", std::vector<int64_t>({1, 2, 3, 4}));
    this->declare_parameter("reverse_axes", std::vector<double>({1., 1., 1., 1.}));
    this->declare_parameter("ch_to_buttons", std::vector<int64_t>({5, 6}));
    this->declare_parameter("reverse_buttons", std::vector<double>({1., 1.}));
    this->declare_parameter("nstates_buttons", std::vector<double>({2., 2.}));

    // Get parameters from the server (or use defaults)
    _pwm_max = this->get_parameter("pwm_max").as_double();
    _pwm_min = this->get_parameter("pwm_min").as_double();
    _pwm_deadzone = this->get_parameter("pwm_deadzone").as_double();
    _map_to_axes = this->get_parameter("ch_to_axes").as_integer_array();
    _axes_dir = this->get_parameter("reverse_axes").as_double_array();
    _map_to_button = this->get_parameter("ch_to_buttons").as_integer_array();
    _button_dir = this->get_parameter("reverse_buttons").as_double_array();
    _button_nstates = this->get_parameter("nstates_buttons").as_double_array();

    // Your existing calculations and logic
    _pwm_mid = (_pwm_max + _pwm_min) / 2.0;
    _pwm_delta = (_pwm_max - _pwm_min) / 2.0;
    _pwm_range = (_pwm_max - _pwm_min);

    _N_axes = _map_to_axes.size();
    _N_buttons = _map_to_button.size();

    // Parameter validation and logging
    if (_axes_dir.size() != _N_axes)
    {
        _axes_dir.resize(_N_axes);
        for (int i = 0; i < _N_axes; i++)
            _axes_dir[i] = 1.0;
    }
    std::stringstream ss;
    for (int i = 0; i < _map_to_axes.size(); i++)
    {
        ss << _axes_dir[i] * _map_to_axes[i] << " ";
    }
    RCLCPP_INFO(this->get_logger(), "axes are channels [ %s]", ss.str().c_str());

    if (_button_dir.size() != _N_buttons)
    {
        _button_dir.resize(_N_buttons);
        for (int i = 0; i < _N_buttons; i++)
            _button_dir[i] = 1.0;
    }
    if (_button_nstates.size() != _N_buttons)
    {
        _button_nstates.resize(_N_buttons);
        for (int i = 0; i < _N_buttons; i++)
            _button_nstates[i] = 2.0;
    }

    std::stringstream ss2, ss3;
    for (int i = 0; i < _map_to_button.size(); i++)
    {
        ss2 << _button_dir[i] * _map_to_button[i] << " ";
        ss3 << _button_nstates[i] << " ";
    }
    RCLCPP_INFO(this->get_logger(), "buttons are channels [ %s]", ss2.str().c_str());
    RCLCPP_INFO(this->get_logger(), "buttons have nstates [ %s]", ss3.str().c_str());
}

RcToJoy::RcToJoy() : Node("rc_to_joy")
{   
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    _rc_sub = this->create_subscription<px4_msgs::msg::RcChannels>(
        "/fmu/out/rc_channels", qos,
        std::bind(&RcToJoy::rc_cb, this, std::placeholders::_1));
    _joy_pub = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    _first_rc = false;
    _n_ch = 0;

    load_params();
}

void RcToJoy::rc_cb(const px4_msgs::msg::RcChannels::SharedPtr data)
{
    _first_rc = true;
    _n_ch = data->channels.size();
    _pwm.resize(_n_ch);
    _mtx.lock();
    for (int i = 0; i < _n_ch; i++)
    {
        _pwm[i] = data->channels[i];
    }
    _mtx.unlock();
}

// double RcToJoy::to_unit_axes(double pwm)
// {
//     if (pwm > _pwm_max)
//         pwm = _pwm_max;
//     else if (pwm < _pwm_min)
//         pwm = _pwm_min;
//     else if (pwm < _pwm_mid + _pwm_deadzone && pwm > _pwm_mid - _pwm_deadzone)
//         pwm = _pwm_mid;

//     return (pwm - _pwm_mid) / _pwm_delta; //[-1,1]
// }

// double RcToJoy::to_unit_buttons(double pwm)
// {
//     if (pwm > _pwm_max)
//         pwm = _pwm_max;
//     else if (pwm < _pwm_min)
//         pwm = _pwm_min;

//     return (pwm - _pwm_min) / _pwm_range; //[0,1]
// }

double RcToJoy::to_unit_axes(double pwm)
{
    return pwm; //[-1,1]
}

double RcToJoy::to_unit_buttons(double pwm)
{
    
    return (pwm + 1.0) / 2.0; //[0,1]
}

void RcToJoy::data_publisher()
{
    rclcpp::Rate rate(50.0);
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.frame_id = "px4_radio";
    joy_msg.axes.resize(_N_axes);
    joy_msg.buttons.resize(_N_buttons);
    bool change = false;

    while (rclcpp::ok())
    {
        change = false;
        _mtx.lock();
        for (int i = 0; i < _n_ch; i++)
        {
            if (fabs(_pwm[i] - _old_pwm[i]) > _pwm_deadzone)
            {
                change = true;
                _old_pwm[i] = _pwm[i];
            }
        }
        _mtx.unlock();
        if (change)
        {
            joy_msg.header.stamp = this->now();
            for (int i = 0; i < _N_axes; i++)
            {
                // joy_msg.axes[i] = _axes_dir[i] * to_unit_axes(_ol_pwm[_map_to_axes[i] - 1]);
                joy_msg.axes[i] = _axes_dir[i] * _old_pwm[_map_to_axes[i] -1];
            }
            for (int i = 0; i < _N_buttons; i++)
            {
                if (_button_dir[i] > 0.0)
                    joy_msg.buttons[i] = round((_button_nstates[i] - 1.0) * to_unit_buttons(_old_pwm[_map_to_button[i] - 1]));
                else
                    joy_msg.buttons[i] = round((_button_nstates[i] - 1.0) * (1.0 - to_unit_buttons(_old_pwm[_map_to_button[i] - 1])));
            }
            _joy_pub->publish(joy_msg);
        }
        rate.sleep();
    }
}

void RcToJoy::run()
{
    rclcpp::Rate rate(1.0);
    RCLCPP_INFO(this->get_logger(), "Waiting first rc...");
    while (!_first_rc)
    {
        rclcpp::spin_some(this->shared_from_this());
        rate.sleep();
    }
    _old_pwm.resize(_n_ch);
    for (int i = 0; i < _n_ch; i++)
        _old_pwm[i] = 0;
    RCLCPP_INFO(this->get_logger(), "First rc arrived");

    std::thread sp_pub_t(&RcToJoy::data_publisher, this);
    sp_pub_t.detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RcToJoy>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}