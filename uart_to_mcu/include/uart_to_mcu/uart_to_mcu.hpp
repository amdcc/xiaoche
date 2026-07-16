#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace uart_to_mcu
{
inline constexpr const char * kDefaultWheelSpeedsTopic = "/wheel_speeds";
inline constexpr const char * kDefaultGoalReachedTopic = "/goal_reached";
inline constexpr const char * kDefaultGoalPublishedTopic = "/goal_published";

// 通过 ROS 话题接收循迹端的车速 / 结束事件,打包成统一格式经串口下发给下位机。
// 帧格式(共 10 字节): 0x5A | speed_left(i32,LE) | speed_right(i32,LE) | 0xA5
// 无包 ID:每一帧都是"设定左右轮速",(0,0) 即停车。
inline constexpr uint8_t kFrameHead = 0x5AU;
inline constexpr uint8_t kFrameTail = 0xA5U;
class UartToMcuNode : public rclcpp::Node
{
public:
	UartToMcuNode();
	~UartToMcuNode() override;

private:
	void wheel_speeds_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
	void goal_reached_callback(const std_msgs::msg::Bool::SharedPtr msg);
	void goal_published_callback(const std_msgs::msg::Bool::SharedPtr msg);

	bool open_serial();
	void close_serial();

	// 组装一帧数据包(帧头 + 左右轮速 + 帧尾)。
	std::vector<uint8_t> build_packet(int32_t speed_left, int32_t speed_right) const;
	bool write_packet(const std::vector<uint8_t> & packet);

	// 将 /wheel_speeds 的一路 RPS 乘以缩放系数后钳到 int32 范围。
	int32_t scale_to_i32(double rps) const;

	static void append_i32_le(std::vector<uint8_t> & out, int32_t value);

	std::string wheel_speeds_topic_;
	std::string goal_reached_topic_;
	std::string goal_published_topic_;
	std::string serial_port_;
	double wheel_speed_scale_;

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_published_sub_;

	int serial_fd_;
};
}  // namespace uart_to_mcu
