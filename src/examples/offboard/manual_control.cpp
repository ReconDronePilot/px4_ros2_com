
/**
 * @brief Offboard control example
 * @file manual_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_air_data.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

float vehicle_alt;
float vehicle_home_alt;
bool home_set;
bool manual_control = false;
bool armed = false;
bool takeoffed = false;
float alt_err;
float speed  = 3.0;

std::array<float, 3> max_velocity;
px4_msgs::msg::ManualControlSetpoint cmd_setpoint;
sensor_msgs::msg::Joy::SharedPtr joy_msg_;
geometry_msgs::msg::Twist::SharedPtr twist_msg_;


int MANUAL_MODE = 1;
int ALT_CTRL_MODE = 2;
int POSCTL_MODE = 3;
int AUTO_MODE = 4;
int ACRO_MODE = 5;
int OFFBOARD_MODE = 6;
int STABILIZED_MODE = 7;
int RATTITUDE_MODE = 8;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		//---Variables---//
		vehicle_alt = 0.0;
		vehicle_home_alt = 0.0;
		home_set = false;
		alt_err = 1;
		max_velocity = {0.2, 0.2, 0.2};
		offboard_setpoint_counter_ = 0;
		manual_control = false;

		//---Publishers----------------------------------------------//
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		manual_control_publisher_ = this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input",10);            

		//---Subscribers---------------------------------------------//
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vehicle_gps_subscriber_ = this->create_subscription<SensorGps>("/fmu/out/vehicle_gps_position", qos,[this](const SensorGps::UniquePtr msg) {
			if(!home_set){ 
				vehicle_home_alt = msg->altitude_msl_m;
				home_set = true;
			}
			vehicle_alt = msg->altitude_msl_m;
			//RCLCPP_INFO(this->get_logger(), "Altitude : %f",msg.altitude_msl_m);
		});

		vehicle_command_ack_subscriber_ = this->create_subscription<VehicleCommandAck>("/fmu/out/vehicle_command_ack",qos,std::bind(&OffboardControl::cmdAckCallback, this, std::placeholders::_1));

		joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&OffboardControl::joyCallback, this, std::placeholders::_1));

		twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&OffboardControl::twistCallback, this, std::placeholders::_1));


		//---Main Program------------------------------------------------------------------------------------------------------//
		auto timer_callback = [this]() -> void {

			if((offboard_setpoint_counter_ %10)==0){ //RCLCPP_INFO(this->get_logger(),"Altitude : %f",vehicle_alt); 
			}

			if(joy_msg_ != nullptr){
				//---Manual Control---//
				// Square button for Arm and then takeoff
				if(joy_msg_->buttons[3]==1) {
				//if(offboard_setpoint_counter_ == 10) {
					// Change to Offboard mode after 10 setpoints
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, OFFBOARD_MODE);
					// Arm the vehicle
					this->arm();
					armed = true;
				}
				// Round button for disarm
				if(joy_msg_->buttons[1]==1) {
					// disarm the vehicle
					this->disarm();
					armed = false;
				}
				// Triangle button for change mode to manual control
				if (joy_msg_->buttons[2]==1){
					manual_control = true;
				}
				// X for takeoff
				if (joy_msg_->buttons[0]==1){
					takeoffed = true;
				}
			}
			if(armed==true) {
				this->publish_offboard_control_mode();
			}
			if(armed==true && manual_control == false && takeoffed==true){
				// Takeoff
				this->publish_trajectory_setpoint(0.0, 0.0, 5.0, 3.14/2);
			}
			if(armed==true && manual_control==true){	
				// Manual control
				this->publish_trajectory_velocity(
					twist_msg_->linear.x * speed,
					twist_msg_->linear.y * speed,
					twist_msg_->linear.z * speed,
					twist_msg_->angular.z * speed
					);
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback); // call le the loop every 100ms
	}

	void arm();
	void disarm();

private:

	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
		joy_msg_ = joy_msg;
        // Print status of the controller
		//RCLCPP_INFO(get_logger(), "Controller pad_up: %d", joy_msg->buttons[11]);
    }

	void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
		twist_msg_ = twist_msg;
        // Print status of the controller
		//RCLCPP_INFO(get_logger(), "CMD_VEL lin_x :%f	lin_y :%f	lin_z :%f	angular_z :%f", twist_msg->linear.x,twist_msg->linear.y,twist_msg->linear.z,twist_msg->angular.z);
    }
	
	void cmdAckCallback(const VehicleCommandAck::SharedPtr cmd_ack)
	{
		// Result of commands received
		RCLCPP_INFO(get_logger(), "Command ack: %u	Result: %d", cmd_ack->command, cmd_ack->result);
	}
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_control_publisher_;

	rclcpp::Subscription<SensorGps>::SharedPtr vehicle_gps_subscriber_;
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw);
	void publish_trajectory_velocity(float x_vel, float y_vel, float z_vel, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_manual_control_setpoint(float x_vel, float y_vel, float z_vel);
	void orbit(float radius, float vel);
	void takeoff();
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to takeoff
 */
void OffboardControl::takeoff()
{
	//publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0);
	// Création du message de commande de décollage
    VehicleCommand msg{};
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    msg.param1 = 0.0; // Ignoré pour le décollage
    msg.param2 = 0.0; // Ignoré pour le décollage
    msg.param3 = 0.0; // Ignoré pour le décollage
    msg.param4 = 0.0; // Ignoré pour le décollage
    msg.param5 = 0.0; // Ignoré pour le décollage
    msg.param6 = 0.0; // Ignoré pour le décollage
    msg.param7 = 500.0; // Altitude de décollage en mètres
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
	vehicle_command_publisher_->publish(msg);

	RCLCPP_INFO(this->get_logger(), "Takeoff ...");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

// /**
//  * @brief Publish the altitude control mode.
//  */
// void OffboardControl::publish_altitude_control_mode()
// {
// 	OffboardControlMode msg{};
// 	msg.position = true;
// 	msg.velocity = false;
// 	msg.acceleration = false;
// 	msg.attitude = false;
// 	msg.body_rate = false;
// 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
// 	offboard_control_mode_publisher_->publish(msg);
// }

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x,y,-z};
	msg.yaw = yaw; 
	msg.velocity = max_velocity;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_velocity(float x_vel, float y_vel, float z_vel, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {std::nan(""), std::nan(""),std::nan("")};
	msg.velocity = { x_vel, -y_vel, -z_vel };
	msg.yaw = -yaw; 
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_manual_control_setpoint(float x_vel, float y_vel, float z_vel)
{
	ManualControlSetpoint msg{};
	msg.valid = true;
	msg.throttle = z_vel;
	msg.roll = x_vel;
	msg.pitch = y_vel;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	manual_control_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Send a command to orbit
 */
void OffboardControl::orbit(float radius, float vel)
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_ORBIT, radius, vel);

	RCLCPP_INFO(this->get_logger(), "Orbit starting ...");
}


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}