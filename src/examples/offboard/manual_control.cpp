
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
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;

float vehicle_alt;
float vehicle_home_alt;
bool home_set;
bool manual_control;
float alt_err;	
std::array<float, 3> max_velocity;
px4_msgs::msg::ManualControlSetpoint cmd_setpoint;
geometry_msgs::msg::Twist::SharedPtr cmd_vel_;

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

		//---Publishers---//
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		manual_control_publisher_ = this->create_publisher<ManualControlSetpoint>("/fmu/in/manual_control_input",10);            

		//---Subscribers---//
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

		cmd_vel_subscription_ = this->create_subscription<Twist>("cmd_vel", 10, std::bind(&OffboardControl::cmdVelCallback, this, std::placeholders::_1));

		//---Main Program------------------------------------------------------------------------------------------------------//
		auto timer_callback = [this]() -> void {

			if((offboard_setpoint_counter_ %10)==0){ RCLCPP_INFO(this->get_logger(),"Altitude : %f",vehicle_alt); }

			if(!manual_control){
				//---Manual Control---//
				if(offboard_setpoint_counter_ == 10) {
					// Change to Offboard mode after 10 setpoints
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					// Arm the vehicle
					this->arm();
				}
				if(offboard_setpoint_counter_<=100 ){
					this->publish_offboard_control_mode();
					// takeoff();
					this->publish_trajectory_setpoint(0.0, 0.0, 5.0, 3.14/2);
				}
				// if (offboard_setpoint_counter_ == 100 ){
				// 	// Change to manual control mode after 100 setpoints
				// 	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1); 
				// }
				if(offboard_setpoint_counter_ > 100){
						
					this->publish_manual_control_setpoint(cmd_vel_->linear.x, cmd_vel_->linear.y, cmd_vel_->linear.z);
				}
				//this->publish_offboard_control_mode();
			}else{
				

			}
			if (offboard_setpoint_counter_ < 500) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback); // call le the loop every 100ms
	}

	void arm();
	void disarm();

private:

	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
		cmd_vel_ = cmd_vel;
        // // Callback appelée à chaque fois qu'un message cmd_vel est reçu
        // RCLCPP_INFO(get_logger(), "Linear Velocity x: %f,Linear Velocity y: %f, Linear Velocity z: %f",
        //             cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->linear.z);
    }

	void cmdAckCallback(const VehicleCommandAck::SharedPtr cmd_ack)
	{
		RCLCPP_INFO(get_logger(), "Command ack: %u	Result: %d", cmd_ack->command, cmd_ack->result);
	}
	
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_control_publisher_;

	rclcpp::Subscription<SensorGps>::SharedPtr vehicle_gps_subscriber_;
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

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
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, -z};
	msg.yaw = yaw; 
	msg.velocity = max_velocity;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_velocity(float x_vel, float y_vel, float z_vel, float yaw)
{
	TrajectorySetpoint msg{};
	msg.velocity = {x_vel, y_vel, z_vel};
	msg.yaw = yaw;
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
