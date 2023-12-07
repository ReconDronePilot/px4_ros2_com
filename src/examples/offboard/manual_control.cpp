
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
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_air_data.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

float vehicle_alt = 0.0;
float vehicle_home_alt = 0.0;
bool home_set = false;
float alt_err = 1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		//float vehicle_alt = 0.0;
		vehicle_gps_subscriber_ = this->create_subscription<SensorGps>("/fmu/out/vehicle_gps_position", qos,[this, &vehicle_alt, &home_set](const SensorGps::UniquePtr msg) {
			if(!home_set){ 
				vehicle_home_alt = msg->altitude_msl_m;
				home_set = true;
			}
			vehicle_alt = msg->altitude_msl_m;
			//RCLCPP_INFO(this->get_logger(), "Altitude : %f",msg.altitude_msl_m);
		});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this, &vehicle_alt, &home_set]() -> void {
			if((offboard_setpoint_counter_ %4)==0){ RCLCPP_INFO(this->get_logger(),"Altitude : %f",vehicle_alt); }

			if (offboard_setpoint_counter_ == 10) {
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

			if(offboard_setpoint_counter_>100 && offboard_setpoint_counter_<=200 ){
				this->publish_offboard_control_mode();
				// takeoff();
				this->publish_trajectory_setpoint(0.0, 0.0, 5.0, -3.14/2);
			}

			if(offboard_setpoint_counter_>200){
				this->publish_offboard_control_mode();
				// takeoff();
				this->publish_trajectory_setpoint(0.0, 0.0, -0.5, -3.14/2);
				if(vehicle_home_alt >= vehicle_alt-alt_err && vehicle_home_alt <= vehicle_alt+alt_err) { this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND); } // AJOUTER LAND()
			}			
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 500) {
				offboard_setpoint_counter_++;
			}

		};
		timer_ = this->create_wall_timer(100ms, timer_callback); // call le the loop every 100ms
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<SensorGps>::SharedPtr vehicle_gps_subscriber_;
	//rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr vehicle_air_data_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
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
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
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
