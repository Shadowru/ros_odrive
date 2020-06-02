#include "ros_odrive/odrive.hpp"

using namespace std;

Json::Value odrive_json;
bool targetJsonValid = false;
odrive_endpoint *endpoint = NULL;

float base_width;
float wheel_radius;

float left_encoder;
float right_encoder;

ros::Time current_time, last_time;

double global_x;
double global_y;
double global_th;

void msgCallback(const ros_odrive::odrive_ctrl::ConstPtr &msg) {
    std::string cmd;
    uint8_t u8val;
    uint16_t u16val;
    float fval;

    if (msg->axis == 0) {
        cmd = "axis0";
    } else if (msg->axis == 1) {
        cmd = "axis1";
    } else {
        ROS_ERROR("Invalid axis value in message!");
        return;
    }

    switch (msg->command) {
        case (CMD_AXIS_RESET):
            // Reset errors
            u16val = u8val = 0;
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".motor.error"), u16val);
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".encoder.error"), u8val);
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".controller.error"), u8val);
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".error"), u16val);
            break;
        case (CMD_AXIS_IDLE):
            // Set channel to Idle
            u8val = AXIS_STATE_IDLE;
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".requested_state"), u8val);
            break;
        case (CMD_AXIS_CLOSED_LOOP):
            // Enable Closed Loop Control
            u8val = AXIS_STATE_CLOSED_LOOP_CONTROL;
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".requested_state"), u8val);
            break;
        case (CMD_AXIS_SET_VELOCITY):
            // Set velocity
            fval = msg->fval;
            writeOdriveData(endpoint, odrive_json,
                            cmd.append(".controller.vel_setpoint"), fval);
            break;
        case (CMD_REBOOT):
            execOdriveFunc(endpoint, odrive_json, string("reboot"));
            break;
        default:
            ROS_ERROR("Invalid command type in message!");
            return;
    }
}

/**
 *
 * Publise odrive message to ROS
 * @param endpoint odrive enumarated endpoint
 * @param odrive_json target json
 * @param odrive_pub ROS publisher
 * return ODRIVE_OK in success
 *
 */
ros_odrive::odrive_msg publishMessage(ros::Publisher odrive_pub) {
    uint16_t u16val;
    uint8_t u8val;
    float fval;
    ros_odrive::odrive_msg msg;

    readOdriveData(endpoint, odrive_json, string("axis0.error"), u16val);
    msg.error0 = u16val;
    readOdriveData(endpoint, odrive_json, string("axis1.error"), u16val);
    msg.error1 = u16val;
    readOdriveData(endpoint, odrive_json, string("axis0.current_state"), u8val);
    msg.state0 = u8val;
    readOdriveData(endpoint, odrive_json, string("axis1.current_state"), u8val);
    msg.state1 = u8val;
    readOdriveData(endpoint, odrive_json,
                   string("axis0.encoder.pos_estimate"), fval);
    msg.pos0 = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis1.encoder.pos_estimate"), fval);
    msg.pos1 = fval;

    // Publish message
    odrive_pub.publish(msg);

    return msg;
}

void velCallback(const geometry_msgs::Twist &vel) {

    std::string cmd;
    uint8_t u8val;
    uint16_t u16val;

    float v = vel.linear.x;
    float w = vel.angular.z;

    // m per sec
    float vr = ((2.0 * v) + (w * base_width)) / (2.0 * wheel_radius);
    float vl = ((2.0 * v) + (-1.0 * w * base_width)) / (2.0 * wheel_radius);

    float right = 90 * vr;
    float left = 90 * vl;

    cmd = "axis0.controller.vel_setpoint";
    writeOdriveData(endpoint, odrive_json,
                    cmd, right);

    cmd = "axis1.controller.vel_setpoint";
    writeOdriveData(endpoint, odrive_json,
                    cmd, left);
}

void odrive_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    float fval;

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Odrive nominal");

    readOdriveData(endpoint, odrive_json, string("vbus_voltage"), fval);
    stat.add("Voltage", fval);

    readOdriveData(endpoint, odrive_json, string("axis0.motor.current_control.Iq_measured"), fval);
    stat.add("Axis 0 Current", fval);

    readOdriveData(endpoint, odrive_json, string("axis1.motor.current_control.Iq_measured"), fval);
    stat.add("Axis 1 Current", fval);

    execOdriveGetTemp(endpoint, odrive_json,
                      string("axis0.motor.get_inverter_temp"), fval);
    stat.add("Axis 0 temperature", fval);

    execOdriveGetTemp(endpoint, odrive_json,
                      string("axis1.motor.get_inverter_temp"), fval);
    stat.add("Axis 0 temperature", fval);
}

/**
 *
 * Node main function
 *
 */
int main(int argc, char **argv) {
    std::string od_sn;
    std::string od_cfg;
    int rate;

    ROS_INFO("Starting ODrive...");

    // Initialize ROS node
    ros::init(argc, argv, "ros_odrive"); // Initializes Node Name
    ros::NodeHandle nh("~");

    nh.param("rate", rate, 10);
    ros::Rate r(10);

    nh.param<std::string>("od_sn", od_sn, "0x00000000");

    nh.param<std::string>("od_cfg", od_cfg, "");

    // Get device serial number
    if (nh.getParam("od_sn", od_sn)) {
        ROS_INFO("Node odrive S/N: %s", od_sn.c_str());
    } else {
        ROS_ERROR("Failed to get sn parameter %s!", od_sn.c_str());
        return 1;
    }

    //test robot width
    nh.param<float>("base_width", base_width, 0.58);
    nh.param<float>("wheel_radius", wheel_radius, 0.235 / 2);

    ros::Publisher odrive_pub = nh.advertise<ros_odrive::odrive_msg>("odrive_msg_" + od_sn, 100);

    ros::Publisher odrive_odometry = nh.advertise<nav_msgs::Odometry>("odometry", 100);

    ros::Subscriber odrive_sub = nh.subscribe("odrive_ctrl", 10, msgCallback);

    ros::Subscriber odrive_cmd_vel = nh.subscribe("cmd_vel", 10, velCallback);

    diagnostic_updater::Updater odrive_diagnostics_updater;
    odrive_diagnostics_updater.setHardwareIDf("ODRIVE S/N: %s", od_sn.c_str());

    odrive_diagnostics_updater.add("ODRIVE", odrive_diagnostics);

    // Get odrive endpoint instance
    endpoint = new odrive_endpoint();

    // Enumerate Odrive target
    if (endpoint->init(stoull(od_sn, 0, 16))) {
        ROS_ERROR("Device not found!");
        return 1;
    }

    // Read JSON from target
    if (getJson(endpoint, &odrive_json)) {
        return 1;
    }
    targetJsonValid = true;

    // Process configuration file
    if (nh.searchParam("od_cfg", od_cfg)) {
        nh.getParam("od_cfg", od_cfg);
        ROS_INFO("Using configuration file: %s", od_cfg.c_str());

        updateTargetConfig(endpoint, odrive_json, od_cfg);
    }

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Example loop - reading values and updating motor velocity
    ROS_INFO("Starting idle loop");
    int diagnostics_barrier = 0;
    while (ros::ok()) {
        // Publish status message
        current_time = ros::Time::now();
        publishMessage(odrive_pub);
        last_time = current_time;

        // update watchdog
        //execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
        //execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");

        // idle loop
        r.sleep();
        ros::spinOnce();
        if (diagnostics_barrier++ > 10) {
            odrive_diagnostics_updater.update();
            diagnostics_barrier = 0;
        }
    }

    endpoint->remove();

    delete endpoint;

    return 0;
}
