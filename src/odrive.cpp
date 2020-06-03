#include "ros_odrive/odrive.hpp"

using namespace std;

Json::Value odrive_json;
bool targetJsonValid = false;
odrive_endpoint *endpoint = NULL;

int encoder_click_per_rotate;

float base_width;
float wheel_radius;
float wheel_circum;
float encoder_click_per_meter;

float right_pos;
float left_pos;

ros::Time current_time, last_time;

float global_x;
float global_y;
float global_theta;

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

void publishOdometry(ros::Publisher odometry_pub, const ros_odrive::odrive_msg odrive_msg,
                     tf::TransformBroadcaster odom_broadcaster, const ros::Time current_time,
                     const ros::Time last_time) {
    float curr_tick_right = odrive_msg.pos1;
    float curr_tick_left = odrive_msg.pos0;

    float curr_right_pos = curr_tick_right - right_pos;
    float curr_left_pos = -1.0 * (curr_tick_left - left_pos);

    right_pos = curr_tick_right;
    left_pos = curr_tick_left;

    float delta_right_wheel_in_meter = curr_right_pos / encoder_click_per_meter;
    float delta_left_wheel_in_meter = curr_left_pos / encoder_click_per_meter;

    float local_theta = (delta_right_wheel_in_meter - delta_left_wheel_in_meter) / base_width;

    float distance = (delta_right_wheel_in_meter + delta_left_wheel_in_meter) / 2;

    ros::Duration ros_time_elapsed = current_time - last_time;
    float time_elapsed = ros_time_elapsed.toSec();

    float local_x = cos(global_theta) * distance;
    float local_y = -sin(global_theta) * distance;

    global_x = global_x + (cos(global_theta) * local_x - sin(global_theta) * local_y);
    global_y = global_y + (sin(global_theta) * local_x + cos(global_theta) * local_y);

    global_theta += local_theta;

    //global_theta = math.atan2(math.sin(global_theta), math.cos(global_theta));

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, global_theta);

    ros::Time now_time = ros::Time::now();

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(global_x, global_y, 0.0));
    transform.setRotation(quaternion);

    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));

    nav_msgs::Odometry odom;
    odom.header.stamp = now_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = global_x;
    odom.pose.pose.position.y = global_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(global_theta);
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = distance / time_elapsed;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = local_theta / time_elapsed;
    odometry_pub.publish(odom);

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

    float right = encoder_click_per_meter * vr;
    float left = -1.0 * encoder_click_per_meter * vl;

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
    nh.param("encoder_click_per_rotate", encoder_click_per_rotate, 90);

    wheel_circum = 2.0 * wheel_radius * M_PI;
    encoder_click_per_meter = encoder_click_per_rotate / wheel_circum;

    ros::Publisher odrive_pub = nh.advertise<ros_odrive::odrive_msg>("odrive_msg_" + od_sn, 100);

    ros::Publisher odrive_odometry = nh.advertise<nav_msgs::Odometry>("odometry", 100);

    ros::Subscriber odrive_sub = nh.subscribe("odrive_ctrl", 10, msgCallback);

    ros::Subscriber odrive_cmd_vel = nh.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

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

    ROS_INFO("Init odometry");
    //TODO : SOLID function
    readOdriveData(endpoint, odrive_json,
                   string("axis1.encoder.pos_estimate"), right_pos);;
    readOdriveData(endpoint, odrive_json,
                   string("axis0.encoder.pos_estimate"), left_pos);;
    ROS_INFO("Start pos : Axis0 : %f , Axis1 : %f", left_pos, right_pos);

    float _temp_p_val = 0.02;
    float _temp_i_val = 0.15;

    writeOdriveData(endpoint, odrive_json,
                    "axis0.controller.config.vel_gain", _temp_p_val);
    writeOdriveData(endpoint, odrive_json,
                    "axis0.controller.config.vel_integrator_gain", _temp_i_val);

    writeOdriveData(endpoint, odrive_json,
                    "axis1.controller.config.vel_gain", _temp_p_val);
    writeOdriveData(endpoint, odrive_json,
                    "axis1.controller.config.vel_integrator_gain", _temp_i_val);

    //TODO : read from params
    global_x = 0;
    global_y = 0;
    global_theta = 0;


    // Example loop - reading values and updating motor velocity
    ROS_INFO("Starting idle loop");
    int diagnostics_barrier = 0;
    while (ros::ok()) {
        // Publish status message
        current_time = ros::Time::now();
        publishMessage(odrive_pub);
        //publishOdometry(odrive_odometry, , odom_broadcaster, current_time, last_time);
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
