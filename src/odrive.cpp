#include "ros_odrive/odrive.hpp"

using namespace std;

Json::Value odrive_json;
bool targetJsonValid = false;
odrive_endpoint *endpoint = NULL;

float base_width;

float enc_left_last;
float enc_right_last;

ros::Time current_time, last_time;

double x;
double y;
double th;

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

int publishOdometry(ros::Publisher odrive_odometry, ros_odrive::odrive_msg status_message) {

    tf::TransformBroadcaster odom_broadcaster;

    float delta_enc_left = status_message.vel0 - enc_left_last;
    float delta_enc_right = status_message.vel1 - enc_right_last;

    double distance_local = (delta_enc_left + delta_enc_right) / 2;
    double th_local = (delta_enc_right - delta_enc_right) / base_width;

    //calculate velocities

    double dt = (current_time - last_time).toSec();

    double distance_m_per_s = distance_local / dt;
    double dr_per_s = th_local / dt;

    if(distance_local != 0){
        double x_local = cos(th_local) * distance_local;
        double y_local = -sin(th_local) * distance_local;
        x = x + (cos(th) * x_local - sin(th) * y_local);
        y = y + (sin(th) * x_local + cos(th) * y_local);
    }
    if(th != 0){
        th = th + th_local;
    }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = distance_m_per_s;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = dr_per_s;

    //publish the message
    odrive_odometry.publish(odom);
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

    // Collect data
    readOdriveData(endpoint, odrive_json, string("vbus_voltage"), fval);
    msg.vbus = fval;
    readOdriveData(endpoint, odrive_json, string("axis0.error"), u16val);
    msg.error0 = u16val;
    readOdriveData(endpoint, odrive_json, string("axis1.error"), u16val);
    msg.error1 = u16val;
    readOdriveData(endpoint, odrive_json, string("axis0.current_state"), u8val);
    msg.state0 = u8val;
    readOdriveData(endpoint, odrive_json, string("axis1.current_state"), u8val);
    msg.state1 = u8val;
    readOdriveData(endpoint, odrive_json,
                   string("axis0.encoder.vel_estimate"), fval);
    msg.vel0 = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis1.encoder.vel_estimate"), fval);
    msg.vel1 = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis0.encoder.pos_estimate"), fval);
    msg.pos0 = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis1.encoder.pos_estimate"), fval);
    msg.pos1 = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis0.motor.current_meas_phB"), fval);
    msg.curr0B = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis0.motor.current_meas_phC"), fval);
    msg.curr0C = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis1.motor.current_meas_phB"), fval);
    msg.curr1B = fval;
    readOdriveData(endpoint, odrive_json,
                   string("axis1.motor.current_meas_phC"), fval);
    msg.curr1C = fval;
    execOdriveGetTemp(endpoint, odrive_json,
                      string("axis0.motor.get_inverter_temp"), fval);
    msg.temp0 = fval;
    execOdriveGetTemp(endpoint, odrive_json,
                      string("axis1.motor.get_inverter_temp"), fval);
    msg.temp1 = fval;

    // Publish message
    odrive_pub.publish(msg);

    return msg;
}

void velCallback(const geometry_msgs::Twist &vel) {

    std::string cmd;
    uint8_t u8val;
    uint16_t u16val;
    float fval;
    float left;
    float right;
    float rotation_speed;
    const float linear_multiplier = 90 / 1.5;
    const float angle_multiplier = 90 / M_PI;
    float left_diff;
    float right_diff;

    //assume twist in m/s
    //10.5R wheel have 1.5 m per rotate

    fval = vel.linear.x * linear_multiplier;

    left = -fval;
    right = fval;

    //assume radians/s

    rotation_speed = vel.angular.z;

    left_diff = rotation_speed * angle_multiplier;
    right_diff = left_diff;

    left -= left_diff;
    right -= right_diff;

    cmd = "axis0.controller.vel_setpoint";
    writeOdriveData(endpoint, odrive_json,
                    cmd, right);

    cmd = "axis1.controller.vel_setpoint";
    writeOdriveData(endpoint, odrive_json,
                    cmd, left);
}

/**
 *
 * Node main function
 *
 */
int main(int argc, char **argv) {
    std::string od_sn;
    std::string od_cfg;

    ROS_INFO("Starting ODrive...");

    // Initialize ROS node
    ros::init(argc, argv, "ros_odrive"); // Initializes Node Name
    ros::NodeHandle nh("~");
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
    ros::Publisher odrive_pub = nh.advertise<ros_odrive::odrive_msg>("odrive_msg_" + od_sn, 100);
    ros::Publisher odrive_odometry = nh.advertise<ros_odrive::odrive_msg>("odometry", 100);
    ros::Subscriber odrive_sub = nh.subscribe("odrive_ctrl", 10, msgCallback);

    //
    ros::Subscriber odrive_cmd_vel = nh.subscribe("cmd_vel", 10, velCallback);

    // Get odrive endpoint instance
    endpoint = new odrive_endpoint();

    // Enumarate Odrive target
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

    enc_left_last = 0;
    enc_right_last = 0;

    //test robot width
    base_width = 0.58;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Example loop - reading values and updating motor velocity
    ROS_INFO("Starting idle loop");
    while (ros::ok()) {
        // Publish status message
        current_time = ros::Time::now();
        publishOdometry(odrive_odometry, publishMessage(odrive_pub));
        last_time = current_time;

        // update watchdog
        //execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
        //execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");

        // idle loop
        r.sleep();
        ros::spinOnce();
    }

    endpoint->remove();

    delete endpoint;

    return 0;
}
