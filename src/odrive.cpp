#include "ros_odrive/odrive.hpp"

using namespace std;

std::string LEFT_AXIS = "axis0";
std::string RIGHT_AXIS = "axis1";

Json::Value odrive_json;
bool targetJsonValid = false;
odrive_endpoint *endpoint = NULL;

// Args
int encoder_click_per_rotate;
float base_width;
float wheel_radius;

// Calculated values
float wheel_circum;
float encoder_click_per_meter;
double coeff;

ros::Time current_time, last_time;

double raw_wheel_L_ang_pos;
double raw_wheel_R_ang_pos;

double wheel_L_ang_vel;
double wheel_R_ang_vel;
double wheel_L_ang_pos;
double wheel_R_ang_pos;
double robot_angular_vel;
double robot_angular_pos;
double robot_x_vel;
double robot_y_vel;
double robot_x_pos;
double robot_y_pos;

sensor_msgs::JointState joint_states;

//creating the arrays for the message
vector<double> pos(2, 0.0);
vector<double> vel(2, 0.0);
vector<double> eff(2, 0.0);

#define ODOM_COV 0.005

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


float readWheelEncoder(string axis){
    float fval;
    readOdriveData(endpoint, odrive_json,
                   axis.append(".encoder.pos_estimate"), fval);
    return fval;
}

float readRightWheelEncoder()
{
    return readWheelEncoder(RIGHT_AXIS);
}

float readLeftWheelEncoder()
{
    return readWheelEncoder(LEFT_AXIS);
}


double getAngularPos(std::string axis){
    return coeff * readWheelEncoder(axis);
}

/**
 *
 * Publishe odrive message to ROS
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
    msg.pos0 = readLeftWheelEncoder();
    msg.pos1 = readRightWheelEncoder();

    // Publish message
    odrive_pub.publish(msg);

    return msg;
}


void sendOdometry(tf::TransformBroadcaster odom_broadcaster, ros::Publisher odometry_pub){

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_angular_pos);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "ros_odrive/odom";
    odom_trans.child_frame_id = "base_link";//footprint";

    odom_trans.transform.translation.x = robot_x_pos;
    odom_trans.transform.translation.y = robot_y_pos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "ros_odrive/odom";

    //set the position
    odom.pose.pose.position.x = robot_x_pos;
    odom.pose.pose.position.y = robot_y_pos;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = ODOM_COV;
    odom.pose.covariance[7] = ODOM_COV;
    odom.pose.covariance[14] = ODOM_COV;
    odom.pose.covariance[21] = ODOM_COV;
    odom.pose.covariance[28] = ODOM_COV;
    odom.pose.covariance[35] = ODOM_COV;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = robot_x_vel;
    odom.twist.twist.linear.y = robot_y_vel;
    odom.twist.twist.angular.z = robot_angular_vel;

    //publish the message
    odometry_pub.publish(odom);
}


void publishJointState(ros::Publisher joint_state_pub){
	joint_states.position = pos;
	joint_states.header.stamp = current_time;
	joint_state_pub.publish(joint_states);
}

void resetOdometry(){
    ROS_INFO("Reset odometry");
    raw_wheel_L_ang_pos = getAngularPos(LEFT_AXIS);
    raw_wheel_R_ang_pos = getAngularPos(RIGHT_AXIS);

    wheel_L_ang_pos = 0.0;
    wheel_R_ang_pos = 0.0;

    wheel_R_ang_vel = 0.0;
    wheel_L_ang_vel = 0.0;
    robot_angular_vel = 0.0;
    robot_angular_pos = 0.0;
    robot_x_vel = 0.0;
    robot_y_vel = 0.0;
    robot_x_pos = 0.0;
    robot_y_pos = 0.0;
};

void publishOdometry(ros::Publisher odometry_pub, const ros_odrive::odrive_msg odrive_msg,
                     tf::TransformBroadcaster odom_broadcaster, const ros::Time current_time,
                     const ros::Time last_time) {

    double curr_wheel_L_ang_pos = getAngularPos(LEFT_AXIS);
    double curr_wheel_R_ang_pos = getAngularPos(RIGHT_AXIS);
    double dtime = (current_time - last_time).toSec();

    double modificator = 2 * M_PI;

    double delta_L_ang_pos = curr_wheel_L_ang_pos - raw_wheel_L_ang_pos;
    double delta_R_ang_pos = -1.0 * (curr_wheel_R_ang_pos - raw_wheel_R_ang_pos);

    delta_L_ang_pos = delta_L_ang_pos;
    delta_R_ang_pos = delta_R_ang_pos;

    raw_wheel_L_ang_pos = curr_wheel_L_ang_pos;
    raw_wheel_R_ang_pos = curr_wheel_R_ang_pos;

    wheel_L_ang_vel = delta_L_ang_pos / (dtime);
    wheel_R_ang_vel = delta_R_ang_pos / (dtime);

    wheel_L_ang_pos = wheel_L_ang_pos + delta_L_ang_pos;
    wheel_R_ang_pos = wheel_R_ang_pos + delta_R_ang_pos;


    robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / base_width) - robot_angular_pos) / dtime;
    robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / base_width;

    robot_x_vel = ((wheel_L_ang_vel * wheel_radius + robot_angular_vel * (base_width / 2.0)) * cos(robot_angular_pos));
    robot_y_vel = ((wheel_L_ang_vel * wheel_radius + robot_angular_vel * (base_width / 2.0)) * sin(robot_angular_pos));

    robot_x_pos = robot_x_pos + robot_x_vel * dtime;
    robot_y_pos = robot_y_pos + robot_y_vel * dtime;

    // send odometry
    sendOdometry(odom_broadcaster, odometry_pub);

    pos[0] = wheel_L_ang_pos;
    pos[1] = wheel_R_ang_pos;

    vel[0] = wheel_L_ang_vel;
    vel[1] = wheel_R_ang_vel;

    //Torque [N.m] = 8.27 * Current [A] / KV.
    float fval;
    readOdriveData(endpoint, odrive_json, string("axis0.motor.current_control.Iq_measured"), fval);
    eff[0] = 8.27 * fval / 16;
    readOdriveData(endpoint, odrive_json, string("axis1.motor.current_control.Iq_measured"), fval);
    eff[1] = 8.27 * fval / 16;

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

    float right = -1.0 * encoder_click_per_meter * vr;
    float left = encoder_click_per_meter * vl;

    cmd = "axis1.controller.vel_setpoint";
    writeOdriveData(endpoint, odrive_json,
                    cmd, right);

    cmd = "axis0.controller.vel_setpoint";
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

void updateWatchDog(){
    // update watchdog
    execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
    execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");
}

void setPID(float p, float i){

    writeOdriveData(endpoint, odrive_json,
                    "axis0.controller.config.vel_gain", p);
    writeOdriveData(endpoint, odrive_json,
                    "axis0.controller.config.vel_integrator_gain", i);

    writeOdriveData(endpoint, odrive_json,
                    "axis1.controller.config.vel_gain", p);
    writeOdriveData(endpoint, odrive_json,
                    "axis1.controller.config.vel_integrator_gain", i);
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

    coeff = 2 * M_PI / encoder_click_per_rotate;

    encoder_click_per_meter = encoder_click_per_rotate / wheel_circum;

    ros::Publisher odrive_pub = nh.advertise<ros_odrive::odrive_msg>("odrive_msg_" + od_sn, 10);

    ros::Publisher odrive_odometry = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

	joint_states.header.frame_id = "base_link";

    //assigning the arrays to the message
    vector<std::string> joints_name(2, "");
    joints_name[0]= "left_wheel_hinge";
    joints_name[1]= "right_wheel_hinge";

    joint_states.name = joints_name;

    joint_states.position = pos;
    joint_states.velocity = vel;
    joint_states.effort = eff;

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

    setPID(0.04, 0.15);

    resetOdometry();

    // Example loop - reading values and updating motor velocity
    ROS_INFO("Starting idle loop");
    int diagnostics_barrier = 0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (ros::ok()) {
        // Publish status message
        ros_odrive::odrive_msg odrive_msg = publishMessage(odrive_pub);
        // Publish odometry message
        current_time = ros::Time::now();
        publishOdometry(odrive_odometry, odrive_msg, odom_broadcaster, current_time, last_time);
        publishJointState(joint_state_pub);
        last_time = current_time;

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
