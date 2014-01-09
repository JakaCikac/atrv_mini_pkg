#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ATRVmini_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <stdio.h>
#include <iostream>
#include <string>


using namespace std;

/**
 *  ATRVmini node for ROS - Jaka Cikac 2013
 *  Modified from: B21_Node By David Lu!! 2/2010
 */
class ATRVminiNode {
    private:
        ATRVmini driver;

        ros::Subscriber subs[4];			///< Subscriber handles (cmd_vel, cmd_accel, cmd_sonar_power, cmd_brake_power)
        ros::Publisher base_sonar_pub;		///< Sonar Publisher for Base Sonars (sonar_cloud_base)
        ros::Publisher body_sonar_pub;		///< Sonar Publisher for Body Sonars (sonar_cloud_body)
        ros::Publisher voltage_pub;			///< Voltage Publisher (voltage)
        ros::Publisher brake_power_pub;		///< Brake Power Publisher (brake_power)
        ros::Publisher sonar_power_pub;		///< Sonar Power Publisher (sonar_power)
        ros::Publisher odom_pub;			///< Odometry Publisher (odom)
        ros::Publisher plugged_pub;			///< Plugged In Publisher (plugged_in)
        ros::Publisher joint_pub; ///< Joint State Publisher (state)
        ros::Publisher bump_pub; ///< Bump Publisher (bumps)
        tf::TransformBroadcaster broadcaster; ///< Transform Broadcaster (for odom)

        bool isSonarOn, isBrakeOn;
        float acceleration;
        float last_distance, last_bearing, last_tvel, last_rvel;
        float x_odo, y_odo, a_odo;
        float cmdTranslation, cmdRotation;
        bool brake_dirty, sonar_dirty;
        bool initialized;
        float first_bearing;
        int updateTimer;
        int prev_bumps;
        bool sonar_just_on;

        void publishOdometry();
        void publishSonar();
        void publishBumps();

    public:
        ros::NodeHandle n;
        ATRVminiNode();
        ~ATRVminiNode();
        int initialize(const char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand      (const geometry_msgs::Twist::ConstPtr& msg);
        void SetAcceleration (const std_msgs::Float32   ::ConstPtr& msg);
        void ToggleSonarPower(const std_msgs::Bool      ::ConstPtr& msg);
        void ToggleBrakePower(const std_msgs::Bool      ::ConstPtr& msg);
};

ATRVminiNode::ATRVminiNode() : n ("~") {
    isSonarOn = isBrakeOn = false;
    brake_dirty = sonar_dirty = false;
    sonar_just_on = false;
    cmdTranslation = cmdRotation = 0.0;
    updateTimer = 99;
    initialized = false;
    prev_bumps = 0;
    subs[0] = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1,   &ATRVminiNode::NewCommand, this);
    subs[1] = n.subscribe<std_msgs::Float32>("cmd_accel", 1,     &ATRVminiNode::SetAcceleration, this);
    subs[2] = n.subscribe<std_msgs::Bool>("cmd_sonar_power", 1, &ATRVminiNode::ToggleSonarPower, this);
    subs[3] = n.subscribe<std_msgs::Bool>("cmd_brake_power", 1, &ATRVminiNode::ToggleBrakePower, this);
    acceleration = 0.7;

    base_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_base", 50);
    body_sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud_body", 50);
    sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
    brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1); 
    voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    plugged_pub = n.advertise<std_msgs::Bool>("plugged_in", 1);
    joint_pub = n.advertise<sensor_msgs::JointState>("state", 1);
    bump_pub = n.advertise<sensor_msgs::PointCloud>("bump", 5);
}

int ATRVminiNode::initialize(const char* port) {
    int ret = driver.initialize(port);
    if (ret < 0)
        return ret;

    driver.setOdometryPeriod (100000);
    driver.setDigitalIoPeriod(100000);
    driver.motionSetDefaults();
    return 0;
}

ATRVminiNode::~ATRVminiNode() {
    driver.motionSetDefaults();
    driver.setOdometryPeriod(0);
    driver.setDigitalIoPeriod(0);
    driver.setSonarPower(false);
    driver.setIrPower(false);
}

/// cmd_vel callback
void ATRVminiNode::NewCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    cmdTranslation = msg->linear.x;
    cmdRotation = msg->angular.z;
}

/// cmd_acceleration callback
void ATRVminiNode::SetAcceleration (const std_msgs::Float32::ConstPtr& msg) {
    acceleration = msg->data;
}

/// cmd_sonar_power callback
void ATRVminiNode::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
    isSonarOn=msg->data;
    sonar_dirty = true;
}

/// cmd_brake_power callback
void ATRVminiNode::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    isBrakeOn = msg->data;
    brake_dirty = true;
}

void ATRVminiNode::spinOnce() {
    // Sending the status command too often overwhelms the driver
    if (updateTimer>=100) {
        driver.sendSystemStatusCommand();
        updateTimer = 0;
    }
    updateTimer++;

    if (cmdTranslation != 0 || cmdRotation != 0)
        driver.setMovement(cmdTranslation, cmdRotation, acceleration);

    if (sonar_dirty) {
        driver.setSonarPower(isSonarOn);
        sonar_dirty = false;
        driver.sendSystemStatusCommand();
    }
    if (brake_dirty) {
        driver.setBrakePower(isBrakeOn);
        brake_dirty = false;
        updateTimer = 99;
    }

    std_msgs::Bool bmsg;
    bmsg.data = isSonarOn;
    sonar_power_pub.publish(bmsg);
    bmsg.data = driver.getBrakePower();
    brake_power_pub.publish(bmsg);
    bmsg.data = driver.isPluggedIn();
    plugged_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = driver.getVoltage();
    voltage_pub.publish(vmsg);

    publishOdometry();
    publishSonar();
    publishBumps();
}

/** Integrates over the lastest raw odometry readings from
 * the driver to get x, y and theta */
void ATRVminiNode::publishOdometry() {
    if (!driver.isOdomReady()) {
        return;
    }
	
    float distance = driver.getDistance();
    //ROS_INFO("Distance: %f", distance);
    float true_bearing = angles::normalize_angle(driver.getBearing());
   // ROS_INFO("True bearing: %f", true_bearing);
    
    if (!initialized) {
        initialized = true;
        first_bearing = true_bearing;
		x_odo = 0;
		y_odo = 0;
        a_odo = 0*true_bearing;
    } else {
        float bearing = true_bearing - first_bearing;
        float d_dist = distance-last_distance;
        float d_bearing = bearing - last_bearing;

		// we give the robot odometry a 2.4 meter margin if any errors arise
        if (d_dist > 1.2 || d_dist < -1.2) {
			//printf("atrv_distance %f\n", distance);
				return;
        }
		//printf("atrv_distance_out %f\n", distance);
		
        a_odo += d_bearing;
        a_odo = angles::normalize_angle(a_odo);

        //integrate latest motion into odometry
		//printf("d_dist: %f\n", d_dist);
        x_odo += d_dist * cos(a_odo);
        y_odo += d_dist * sin(a_odo);
    }
	
    last_distance = distance;
    last_bearing = true_bearing - first_bearing;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(last_bearing);
	
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base";

    odom_trans.transform.translation.x = x_odo;
    odom_trans.transform.translation.y = y_odo;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x_odo;
    odom.pose.pose.position.y = y_odo;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base";
    float tvel = driver.getTranslationalVelocity();
	float d_tvel = tvel-last_tvel; 
	if (d_tvel > 1.0 || d_tvel < -1.0) { //1.2 worked quite ok
			return;
    }
	last_tvel = tvel;
    odom.twist.twist.linear.x = tvel*cos(a_odo);
    odom.twist.twist.linear.y = tvel*sin(a_odo);
	float rvel = driver.getRotationalVelocity();
    odom.twist.twist.angular.z = rvel;
	float d_rvel = rvel-last_rvel; 
	if (d_rvel > 1.0 || d_rvel < -1.0) { //1.2 worked quite ok
			return;
    }

	last_rvel = rvel;
	//printf("data %f %f %f %f %f\n", tvel, rvel, distance, d_rvel, d_tvel);

    //publish the messages
    odom_pub.publish(odom);

    // finally, publish the joint state
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.name[0] = "joint_twist";
    joint_state.position[0] = true_bearing;

    joint_pub.publish(joint_state);

}

void ATRVminiNode::publishSonar() {
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "base";

    if (isSonarOn) {
        driver.getBaseSonarPoints(&cloud);
        base_sonar_pub.publish(cloud);

        driver.getBodySonarPoints(&cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub.publish(cloud);

    } else if (sonar_just_on) {
        base_sonar_pub.publish(cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub.publish(cloud);
    }
}

void ATRVminiNode::publishBumps() {
    sensor_msgs::PointCloud cloud1, cloud2;
    cloud1.header.stamp = ros::Time::now();
    cloud2.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "base";
    cloud2.header.frame_id = "body";
    int bumps = driver.getBaseBumps(&cloud1) +
                driver.getBodyBumps(&cloud2);

    if (bumps>0 || prev_bumps>0) {
        bump_pub.publish(cloud1);
        bump_pub.publish(cloud2);
    }
    prev_bumps = bumps;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ATRVmini_node");
    ATRVminiNode node;
    std::string port;
    node.n.param<std::string>("port", port, "/dev/ttyUSB0");
    ROS_INFO("Attempting to connect to %s", port.c_str());
    if (node.initialize(port.c_str())<0) {
        ROS_ERROR("Could not initialize RFLEX driver!\n");
        return 0;
    }
    ROS_INFO("Connected!");


    int hz;
    node.n.param("rate", hz, 50);
    ros::Rate loop_rate(hz);

    while (ros::ok()) {
        node.spinOnce();
        // Process a round of subscription messages
        ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}