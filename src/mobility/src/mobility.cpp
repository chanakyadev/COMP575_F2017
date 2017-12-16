#include <ros/ros.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"

#include <signal.h>
#include <math.h>
#include <vector>

//#define KP 0.5
#define KPL 1.8
#define KPG 2.7
//#define KPG 2.5
using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;



string rover_name;
char host[128];
bool is_published_name = false;


int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;
float angular=0;
float global_local_heading = 0;
float global_av_heading = 0;

float global_local = 0;
float global_average_heading = 0;
float position_average_heading = 0;

pose current_location;
float direction_theta=0;
float average_x=0.0;
float average_y=0.0;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
ros::Publisher posePublisher;
ros::Publisher globalAverageHeadingPublisher;
ros::Publisher localAverageHeadingPublisher;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber messageSubscriber;
ros::Subscriber poseSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); //
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);
void poseHandler(const std_msgs::String::ConstPtr &message);

string get_rover_name_from_message (string msg);
pose get_pose_from_message (string msg);
void parse_pose_message(string msg);
float calculate_global_average_heading();
float calculate_local_average_heading();
void calculate_neighbors(string rover_name);

void calculate_position_average_heading();

vector <pose> neighbors(6);
vector <pose> all_rovers(6);

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
    poseSubscriber = mNH.subscribe(("poses"), 10, poseHandler);

    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    posePublisher = mNH.advertise<std_msgs::String>(("poses"), 10, true);
    globalAverageHeadingPublisher = mNH.advertise<std_msgs::Float32>((rover_name + "/global_average_heading"), 1, true);
    localAverageHeadingPublisher = mNH.advertise<std_msgs::Float32>((rover_name + "/local_average_heading"), 1, true);

    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std_msgs::String state_machine_msg;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
            case STATE_MACHINE_TRANSLATE:
            {
                state_machine_msg.data = "TRANSLATING";//, " + converter.str();
                float angular_velocity = position_average_heading-current_location.theta;

                                //Correction for Homework 4
                                //angular=KPL*(global_local-current_location.theta);
                                //angular=KPG*(global_average_heading-current_location.theta);
                                float linear_velocity = 0;
                setVelocity(linear_velocity, angular_velocity);
                break;
            }
            default:
            {
                state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
                break;
            }
        }

    }
    else
    { // mode is NOT auto

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }
    std_msgs::String pose_message;
    std::stringstream converter;
    converter << rover_name << ", " << current_location.x << ", " << current_location.y << ", " << current_location.theta;
    pose_message.data = converter.str();
    posePublisher.publish(pose_message);

    stateMachinePublish.publish(state_machine_msg);
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
    // the rover's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();

    velocity.linear.x = 0;
    //velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
    // Only used if we want to take action after seeing an April Tag.
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void poseHandler(const std_msgs::String::ConstPtr& message)
{
    std_msgs::String output_message;
    string msg = message->data;

    parse_pose_message(msg);

    float gah = calculate_global_average_heading();

    calculate_neighbors(rover_name);
    float lah = calculate_local_average_heading();

    std::stringstream converter;
    converter << msg << ", " << rover_name << ", " << gah << ", " << lah << ", " << all_rovers[0].theta << ", " << all_rovers[1].theta << ", " << all_rovers[2].theta;
    output_message.data = converter.str();
    debug_publisher.publish(output_message);

    std_msgs::Float32 gah_message;
    std_msgs::Float32 lah_message;
    gah_message.data = gah;
    lah_message.data = lah;
    globalAverageHeadingPublisher.publish(gah_message);
    localAverageHeadingPublisher.publish(lah_message);
}

void calculate_position_averaging_heading(){
    float sum_x=0;
    float sum_y=0;
    float average_position_x=0;
    float average_position_y=0;
    float KP;
    for (int i=0;i<neighbors.size();i++){
        sum_x +=(neighbors[i].x-current_location.x);
    sum_y +=(neighbors[i].y-current_location.y);
    }
    if (neighbors.size()==0){
    average_position_x=current_location.x;
    average_position_y=current_location.y;
    }else{
    average_position_x=current_location.x+sum_x/neighbors.size();
    average_position_y=current_location.y+sum_y/neighbors.size();

    }
    position_average_heading=KP*atan2(average_position_y,average_position_x);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( message->data > 0 )
    {
        if (message->data == 1)
        {
            // obstacle on right side
        }
        else
        {
            //obstacle in front or on left side
        }
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    current_location.x = message->pose.pose.position.x;
    current_location.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulation_mode == 0 || simulation_mode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!is_published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        is_published_name = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.",
             current_time);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void messageHandler(const std_msgs::String::ConstPtr& message)
{

}

pose get_pose_from_message (string msg) {
    stringstream ss(msg);
    string incoming_rover_name;
    string pose_x;
    string pose_y;
    string pose_theta;
    pose incoming_pose;

    std::getline(ss, incoming_rover_name, ',');
    std::getline(ss, pose_x, ',');
    std::getline(ss, pose_y, ',');
    std::getline(ss, pose_theta, ',');

    istringstream pose_x_iss(pose_x);
    istringstream pose_y_iss(pose_y);
    istringstream pose_theta_iss(pose_theta);

    pose_x_iss >> incoming_pose.x;
    pose_y_iss >> incoming_pose.y;
    pose_theta_iss >> incoming_pose.theta;

    return incoming_pose;
}

string get_rover_name_from_message (string msg) {
    stringstream ss(msg);
    string incoming_rover_name;
    std::getline(ss, incoming_rover_name, ',');
    return incoming_rover_name;
}

void parse_pose_message(string msg){
    string incoming_rover_name;
    pose incoming_pose;

    incoming_rover_name = get_rover_name_from_message (msg);
    incoming_pose = get_pose_from_message(msg);

    if(incoming_rover_name.compare("ajax") == 0){
        all_rovers[0] = incoming_pose;
    } else if (incoming_rover_name.compare("aeneas") == 0){
        all_rovers[1] = incoming_pose;
    } else if (incoming_rover_name.compare("achilles") == 0){
        all_rovers[2] = incoming_pose;
    } else if (rover_name.compare("diomedes") == 0){
       all_rovers[3] = incoming_pose;
    }else if (rover_name.compare("hector") == 0){
       all_rovers[4] = incoming_pose;
    }else if (rover_name.compare("paris") == 0){
        all_rovers[5] = incoming_pose;
    } else {
        cout << "We missed something.";
    }
}

float calculate_global_average_heading(){
    float u_x=0;
    float u_y=0;
    float global_average_heading;
    float local_average_heading;
    global_average_heading =  local_average_heading;
    //global_av_heading= global_average_heading;
    for (int i = 0; i<6; i++){
        u_x += cos(all_rovers[i].theta);
        u_y += sin(all_rovers[i].theta);
    }
    global_average_heading = atan2(u_y,u_x);
    return global_average_heading;
 //   global_av_heading = atan2(u_y,u_x);
    return global_av_heading;
}

void calculate_neighbors(string rover_name){
    pose my_pose;
    int my_index=0;
    int my_index;
    if(rover_name.compare("ajax") == 0){
        my_pose = all_rovers[0];
        my_index = 0;
    } else if (rover_name.compare("aeneas") == 0){
        my_pose = all_rovers[1];
        my_index = 1;
    } else if (rover_name.compare("achilles") == 0){
        my_pose = all_rovers[2];
        my_index = 2;
    } else if (rover_name.compare("diomedes") == 0){
        my_pose = all_rovers[3];
        my_index = 3;
    }else if (rover_name.compare("hector") == 0){
        my_pose = all_rovers[4];
        my_index = 4;
    }else if (rover_name.compare("paris") == 0){
        my_pose = all_rovers[5];
        my_index = 5;
    }
    else {
        my_pose = all_rovers[0];
//        cout << "We missed something.";
    }
    neighbors.clear();
    for (int i = 0; i<6; i++){
        for (int i = 0; i<3; i++){
        if(i != my_index){
            if(hypot(my_pose.x-all_rovers[i].x, my_pose.y-all_rovers[i].y)<2){
                neighbors.push_back(all_rovers[i]);
                average_x += my_pose.x + 1/ neighbors.size() * (my_pose.x-all_rovers[i].x);
                average_y += my_pose.y + 1/ neighbors.size() * (my_pose.y-all_rovers[i].y);
            }

        }
    }
    direction_theta= atan2(average_y,average_x);
}

}
float calculate_local_average_heading(){
    float u_x=0;
    float u_y=0;
    float local_average_heading;
    //global_local_heading = local_average_heading;
    global_local = local_average_heading;
    for (int i = 0; i<neighbors.size(); i++){
        u_x += cos(neighbors[i].theta);
        u_y += sin(neighbors[i].theta);
    }
    global_local = atan2(u_y,u_x);
    return global_local;
}
