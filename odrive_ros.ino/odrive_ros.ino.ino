#include <HardwareSerial.h>
//ODrive Library
#include <ODriveArduino.h>

//ODrive Objects
ODriveArduino odrive1(Serial1);

//ROS Libraries
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>

// Define the robot parameters
#define mubot_width (420) //mm
#define wheel_diameter (165.1) //mm 
#define wheel_circumference (518.362788)//mm
#define encoder_cpr (90) //count per revolution
#define rev (5) //motor revolutions per wheel circunfrence

// ROS handle
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

// cmd_vel variables to drive the robot with
float request_x;
float request_z;

// set timers for the sub_main loop
unsigned long currentMillis;
long previousMillis = 0; // set up timers
float loopTime = 10;

void velCallback( const geometry_msgs::Twist& vel)
{
request_x = vel.linear.x;
request_z = vel.angular.z;
}
// subscribe to the cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

//  Setup 

void setup() {

nh.getHardware()->setBaud(115200); // set baud rate to 115200
nh.initNode(); // init ROS
nh.subscribe(sub); // subscribe to cmd_vel
nh.advertise(odom_pub);
broadcaster.init(nh); // set up broadcaster
Serial.begin(115200); // ROS serial communication
Serial1.begin(115200); // ODrive serial communication

// set the ODrive controller to closed loop state
int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
odrive1.run_state(0, requested_state, false); // don't wait
odrive1.run_state(1, requested_state, false); // don't wait

}

// Main loop

void loop() {

nh.spinOnce(); // listen to ROS messages and activate the callback if necessary

currentMillis = millis();
if (currentMillis - previousMillis >= loopTime) { // run a loop every 10ms
previousMillis = currentMillis; // reset the clock

// scaling factor because the wheels are squashy / there is wheel slip etc.
//float modifier_linear = 1.08;
//float modifier_angular = 0.92;

// convert the parameter units into turn/s
float conversion_linear = (rev * 1000)/wheel_circumference;
float conversion_angular = ((mubot_width/2) * rev)/wheel_circumference;
int m0_linear = request_x * conversion_linear; 
int m1_linear = request_x * conversion_linear; 
int m0_angular = request_z * conversion_angular;        
int m1_angular = request_z * conversion_angular;   

// reverse the direction of rotation of one of the wheel       
m1_linear = m1_linear*-1;

// set the velocity control
odrive1.SetVelocity(0, m0_linear + m0_angular); 
odrive1.SetVelocity(1, m1_linear + m1_angular);

  }
} 
