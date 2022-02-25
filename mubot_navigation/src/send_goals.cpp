/*
 # ROS node to send mubot to goal location on a map.

 # 1 = prof_office
 # 2 = secretary_office
 # 3 = phd1_office
 # 4 = phd2_office
 # 5 = technician_office
 # 6 = student_room
 # 7 = workshop
 # 8 = conference_room
 # 9 = resource_room
 # 10 = printer_room
 # 11 = kitchen
 # 12 = passage
 # 13 = elevator
 # 14 = toilet
*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
 
using namespace std;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

    // Connect to ROS
  ros::init(argc, argv, "simple_navigation_goals");
  
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
    // Wait for the action server to come up so that we can begin processing goals.
  while(!ac.waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the move_base action server to come up");
  }
  int goal_location = 14;
  char wish_to_continue = 'Y';
  bool run = true;
     
  while(run) {
    // Ask the user where he wants the robot to go?
  
  cout << "==================================" << endl;
  cout << " Where do you want the mubot to go?  " << endl;
  cout << "==================================" << endl;
  cout << "1 = prof_office" << endl;
  cout << "2 = secretary_office" << endl;
  cout << "3 = phd1_office" << endl;
  cout << "4 = phd2_office" << endl;
  cout << "5 = technician_office" << endl;
  cout << "6 = student_room" << endl;
  cout << "7 = workshop" << endl;
  cout << "8 = conference_room" << endl;
  cout << "9 = resource_room" << endl;
  cout << "10 = printer_room" << endl;
  cout << "11 = kitchen" << endl;
  cout << "12 = passage" << endl;
  cout << "13 = elevator" << endl;
  cout << "14 = toilet" << endl;
  cout << "Please select a number from 0~14 : ";
  cin >> goal_location;
 
    // Create a new goal to send to move_base 
  move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
  goal.target_pose.header.frame_id = "map"; // we could as well use base_link or odom
  goal.target_pose.header.stamp = ros::Time::now();

  bool valid_choice = true;
    
   switch (goal_location) {
    case 1:
      cout << "\nGoal Location: prof_office\n" << endl;
      goal.target_pose.pose.position.x = -2.23382;
      goal.target_pose.pose.position.y = -12.85285;
      goal.target_pose.pose.orientation.w = 0.00315;
      break;
    case 2:
      cout << "\nGoal Location:secretary_office\n" << endl;
      goal.target_pose.pose.position.x = 4.63244;
      goal.target_pose.pose.position.y = -15.53998;
      goal.target_pose.pose.orientation.w = -0.000156;
      break;
    case 3:
      cout << "\nGoal Location: phd1_office\n" << endl;
      goal.target_pose.pose.position.x = 9.18768;
      goal.target_pose.pose.position.y = -15.00094;
      goal.target_pose.pose.orientation.w = 0.14068;
      break;
    case 4:
      cout << "\nGoal Location: phd2_office\n" << endl;
      goal.target_pose.pose.position.x = 3.59374;
      goal.target_pose.pose.position.y = -4.09621;
      goal.target_pose.pose.orientation.w = 0.00233;
      break;
    case 5:
      cout << "\nGoal Location: technician_office\n" << endl;
      goal.target_pose.pose.position.x = 7.47799;
      goal.target_pose.pose.position.y = -15.11448;
      goal.target_pose.pose.orientation.w = 0.00358;
      break;
    case 6:
      cout << "\nGoal Location: student_room\n" << endl;
      goal.target_pose.pose.position.x = 9.55626;
      goal.target_pose.pose.position.y = -3.65955;
      goal.target_pose.pose.orientation.w = 0.00591;
    case 7:
      cout << "\nGoal Location: workshop\n" << endl;
      goal.target_pose.pose.position.x = 6.81044;
      goal.target_pose.pose.position.y = -2.15397;
      goal.target_pose.pose.orientation.w = 0.00529;
      break;
    case 8:
      cout << "\nGoal Location: conference_room\n" << endl;
      goal.target_pose.pose.position.x = 0.58864;
      goal.target_pose.pose.position.y = -4.33523;
      goal.target_pose.pose.orientation.w = 0.00384;
      break;
    case 9:
      cout << "\nGoal Location: resource_room\n" << endl;
      goal.target_pose.pose.position.x = -4.47071;
      goal.target_pose.pose.position.y = -3.44321;
      goal.target_pose.pose.orientation.w = 0.00370;
      break;
    case 10:
      cout << "\nGoal Location: printer_room\n" << endl;
      goal.target_pose.pose.position.x = 8.86839;
      goal.target_pose.pose.position.y = -9.00390;
      goal.target_pose.pose.orientation.w = 0.00263;
      break;
    case 11:
      cout << "\nGoal Location: kitchen\n" << endl;
      goal.target_pose.pose.position.x = -4.64802;
      goal.target_pose.pose.position.y = -15.55919;
      goal.target_pose.pose.orientation.w = 0.00709;
      break;
    case 12:
      cout << "\nGoal Location: passage\n" << endl;
      goal.target_pose.pose.position.x = -8.07833;
      goal.target_pose.pose.position.y = -0.69044;
      goal.target_pose.pose.orientation.w = 0.00583;
      break;
    case 13:
      cout << "\nGoal Location: elevator\n" << endl;
      goal.target_pose.pose.position.x = -8.04683;
      goal.target_pose.pose.position.y = -15.65806;
      goal.target_pose.pose.orientation.w = -0.00110;
      break;
    case 14:
      cout << "\nGoal Location: toilet\n" << endl;
      goal.target_pose.pose.position.x = -11.05528;
      goal.target_pose.pose.position.y = -9.42936;
      goal.target_pose.pose.orientation.w = 0.00710;
      break;
    default:
      cout << "\nInvalid choice. Please try again.\n" << endl;
      valid_choice = false;
    }            
    // Go back to beginning if the choice is invalid.
    if(!valid_choice) {
      continue;
    }
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    
    // Wait until the robot reaches the goal
    ac.waitForResult();
     
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfuly  arrived the goal location");
    else
      ROS_INFO("Failed to reach the goal location");
   
   // Ask if the user wants to continue to another goal
    do {
      cout << "\nDo you wish to go to another goal location? (Y/N)" << endl;
     cin >> wish_to_continue;
     wish_to_continue = tolower(wish_to_continue);
    } while (wish_to_continue != 'n' && wish_to_continue != 'y'); 
 
    if(wish_to_continue =='n') {
        run = false;
    }
  }  
  return 0;
}

