#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "dotbot_msgs/Speed.h"
#include "dotbot_msgs/Attack.h"

#define speed_motor 85

bool _start_motor = false;
ros::Publisher _speed_pub;

void move_CB(const std_msgs::UInt8::ConstPtr &move_msg);
void start_motor_CB(const std_msgs::Empty::ConstPtr &empty_msg);
void attack_CB(const dotbot_msgs::Attack::ConstPtr &attack_msg);


int main(int argc, char **argv) {
     ros::init(argc, argv, "motor_node");
     ros::NodeHandle n;
     _speed_pub = n.advertise<dotbot_msgs::Speed>("/silver/speed", 1000);
     ros::Subscriber move_sub = n.subscribe("/silver/move", 1000, move_CB);
     ros::Subscriber move_start_sub = n.subscribe("/silver/move_start", 1000, start_motor_CB);
     ros::Subscriber attack_sub = n.subscribe("/white/attack", 1000, attack_CB);
     ros::spin();
     return 0;
}

void move_CB(const std_msgs::UInt8::ConstPtr &move_msg) {

     dotbot_msgs::Speed speed_msg;
     if (_start_motor) {
          switch(move_msg -> data) {

               case 1:
                    speed_msg.dx = speed_motor;
                    speed_msg.sx = 0;
                    _speed_pub.publish(speed_msg);
                    ros::spinOnce();
               break;

               case 2:
                    speed_msg.dx = 0;
                    speed_msg.sx = speed_motor;
                    _speed_pub.publish(speed_msg);
                    ros::spinOnce();
               break;

               case 3:
                    speed_msg.dx = speed_motor;
                    speed_msg.sx = speed_motor;
                    _speed_pub.publish(speed_msg);
                    ros::spinOnce();
               break;

               case 4:
                    speed_msg.dx = speed_motor;
                    speed_msg.sx = 0;
                    _speed_pub.publish(speed_msg);
                    ros::spinOnce();
               break;
          }
          _start_motor = false;
     }

}

void start_motor_CB(const std_msgs::Empty::ConstPtr &empty_msg) {
    _start_motor = true;

}

void attack_CB(const dotbot_msgs::Attack::ConstPtr &attack_msg){
     dotbot_msgs::Speed speed_msg;
     speed_msg.dx = -speed_motor;
     speed_msg.sx = -speed_motor;
     _speed_pub.publish(speed_msg);
     ros::spinOnce();
     sleep(2);
     speed_msg.dx = speed_motor;
     speed_msg.sx = speed_motor;
}
