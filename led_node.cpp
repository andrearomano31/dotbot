
#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "dotbot_msgs/Led.h"

//==========================================================Hp_led.hpp==========================================================

class Hp_led {

     public:
           Hp_led();
          ~Hp_led();

          void set_led1(bool);
          void set_led2(bool);
          void set_led3(bool);
          void set_delay_hp(unsigned short int);
          double get_delay() const;
          void hp_CB(const std_msgs::UInt8);
          void led_off();
          void led_on();
          void run();


     private:
          ros::NodeHandle _nh;
          ros::Publisher _led_pub;
          ros::Subscriber _hp_sub;
          double _delay;
          unsigned short int _hp;
          dotbot_msgs::Led _led_msg;

};

//==========================================================Hp_led.cpp==========================================================

Hp_led::Hp_led() {
    this->_hp_sub = this->_nh.subscribe("/silver/hp", 1000, &Hp_led::hp_CB, this);
    this->_led_pub = this->_nh.advertise<dotbot_msgs::Led>("/silver/led", 1000);
    this->set_delay_hp(100);
}

Hp_led::~Hp_led() {
    ROS_INFO("Nodo led terminato");
}

void Hp_led::set_led1(bool _led1) {
     this->_led_msg.led1 = _led1;
}

void Hp_led::set_led2(bool _led2) {
     this->_led_msg.led2 = _led2;
}

void Hp_led::set_led3(bool _led3) {
     this->_led_msg.led3 = _led3;
}

void Hp_led::set_delay_hp(unsigned short int hp) {
     this->_hp = hp;
     if (this->_hp >= 90) {
          this->_delay = 0.001;
     }
     else {
          this->_delay = (hp*((0.46)/90))+0.04;
     }

     //ROS_INFO("%d", this->_hp);
}

double Hp_led::get_delay() const {
     return this->_delay;
}

void Hp_led::hp_CB(const std_msgs::UInt8 hp_msg) {
     this->set_delay_hp(hp_msg.data);
}

void Hp_led::led_off() {
     if(this->_hp < 90) {
          this->set_led1(false);
          this->set_led2(false);
          this->set_led3(false);
          this->_led_pub.publish(this->_led_msg);
          ros::spinOnce();
     }
}

void Hp_led::led_on() {
     if(this->_hp >= 60) {
            this->set_led1(true);
            this->set_led2(true);
            this->set_led3(true);
        }

        else if((this->_hp < 60) && (this->_hp >= 30)) {
            this->set_led1(true);
            this->set_led2(true);
            this->set_led3(false);
        }

        else if((this->_hp < 30) && (this->_hp > 0)) {
            this->set_led1(true);
            this->set_led2(false);
            this->set_led3(false);
        }

        else if(this->_hp <= 0 ) {
            this->set_led1(false);
            this->set_led2(false);
            this->set_led3(false);
        }
     this->_led_pub.publish(this->_led_msg);
     ros::spinOnce();
}
void Hp_led::run(){
     while(ros::ok()) {
     this->led_on();
     ///std::cout << this->_led_msg.led1 << " " << this->_led_msg.led2 << " " << this->_led_msg.led3 << std::endl;
     ros::Duration(this->get_delay()).sleep();
     this->led_off();
     ros::Duration(this->get_delay()).sleep();
     }

}


//==========================================================led_node.hpp==========================================================
int main(int argc, char **argv) {
     ros::init(argc, argv, "led_node");
     Hp_led led;
     led.run();
     return 0;
}
