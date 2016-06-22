#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "dotbot_msgs/Attack.h"
#include "dotbot_msgs/GetMoves.h"
#include "dotbot_msgs/GetRobotInfo.h"
#include <string>
#include <time.h>


//========================================================Robot.hpp========================================================

class Robot {
	public:
		Robot(int,std::string);
		~Robot();
		void run();
		bool moves_srv_CB(dotbot_msgs::GetMoves::Request &req, dotbot_msgs::GetMoves::Response &res);
		bool robot_info_srv_CB(dotbot_msgs::GetRobotInfo::Request &req, dotbot_msgs::GetRobotInfo::Response &res);
		void set_hp(int);
		int get_hp() const;
		void move_CB(const std_msgs::UInt8::ConstPtr &nmove);
		void move_start_CB(std_msgs::Empty empty_msg);
		void attack_CB(const dotbot_msgs::Attack::ConstPtr &damage_msg);
		void set_move();
		void modify_Ap();
		void modify_Df();
		void cure();
		void modify_dam(std::string);
		void modify_Hp();
		void give_dam();
		void new_turn_CB(std_msgs::Empty empty_msg);



	private:

		ros::NodeHandle _nh;

		ros::Publisher _hp_pub;
		ros::Publisher _attack_pub;
		ros::Publisher _move_ended_pub;
		ros::Publisher _move_speed_pub;
		ros::Publisher _log_pub;

		ros::Subscriber _move_sub;
		ros::Subscriber _move_start_sub;
		ros::Subscriber _new_turn_sub;
		ros::Subscriber _attack_sub;

		bool _newTurn;
		bool _move_start;
		bool _get_move;
		int _hp;
		int _DannoRic;
		int _DF;
		int _AP;
		int _SP;
		int _move_n;
		std::string _type_bot;
		std::string _name_robot;

		dotbot_msgs::Move _move1;
		dotbot_msgs::Move _move2;
		dotbot_msgs::Move _move3;
		dotbot_msgs::Move _move4;
		dotbot_msgs::Move *_moveptr;

		dotbot_msgs::RobotInfo robot_status;

};

//========================================================Robot.cpp========================================================

Robot::Robot(int _hp, std::string _name_robot) {

     this->_name_robot = _name_robot;
     this->robot_status.name = _name_robot;
	this->_get_move = true;
	///inizializzo i publisher
	this->_hp_pub = this->_nh.advertise<std_msgs::UInt8>("/" + this->_name_robot + "/hp", 1000);
	this->_attack_pub = this->_nh.advertise<dotbot_msgs::Attack>("/" + this->_name_robot + "/attack", 1000);
	this->_move_ended_pub = this->_nh.advertise<std_msgs::Empty>("/" + this->_name_robot + "/move_ended", 1000);
	this->_move_speed_pub = this->_nh.advertise<std_msgs::UInt8>("/" + this->_name_robot + "/move_speed", 1000);
	this->_log_pub = this->_nh.advertise<std_msgs::String>("/" + this->_name_robot + "/log",1000);
	this->set_hp(_hp);	///inizializzo gli hp
	///inizializzo parametri mossa 1
	_move1.name = "Piro_lancio";
	_move1.damage = 20;
	_move1.atk_mod = 2;
	_move1.def_mod = 0;
	_move1.priority = true;
	_move1.type = "fire";
	_move1.cure = -5;

	///inizializzo parametri mossa 2
	_move2.name = "Pugno_Terremoto";
	_move2.damage = 15;
	_move2.atk_mod = 2;
	_move2.def_mod = 1;
	_move2.priority = false;
	_move2.type = "grass";
	_move2.cure = 0;

	///inizializzo parametri mossa 3
	_move3.name = "Tsunami";
	_move3.damage = 10;
	_move3.atk_mod = 1;
	_move3.def_mod = 1;
	_move3.priority = false;
	_move3.type = "water";
	_move3.cure = 5;

	///inizializzo parametri mossa 4
	_move4.name = "Save_silver";
	_move4.damage = 5;
	_move4.atk_mod = 0;
	_move4.def_mod = 2;
	_move4.priority = true;
	_move4.type = "normal";
	_move4.cure = 15;

	int type = rand()%2;
	switch (type) {
	case 0: this->_type_bot = "water"; break;
	case 1: this->_type_bot = "fire"; break;
	case 2: this->_type_bot = "grass"; break;
	}
     std::cout << "Sono: " << type << " " << this->_type_bot << std::endl;

	///inizializzo info robot
	this->robot_status.type = this->_type_bot;
	this->robot_status.m1 = this->_move1;
	this->robot_status.m2 = this->_move2;
	this->robot_status.m3 = this->_move3;
	this->robot_status.m4 = this->_move4;




	///comunico alla GUI il nome delle mosse
	ros::ServiceServer service_moves = this->_nh.advertiseService("/" + this->_name_robot + "/services/get_moves", &Robot::moves_srv_CB,this);
	///comunico all'IA lo stato del robot
	ros::ServiceServer service_status_robot = this->_nh.advertiseService("/silver/services/get_robot_info", &Robot::robot_info_srv_CB,this);

	///inizializzo AP DF

     this->_AP = 4;
     this->_DF = 5;

}

Robot::~Robot() {
	std::cout << "Died" << std::endl;
}

void Robot::set_hp(int hp){
	std_msgs::UInt8 hp_msg;
	if((hp >= 0) && (hp <=100)) {
		hp_msg.data = hp;
		this->_hp_pub.publish(hp_msg);
		ros::spinOnce();
		this->_hp = hp;
		this->robot_status.hp_max = hp;
	}
	else if(hp > 100) {
		hp_msg.data = 100;
		this->_hp_pub.publish(hp_msg);
		ros::spinOnce();
		this->_hp = 100;
		this->robot_status.hp_max = 100;
	}
	else {
		hp_msg.data = 0;
		this->_hp_pub.publish(hp_msg);
                ros::spinOnce();
		this->_hp = 0;
		this->robot_status.hp_max = 0;
	}

}

int Robot::get_hp() const {
    return this->_hp;
}

void Robot::run() {///funzione principale
    while(ros::ok()){

        this->_move_sub = this->_nh.subscribe("/" + this->_name_robot + "/move", 1000, &Robot::move_CB, this);
        this->_new_turn_sub = this->_nh.subscribe("/new_turn", 1000, &Robot::new_turn_CB, this);

        if(this->_newTurn){

            this->_move_start_sub = this->_nh.subscribe("/" + this->_name_robot + "/move_start",1000, &Robot::move_start_CB, this);
            this->_attack_sub = this->_nh.subscribe("/white/attack",1000, &Robot::attack_CB, this);
            this->_newTurn = false;
        }
	ros::spin();
    }
}

bool Robot::moves_srv_CB(dotbot_msgs::GetMoves::Request &req, dotbot_msgs::GetMoves::Response &res) {
    res.move1 = this->_move1.name;
    res.move2 = this->_move2.name;
    res.move3 = this->_move3.name;
    res.move4 = this->_move4.name;
    this->set_hp(this->_hp);
    ROS_INFO("setting moves");
    return true;
}

bool Robot::robot_info_srv_CB(dotbot_msgs::GetRobotInfo::Request &req, dotbot_msgs::GetRobotInfo::Response &res) {
    res.robot = this->robot_status;
    std::cout<<"mando servizio info"<<std::endl;
    return true;
}

void Robot::new_turn_CB(std_msgs::Empty empty_msg) {
	this->_newTurn = true;
	std::cout<<"inizio mossa"<<std::endl;
}

void Robot::move_CB(const std_msgs::UInt8::ConstPtr &nmove) {

    std_msgs::UInt8 speed_msg;
    if(this->_get_move){ ///prendiamo solo la prima mossa scelta all'inizio del turno
	std::cout<<"leggo mossa"<<std::cout;
        switch (nmove -> data){

            case 1:
                this->_moveptr = &_move1;
                break;

            case 2:
                this->_moveptr = &_move2;
                break;

            case 3:
                this->_moveptr = &_move3;
                break;

            case 4:
                this->_moveptr = &_move4;
                break;

            }
        std_msgs::String move_msg;
        move_msg.data = "Silver usa: " + _moveptr->name;
        this->_log_pub.publish(move_msg);
        ros::spinOnce();
	std::cout<<"pubblico su log"<<std::cout;
        speed_msg.data = this->_SP + 100 * this->_moveptr->priority;
        this->_move_speed_pub.publish(speed_msg);
	ros::spinOnce();
	std::cout<<"pubblico velocità"<<std::cout;
        this->_get_move = false;
    }

}

void Robot::move_start_CB(std_msgs::Empty empty_msgs){
    std_msgs::String log_msg;
    log_msg.data = "Silver usa: " + _moveptr->name;
    this->_log_pub.publish(log_msg);
    ros::spinOnce();
    std::cout<<"leggo mossa"<<std::cout;
    this->give_dam();
    this->cure();
    this->_get_move = true;
    log_msg.data = "Silver ha terminato il suo turno";
    this->_log_pub.publish(log_msg);
    ros::spinOnce();
    std::cout<<"leggo mossa"<<std::cout;
    std_msgs::Empty empty_msg;
    this->_move_ended_pub.publish(empty_msg);
    ros::spinOnce();
    std::cout<<"mossa terminata"<<std::cout;
}

void Robot::attack_CB(const dotbot_msgs::Attack::ConstPtr &damage_msg){
    std_msgs::String defense_msg;
    defense_msg.data = "White mi sta attaccando!!";
    this->_log_pub.publish(defense_msg);
    ros::spinOnce();
    std::cout<<"leggo attacco di white"<<std::cout;
    ///ricevo il messaggio di attacco e calcolo il danno
    this->_DannoRic = damage_msg->damage;
    this->modify_dam(damage_msg->type);
    this->modify_Hp();
}

void Robot::cure () {
    this->_hp = this->_hp + this->_moveptr->cure;
    this->set_hp(this->_hp);
}

void Robot::modify_dam(std::string type_attack) { /// R=robot;


/// Robot tipo pianta

    if(this->_type_bot == "grass") {

        if(type_attack == "grass") {
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else if(type_attack == "fire") {
            this->_DannoRic = this->_DannoRic*2;
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else if(type_attack == "water") {
            this->_DannoRic = this->_DannoRic/2;
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else {
            this->_DannoRic = ceil(this->_DannoRic);
        }
    }
/// Robot tipo fuoco

    else if(this->_type_bot == "fire") {

        if(type_attack == "fire") {
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else if(type_attack == "water") {
            this->_DannoRic = this->_DannoRic*2;
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else if(type_attack == "grass") {
            this->_DannoRic = this->_DannoRic/2;
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else {
            this->_DannoRic = ceil(this->_DannoRic);
        }
    }
/// Robot tipo acqua

    else if(this-> _type_bot == "water") {

        if(type_attack == "water") {
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else if(type_attack == "grass") {
            this->_DannoRic = this->_DannoRic*2;
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else if(type_attack == "fire") {
            this->_DannoRic = this->_DannoRic/2;
            this->_DannoRic = ceil(this->_DannoRic);
        }
        else {
            this->_DannoRic = ceil(this->_DannoRic);
        }
    }
/// Robot tipo normale
    else {

        this->_DannoRic = ceil(this->_DannoRic);
    }
}

void Robot::modify_Hp() {
    int Dr;
    Dr = this->_DannoRic / this->_DF;
    this->_hp = round(this->_hp - Dr);
    this->set_hp(this->_hp);
}

void Robot::give_dam() {
    dotbot_msgs::Attack attack_msg;
    attack_msg.type = this->_moveptr->type;
    attack_msg.damage = this->_moveptr->damage * this->_AP * ((rand() %4)/10)+0.8;
    this->_attack_pub.publish(attack_msg);
    ros::spinOnce();
}


//========================================================silver.cpp========================================================

int main(int argc, char **argv) {
    ros::init(argc, argv, "silver");
    Robot silver(100, "silver");
    silver.run();
    return 0;
}
