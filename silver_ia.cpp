#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "dotbot_msgs/Attack.h"
#include "dotbot_msgs/GetMoves.h"
#include "dotbot_msgs/GetRobotInfo.h"
#include "dotbot_msgs/RobotInfo.h"
#include <string>
#include <time.h>


///------------------IA.hpp---------------------------------------------

class IA
{
public:
    IA();
    ~IA();

    void run();
    void hp_cb(const std_msgs::UInt8::ConstPtr &hp_msg);
    void newturn_cb(std_msgs::Empty empty_msg);


private:
    int _nmove;
    int _Dp1;
    int _Dp2;
    int _Dp3;
    int _hp;
    int _AP;
    bool _newturn;
    dotbot_msgs::GetRobotInfo _silver_srv;
    dotbot_msgs::GetRobotInfo _white_srv;
    dotbot_msgs::RobotInfo _silverinfo;
    dotbot_msgs::RobotInfo _whiteinfo;


    std::string _whitetype;

    ros::Publisher _move_IA_pub;
    ros:: Publisher _log_IA_pub;

    ros::ServiceClient _client_silver_IA;
    ros::ServiceClient _client_white_IA;

    ros::Subscriber _hp_IA_sub;
    ros::Subscriber _nt_IA_sub;
    ros::NodeHandle _nh;

    struct Moves
    {
        std::string _name;
        int _Db;
        int _Am;
        int _Dm;
        bool _priority;
        std::string _type;
        int _cure;
    };

    struct Moves _move1;
    struct Moves _move2;
    struct Moves _move3;
    struct Moves _move4;
};

///---------------------IA.cpp----------------------

IA::IA()
{
    this->_move_IA_pub = this->_nh.advertise<std_msgs::UInt8>("/silver/move", 1000);

    //this->_hp_IA_sub = this->_nh.subscribe("/silver/hp", 1000, &IA::hp_cb, this);
    this->_newturn = false;

    this->_nt_IA_sub = this->_nh.subscribe("/new_turn",1000, &IA::newturn_cb, this);

    this->_client_silver_IA = this->_nh.serviceClient<dotbot_msgs::GetRobotInfo>("/silver/services/get_robot_info");
    this->_client_white_IA = this->_nh.serviceClient<dotbot_msgs::GetRobotInfo>("/white/services/get_robot_info");

     std::cout<<"inizializzo nodo"<<std::endl;

    this->_hp = 100;



}

IA::~IA()
{

}
void IA::hp_cb(const std_msgs::UInt8::ConstPtr &hp_msg)
{

    this->_hp=hp_msg->data;


}
void IA::newturn_cb(std_msgs::Empty empty_msg)
{
    this->_newturn=true;
    std::cout<<"ho letto new turn"<<std::endl;
}


void IA::run()
{
     std::cout << "Sono in run" << std::endl;
    while(ros::ok())
    {
        std_msgs::UInt8 move_msg;

        if(this->_newturn)
        {
            this->_newturn = false;
            if(this->_client_white_IA.call(this->_white_srv))
            {
                std::cout << "Risposta dal server white" << std::endl;
                _whiteinfo=_white_srv.response.robot;
                _whitetype=_whiteinfo.type;
            }



            if(this->_client_silver_IA.call(this->_silver_srv))
            {
                std::cout << "Risposta dal server silver" << std::endl;

                _silverinfo=_silver_srv.response.robot;
                _AP=_silverinfo.atk;

                this->_move1._name = _silverinfo.m1.name;
                this->_move1._Db = _silverinfo.m1.damage;
                this->_move1._Am = _silverinfo.m1.atk_mod;
                this->_move1._Dm = _silverinfo.m1.def_mod;
                this->_move1._priority = _silverinfo.m1.priority;
                this->_move1._type = _silverinfo.m1.type;
                this->_move1._cure = _silverinfo.m1.cure;


                this->_move2._name = _silverinfo.m2.name;
                this->_move2._Db = _silverinfo.m2.damage;
                this->_move2._Am = _silverinfo.m2.atk_mod;
                this->_move2._Dm = _silverinfo.m2.def_mod;
                this->_move2._priority = _silverinfo.m2.priority;
                this->_move2._type = _silverinfo.m2.type;
                this->_move2._cure = _silverinfo.m2.cure;


                this->_move3._name = _silverinfo.m3.name;
                this->_move3._Db = _silverinfo.m3.damage;
                this->_move3._Am = _silverinfo.m3.atk_mod;
                this->_move3._Dm =_silverinfo.m3.def_mod;
                this->_move3._priority = _silverinfo.m3.priority;
                this->_move3._type = _silverinfo.m3.type;
                this->_move3._cure = _silverinfo.m3.cure;


                this->_move4._name = _silverinfo.m4.name;
                this->_move4._Db = _silverinfo.m4.damage;
                this->_move4._Am = _silverinfo.m4.atk_mod;
                this->_move4._Dm = _silverinfo.m4.def_mod;
                this->_move4._priority = _silverinfo.m4.priority;
                this->_move4._type = _silverinfo.m4.type;
                this->_move4._cure = _silverinfo.m4.cure;

                this->_hp = _silverinfo.hp_max;

                std::cout<<"Ho letto info silver"<<std::endl;


            }

            if ( this->_hp < 30)
            {
                std::cout << "hp<30" << std::endl;
                move_msg.data = 4;
            }
            else
            {

                this->_Dp1 = this->_move1._Db * this->_AP * ((rand() %4)/10)+0.8;
                this->_Dp2 = this->_move2._Db * this->_AP * ((rand() %4)/10)+0.8;
                this->_Dp3 = this->_move3._Db * this->_AP * ((rand() %4)/10)+0.8;

                if(this->_whitetype == "grass")    /** Db1 è fuoco - Db2 è acqua - Db3 è erba **/
                {
                    this->_Dp1 = this->_Dp1 * 2;
                    this->_Dp2 = this->_Dp2 / 2;
                }
                if(this->_whitetype == "fire")
                {
                    this->_Dp2 = this->_Dp2 * 2;
                    this->_Dp3 = this->_Dp3 / 2;
                }
                if(this->_whitetype == "water")
                {
                    this->_Dp3 = this->_Dp3*2;
                    this->_Dp1 = this->_Dp1/2;
                }

                if(this->_Dp1 > this->_Dp2)
                {
                    if(this->_Dp1 > this->_Dp3)
                    {
                        move_msg.data = 1;
                    }
                }
                else
                {

                    if(this->_Dp2 > this->_Dp3)
                    {
                        move_msg.data =2;
                    }
                    else
                    {
                        move_msg.data = 3;

                    }
                }




            }
            this->_move_IA_pub.publish(move_msg);
            ros::spinOnce();

        }
        ros::spinOnce();
    }
}
///-------------------------------Main_IA.cpp-----------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "silver_ia");
    IA silver;
    silver.run();
    return 0;
}
