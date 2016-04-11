#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ev3dev.h"

using namespace std;
using namespace ev3dev;

class Ev3test
{
  public:
    Ev3test(int speed);
    ~Ev3test();

    void update();

  private:
    ros::Subscriber sub_cmd_;
    ros::Publisher  pub_state_;

    void cmdCallback(const std_msgs::String::ConstPtr& msg);
    void checkForNewCMD();
    void init_state();
    void idle_state();
    void run_state();

    large_motor motor_;

    enum States
    {
      INIT,
      IDLE,
      RUN
    };

    States state_;

    bool cmd_received_;
    std_msgs::String cmd_;
    std_msgs::String sstate_;

    ros::Rate loop_rate_;
};

/*******************
CTOR
*******************/
Ev3test::Ev3test(int speed=100) :
  motor_(OUTPUT_B),
  cmd_received_(false),
  state_(INIT),
  loop_rate_(ros::Rate(10.0))
{
  ros::NodeHandle nh("~");
  
  pub_state_ = nh.advertise<std_msgs::String>("state", 1);
  sub_cmd_ = nh.subscribe("cmd", 10, &Ev3test::cmdCallback, this);

  motor_.set_duty_cycle_sp(speed);
}

/***************
DTOR
***************/
Ev3test::~Ev3test() {
  sub_cmd_.shutdown();
}
/***************/
void Ev3test::init_state()
{
  if (motor_.connected())
    state_ = IDLE;
  else
    state_ = INIT;
}
void Ev3test::idle_state()
{
  motor_.stop();
}
void Ev3test::run_state()
{
  motor_.run_forever();
}

/**************
Check for new CMD
**************/
void Ev3test::checkForNewCMD()
{
  if (!cmd_received_)
    return;

  ROS_INFO_STREAM("Received CMD: " << cmd_.data);

  if (     cmd_.data == "start")
  {
    state_ = RUN;
    sstate_.data = "RUN";
    pub_state_.publish(sstate_);
  }
  else if (cmd_.data == "stop")
  {
    state_ = IDLE;
    sstate_.data = "IDLE";
    pub_state_.publish(sstate_);
  }
  else
    ROS_ERROR_STREAM("CMD not supported: " << cmd_.data);

  cmd_received_ = false;
}
/**************
Update
**************/
void Ev3test::update()
{
  checkForNewCMD();

  switch(state_)
  {
    case INIT: init_state(); break;
    case IDLE: idle_state(); break;
    case RUN:  run_state();  break;
  }
}
/**************
cmd CB 
***************/
void Ev3test::cmdCallback(const std_msgs::String::ConstPtr& msg)
{
  cmd_ = *msg;
  cmd_received_ = true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "ev3test_controller");
  ros::NodeHandle nh("~");

  Ev3test ev3_controller;

  ros::Rate loop_rate(100);

  while(ros::ok())
  {  
    ros::spinOnce();

    ev3_controller.update();

    loop_rate.sleep();
  }

  return 0;
}

