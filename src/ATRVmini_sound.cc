#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <iostream>
#include <string>
#include <sound_play/sound_play.h>
#include <unistd.h>


class ATRVSound {

public:
  ATRVSound();
  void helloWorld();
  
private:
  void publishSoundPlayed();
  ros::Publisher sound_pub;
  ros::NodeHandle ph, nh;
  void sleepok(int t, ros::NodeHandle &nh);
  boost::mutex publish_mutex_;
  ros::Timer timer_;
};

ATRVSound::ATRVSound() :  ph("~")
{
	sound_pub = ph.advertise<std_msgs::Bool>("sound_played", 10);
	timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&ATRVSound::publishSoundPlayed, this));
}

void ATRVSound::sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
      sleep(t);
 }
 
void ATRVSound::helloWorld()
{
  sound_play::SoundClient sc;
  sleepok(1, nh);
  sc.playWave("/home/atrv/ROS/src/atrv_mini_pkg/sounds/alive.wav");
  sleepok(2, nh);
}
 
void ATRVSound::publishSoundPlayed() {	
  //create a bolean message to publish
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ATRVmini_sound");
  ATRVSound ATRV_sound;
  
  //ros::Duration(1.5).sleep();
  ATRV_sound.helloWorld();
  ros::spin();
}
