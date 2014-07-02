#include "../include/koala.h"
#include <time.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "koala_node");

    ros::NodeHandle nh = ros::NodeHandle("~");

    koala_bot* my_koala = new koala_bot(nh);

    my_koala->initialize();

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        my_koala->run();

        ros::spinOnce();// Allow ROS to check for new ROS Messages
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate

    }

    my_koala->stop();


    return 0;
}
