#include "../include/koala.h"


koala_bot::koala_bot(NodeHandle nh){
    n_ = nh;
    configureROSComms();
    getROSParams();
}


void
koala_bot::initialize(){

    my_serial_.setStopbits(serial::stopbits_two);
    my_serial_.setBaudrate(baud_);
    my_serial_.setPort(port_);

    cout<<"Attempting to Connect to Koala"<<endl;
    try {
        my_serial_.open();
    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open port "<< my_serial_.getPort() << "err: " << e.what() <<endl;
    }

    if(my_serial_.isOpen()){

        cout<<"Koala is Connected on Port: "<<my_serial_.getPort()<<" at Baudrate: "<<my_serial_.getBaudrate()<<endl;
    }
    else{

        ROS_ERROR("Serial Port was Unable to Open, check port settings and Koala mode knob");
    }

    clock_gettime(CLOCK_MONOTONIC, &start_time_);
    clock_gettime(CLOCK_MONOTONIC, &slow_timer_);
    clock_gettime(CLOCK_MONOTONIC, &fast_timer_);

    dual_drive_ = true;

    test_speed_ = 0;
    setPositionCounter(0,0); //zero out position counter
    //setSpeed(10,0);

    //setPWM(100,100);

}

void
koala_bot::run(){

    clock_gettime(CLOCK_MONOTONIC,&current_time_);

    if(diff_ms(current_time_,slow_timer_)>=(1/SLOW_LOOP_FREQ)*1000){

        getStatus();
        clock_gettime(CLOCK_MONOTONIC, &slow_timer_);

    }

    if(diff_ms(current_time_,fast_timer_)>=(1/FAST_LOOP_FREQ)*1000){

        getCurrentPose();
        //readSpeed();
        clock_gettime(CLOCK_MONOTONIC, &fast_timer_);

    }
}

void
koala_bot::stop(){
    setSpeed(0,0);
}

void
koala_bot::configureROSComms(){

    status_pub_ = n_.advertise<koala::KoalaStatus>("/koala_status",100);
    pose_pub_ = n_.advertise<koala::KoalaPose>("/koala_pose",1000);
    prox_pub_ = n_.advertise<koala::KoalaProximity>("/koala_prox",1000);

    joy_cmd_sub_ = n_.subscribe("/joy",100,&koala_bot::joyCmdCallBack,this);
    skid_cmd_sub_ = n_.subscribe("/skid_cmd",100,&koala_bot::skidCmdCallBack,this);

}
void
koala_bot::getROSParams(){

    if(n_.getParam("port",port_)){
        ROS_INFO("Got Param: %s",port_.c_str());
    }
    else{
        port_ = "/dev/ttyUSB0";
        ROS_WARN("Failed to get Serial Port from Launch File, defaulting to %s \n",port_.c_str());
    }

    if(n_.getParam("baud",baud_)){

        ROS_INFO("Got Param: %d",baud_);

    }
    else{
        baud_ = 38400;
        ROS_WARN("Failed to get Serial Baudrate from Launch File, defaulting to %d \n",baud_);
    }

    my_serial_.setPort(port_.c_str());
    my_serial_.setBaudrate(baud_);
}

bool
koala_bot::configureSpeedController(int Kp,int Ki,int Kd){
    char msg[10];
    sprintf(msg,"A,%d,%d,%d\n",Kp,Ki,Kd);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("a"));
}

bool
koala_bot::configurePosition_Controller(int Kp,int Ki,int Kd){
    char msg[10];
    sprintf(msg,"F,%d,%d,%d\n",Kp,Ki,Kd);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("f"));

}

bool
koala_bot::configureSpeedProfile(int left_max_speed,
                                 int left_accel,
                                 int right_max_speed,
                                 int right_accel){
    char msg[10];
    sprintf(msg,"J,%d,%d,%d,%d\n",left_max_speed,left_accel,right_max_speed,right_accel);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("j"));
}

bool
koala_bot::setDesiredPosition(int left_pos,int right_pos){
    char msg[10];
    sprintf(msg,"C,%d,%d\n",left_pos,right_pos);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("c"));
}

bool
koala_bot::setSpeed(int left_motor,int right_motor){
    char msg[10];

    if(left_motor>127){
        left_motor = 127;
    }else if(left_motor<-127){
        left_motor = -127;
    }
    else{

    }

    if(right_motor>127){
        right_motor = 127;
    }else if(right_motor<-127){
        right_motor = -127;
    }
    else{

    }
    sprintf(msg,"D,%d,%d\n",left_motor,right_motor);
    my_serial_.write(msg);

    string ack = my_serial_.readline();

    return(ack.compare("d"));
}

bool
koala_bot::setPWM(int left_pwm, int right_pwm){
    char msg[10];
    sprintf(msg,"P,%d,%d\n",left_pwm,right_pwm);
    my_serial_.write(msg);

    string ack = my_serial_.readline();

    return(ack.compare("p"));

}
bool
koala_bot::setPositionCounter(int left_pos,int right_pos){
    char msg[10];
    sprintf(msg,"G,%d,%d\n",left_pos,right_pos);
    my_serial_.write(msg);

    string ack = my_serial_.readline();

    return(ack.compare("g"));
}

void
koala_bot::resetPositionCounter(){
    setPositionCounter(0,0);
}

bool
koala_bot::readPosition(){

    my_serial_.write("H\n");

    string msg = my_serial_.readline();

    printf("%s\n",msg.c_str());
    return 0;
}

void
koala_bot::readSpeed(){

    my_serial_.write("E\n");
    string msg = my_serial_.readline();
    printf("%s\n",msg.c_str());
}

void
koala_bot::getCurrentPose(){

    my_serial_.write("H\n");
    string msg = my_serial_.readline();

    if(msg.size()>0){

        int ind = msg.find_first_of(",");
        int ind2 = msg.find_first_of(",",ind+1);

        string data1 = msg.substr(ind+1,ind2-ind);
        string data2 = msg.substr(ind2+1,msg.size()-ind2);
        int left_pos = atoi(data1.c_str());
        int right_pos = atoi(data2.c_str());

        pose_msg_.position = 0.5*(0.045*(left_pos + right_pos))/1000;
        pose_msg_.heading  = (1/WHEEL_BASE)*((0.045*(left_pos-right_pos))/1000);


    }

    my_serial_.write("E\n");
    string msg1 = my_serial_.readline();

    if(msg1.size()>0){

        int ind = msg1.find_first_of(",");
        int ind2 = msg1.find_first_of(",",ind+1);

        string ack = msg1.substr(0,ind-1);
        string data1 = msg1.substr(ind+1,ind2-ind);
        string data2 = msg1.substr(ind2+1,msg1.size()-ind2);
        int left_vel = atoi(data1.c_str());
        int right_vel = atoi(data2.c_str());

        pose_msg_.velocity = 0.5*(4.5*(left_vel + right_vel))/1000;
        pose_msg_.yaw_rate = (1/WHEEL_BASE)*((4.5*(left_vel-right_vel))/1000);

    }

    pose_pub_.publish(pose_msg_);

}

void
koala_bot::getStatus(){

    if(status_pub_.getNumSubscribers()>0){

        int data[6];

        for(int i=0;i<6;i++){
            char write_buf[10];

            sprintf(write_buf,"M,%u\n",i);
            my_serial_.write(write_buf);

            string msg = my_serial_.readline();

            int ind1 = msg.find_first_of(",");
            string dat = msg.substr(ind1+1,msg.size()-ind1);

            char local;
            msg.copy(&local,msg.size(),0);

            data[i]=atoi(dat.c_str());

        }

        clock_gettime(CLOCK_MONOTONIC,&current_time_);

        status_msg_.header.stamp.sec = current_time_.tv_sec;
        status_msg_.header.stamp.nsec = current_time_.tv_nsec;
        status_msg_.battery_voltage = 20.0*data[0]/1000.0; //20mV per integer converted to volts
        status_msg_.current_consumption = 8.0*data[1]; // Converted to mA
        status_msg_.ambient_temp = ((9/5)*(0.1*data[2]))+32; // temperature in degrees F
        status_msg_.left_motor_current = 4*data[3];
        status_msg_.right_motor_current = 4*data[4];
        status_msg_.battery_temp = ((9/5)*(0.1*data[5]))+32;


        status_pub_.publish(status_msg_);
    }

}
void
koala_bot::getBatteryStatus(){}

// Sensor Reading Prototypes
void koala_bot::readProximitySensors(){}
void koala_bot::readLightSensors(){}
void koala_bot::read_ADC(int channel){}

void koala_bot::skidCmdCallBack(const ugv_cmd_msgs::SkidSteer &skid_msg){

    float velocity;
    float yaw_rate;

    //Saturate Commanded Speed
    if(skid_msg.velocity > MAX_SPEED){
        velocity = MAX_SPEED;
    }else if(skid_msg.velocity < -MAX_SPEED){
        velocity = -MAX_SPEED;
    }else{
        velocity = skid_msg.velocity;
    }

    //Saturate Commanded Yaw Rate
    if(skid_msg.yaw_rate > MAX_YAW_RATE){
        yaw_rate = MAX_YAW_RATE;
    }else if(skid_msg.yaw_rate < -MAX_YAW_RATE){
        yaw_rate = -MAX_YAW_RATE;
    }else{
        yaw_rate = skid_msg.yaw_rate;
    }

    int speed_L = (skid_msg.yaw_rate*(1/WHEEL_BASE) + skid_msg.velocity)*(1000/4.5); //Calculate desired wheel velocities and convert to
    int speed_R = (-skid_msg.yaw_rate*(1/WHEEL_BASE) + skid_msg.velocity)*(1000/4.5);

    printf("Left Cmd: %d ; Right Cmd: %d\n",speed_L,speed_R);

}

void koala_bot::joyCmdCallBack(const sensor_msgs::Joy &joy_msg){


    LX_ = joy_msg.axes[0];
    LY_ = joy_msg.axes[1];
    LT_ = -joy_msg.axes[2];
    RX_ = -joy_msg.axes[3];
    RY_ = joy_msg.axes[4];
    RT_ = joy_msg.axes[5];
    DPX_ = joy_msg.axes[6];
    DPY_ = joy_msg.axes[7];

    BA_ = joy_msg.buttons[0];

    if(BA_){
        printf("toggle\n");
        dual_drive_= !dual_drive_;
    }

    if(dual_drive_){
        speed_L_ = LY_*100;
        speed_R_ = RY_*100;
    }
    else{

    double yaw_rate = (RX_*(0.5*MAX_YAW_RATE));
    double velocity = (RY_*(0.5*MAX_SPEED));
   // float yaw_rate = (RX_*(0.5*MAX_YAW_RATE + 0.1*MAX_YAW_RATE*boost));
   // float velocity = (RY_*(0.5*MAX_SPEED + 0.1*MAX_SPEED*boost));

        speed_L_ = int ((yaw_rate*WHEEL_BASE + velocity)*(1000/4.5)); //Calculate desired wheel velocities and convert to
        speed_R_ = int ((-yaw_rate*WHEEL_BASE + velocity)*(1000/4.5));
    }

    if(speed_L_<-99)
        speed_L_=-99;

    if(speed_R_<-99)
        speed_R_=-99;

   // printf("Dual Drive %d Left Speed: %d ; Right Speed: %d\n",dual_drive_,speed_L_, speed_R_);

    setSpeed(speed_L_,speed_R_);
}


int diff_ms(timespec curr, timespec prev){

    int ms = (curr.tv_sec - prev.tv_sec)*1000 + (curr.tv_nsec - prev.tv_nsec)/100000;

    return ms;
}
