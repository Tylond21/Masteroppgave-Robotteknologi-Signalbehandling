#include "abb_librws/rws_state_machine_interface.h"
#include "stdio.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Pose.h>
#include <cstdlib>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include  "math.h"

using namespace abb;
using namespace rws;
using namespace std;

void Reset_Signals();
void Reset_EGM_STOP();
void Reset_EGM_START();
void Reset_RUN_RAPID();
void Set_EGM_STOP();
void Set_EGM_START();
void Set_RUN_RAPID();
void Restart_System();

//
// rws constants
//
#define     POS_TOLERANCE       0.01
#define     COMM_TIMEOUT        10
#define     RAMP_IN_TIME        0.1
#define     RAMP_OUT_TIME       0.1
#define     COND_TIME           10000 //Changed from 0.01
#define     COMM_DELAY          0.1
#define     PORT_REAL_ROBOT     80
#define     PORT_ROBOT_STUDIO   8080
#define     LP                  70
#define     K                  1
#define     MAX_SPEED_DEV       100.0
#define     SAMPLE_RATE         4
#define     COND_MIN_MAX        1
#define     RWS_START_DELAY     2


const string IP_ADDRESS_ROBOT_STUDIO = "192.168.209.129";
//const string IP_ADDRESS_REAL_ROBOT = "172.16.217.36";
const string IP_ADDRESS_REAL_ROBOT = "192.168.1.114";

//RWSStateMachineInterface IRB_140(IP_ADDRESS_ROBOT_STUDIO, PORT_REAL_ROBOT, PORT_ROBOT_STUDIO);
RWSStateMachineInterface IRB_140(IP_ADDRESS_REAL_ROBOT, PORT_REAL_ROBOT);

const string TASK = SystemConstants::RAPID::TASK_ROB_1;
const string ROBOT = SystemConstants::General::MECHANICAL_UNIT_ROB_1;
const string SIGNAL_EGM_STOP = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_STOP;
const string SIGNAL_EGM_START = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::EGM_START_POSE;
const string SIGNAL_RUN_RAPID = RWSStateMachineInterface::ResourceIdentifiers::IOSignals::RUN_RAPID_ROUTINE;
const string SIGNAL_SUCTION_ON = "AirValve";
const string HIGH = SystemConstants::IOSignals::HIGH;
const string LOW = SystemConstants::IOSignals::LOW;

bool vacuum = false;

void vacuum_callback(const std_msgs::Bool& msg )
{
    if(vacuum)
    {
        IRB_140.setIOSignal(SIGNAL_SUCTION_ON, HIGH);
        vacuum = false;
    }
        
    else
    {
        IRB_140.setIOSignal(SIGNAL_SUCTION_ON, LOW);
        vacuum = true;
    }
}

int main(int argc, char** argv)
{
    //
    // ros initializations
    //
    ros::init(argc, argv, "rws_demo");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("/robot_gripper", 1, &vacuum_callback);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //
    // egm settings
    //
    RWSStateMachineInterface::EGMSettings egm_settings;

    egm_settings.activate.tool.robhold = true;
    egm_settings.activate.tool.tframe.pos.x = 0 ;
    egm_settings.activate.tool.tframe.pos.y = 0;
    egm_settings.activate.tool.tframe.pos.z = 0;
    egm_settings.activate.tool.tframe.rot.q1 = 1;
    egm_settings.activate.tool.tframe.rot.q2 = 0;
    egm_settings.activate.tool.tframe.rot.q3 = 0;
    egm_settings.activate.tool.tframe.rot.q4 = 0;
    egm_settings.activate.tool.tload.mass = 0.001;
    egm_settings.activate.tool.tload.cog.x = 0;
    egm_settings.activate.tool.tload.cog.y = 0;
    egm_settings.activate.tool.tload.cog.z = 0.001;
    egm_settings.activate.tool.tload.aom.q1 = 1;
    egm_settings.activate.tool.tload.aom.q2 = 0;
    egm_settings.activate.tool.tload.aom.q3 = 0;
    egm_settings.activate.tool.tload.aom.q4 = 0;
    egm_settings.activate.tool.tload.ix = 0;
    egm_settings.activate.tool.tload.iy = 0;
    egm_settings.activate.tool.tload.iz = 0;

    egm_settings.activate.wobj.robhold = false;
    egm_settings.activate.wobj.ufprog = true;
    egm_settings.activate.wobj.ufmec.value = "\0";
    egm_settings.activate.wobj.uframe.pos.x = 0;
    egm_settings.activate.wobj.uframe.pos.y = 0;
    egm_settings.activate.wobj.uframe.pos.z = 0;
    egm_settings.activate.wobj.uframe.rot.q1 = 1;
    egm_settings.activate.wobj.uframe.rot.q2 = 0;
    egm_settings.activate.wobj.uframe.rot.q3 = 0;
    egm_settings.activate.wobj.uframe.rot.q4 = 0;
    egm_settings.activate.wobj.oframe.pos.x = 0;
    egm_settings.activate.wobj.oframe.pos.y = 0;
    egm_settings.activate.wobj.oframe.pos.z = 0;
    egm_settings.activate.wobj.oframe.rot.q1 = 1;
    egm_settings.activate.wobj.oframe.rot.q2 = 0;
    egm_settings.activate.wobj.oframe.rot.q3 = 0;
    egm_settings.activate.wobj.oframe.rot.q4 = 0;

    egm_settings.activate.correction_frame.operator= (egm_settings.activate.tool.tframe);
    egm_settings.activate.sensor_frame.operator= (egm_settings.activate.tool.tframe);

    egm_settings.activate.lp_filter = LP;
    egm_settings.activate.max_speed_deviation = MAX_SPEED_DEV;
    egm_settings.activate.sample_rate = SAMPLE_RATE;
    egm_settings.activate.cond_min_max = COND_MIN_MAX;

    egm_settings.allow_egm_motions = true;
    egm_settings.setup_uc.comm_timeout = COMM_TIMEOUT;
    egm_settings.setup_uc.use_filtering = true;
    egm_settings.stop.ramp_out_time = RAMP_OUT_TIME;

    egm_settings.run.ramp_in_time = RAMP_IN_TIME;
    egm_settings.run.cond_time = COND_TIME;
    egm_settings.run.pos_corr_gain = K;
    egm_settings.run.offset.operator=(egm_settings.activate.tool.tframe);
 

    Reset_Signals();
    Restart_System();
    Restart_System();    
    ros::Duration(2).sleep();
    IRB_140.services().egm().setSettings(TASK, egm_settings);
    Set_EGM_START();
    ros::waitForShutdown();
    Reset_EGM_START();
    Set_EGM_STOP();
    return 0;
}

void Reset_Signals()
{
    Reset_EGM_START();
    Reset_EGM_STOP();
    Reset_RUN_RAPID();
}
void Reset_EGM_STOP()
{
    if(IRB_140.getIOSignal(SIGNAL_EGM_STOP) == HIGH)
        IRB_140.setIOSignal(SIGNAL_EGM_STOP, LOW);

}
void Reset_EGM_START()
{
    if(IRB_140.getIOSignal(SIGNAL_EGM_START) == HIGH)
        IRB_140.setIOSignal(SIGNAL_EGM_START, LOW);

}
void Reset_RUN_RAPID()
{
    if(IRB_140.getIOSignal(SIGNAL_RUN_RAPID) == HIGH)
        IRB_140.setIOSignal(SIGNAL_RUN_RAPID, LOW);

}
void Set_EGM_STOP()
{
    if(IRB_140.getIOSignal(SIGNAL_EGM_STOP) == LOW)
        IRB_140.setIOSignal(SIGNAL_EGM_STOP, HIGH);

}
void Set_EGM_START()
{
    if(IRB_140.getIOSignal(SIGNAL_EGM_START) == LOW)
        IRB_140.setIOSignal(SIGNAL_EGM_START, HIGH);
}
void Set_RUN_RAPID()
{
    if(IRB_140.getIOSignal(SIGNAL_RUN_RAPID) == LOW)
        IRB_140.setIOSignal(SIGNAL_RUN_RAPID, HIGH);

}

void Restart_System()
{
    if(IRB_140.isRAPIDRunning().isTrue())
    {
        IRB_140.stopRAPIDExecution();
        ros::Duration(COMM_DELAY).sleep();
        IRB_140.resetRAPIDProgramPointer();
        ros::Duration(COMM_DELAY).sleep();
        IRB_140.startRAPIDExecution();
    }
    else
    {
        IRB_140.resetRAPIDProgramPointer();
        ros::Duration(COMM_DELAY).sleep();
        if(IRB_140.isMotorOn().isFalse())
            IRB_140.setMotorsOn();
        ros::Duration(COMM_DELAY).sleep();
        IRB_140.startRAPIDExecution();
    }
}
