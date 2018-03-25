#include "riegl_remote_control/riegl_remote_control_node.hpp"
#include <signal.h>
#include <thread>
#include <chrono>
// Position in /remote_joy axes array
// static const int buttonPos = 3;
// Button value for "Option 4" Button assigned to controller
// static const double buttonVal = -0.6299212574958801;

RRC::RRC()
    : m_client("riegl_driver/riegl_scan", true)
{
    m_program   = 0;
    m_near_range= 0;
    m_theta_min = 0;
    m_theta_max = 0;
    m_theta_incr= 0;
    m_phi_min   = 0;
    m_phi_max   = 0;
    m_phi_incr  = 0;
    m_sentGoal  = false;

    this->init();

    ros::NodeHandle nh;
    // Subscribe to remote control messages
    sub = nh.subscribe("/joy", 1, &RRC::remote_cb, this);
}

RRC::~RRC() {}

void RRC::remote_cb(const sensor_msgs::Joy::ConstPtr& msg)
{   
    // check for button and already sent the goal
    if (msg->axes[m_button.index] != m_button.value || m_sentGoal)
        return;

    
    m_sentGoal = true;

    ROS_INFO_STREAM("Scan command sent from button " << m_button.index);
    ROS_INFO("Sending a goal from remote...");

    m_client.sendGoal(m_goal,
                      actionlib::SimpleActionClient<riegl_vline_msgs::ScanAction>::SimpleDoneCallback(),
                      actionlib::SimpleActionClient<riegl_vline_msgs::ScanAction>::SimpleActiveCallback(),
                      boost::bind(&RRC::feedback_cb, this, _1));

    ROS_INFO("Goal sent.");

    // Wait for the result
    ROS_INFO("Waiting for riegl scan action to finish...");

    // while(!ros::isShuttingDown() && !m_client.waitForResult(ros::Duration(0.5)));
   

    while(ros::ok() && !m_client.waitForResult(ros::Duration(0.5)))
    {
        ROS_INFO_STREAM("SHUTTING DOWN: " << std::boolalpha << ros::isShuttingDown());
    }

    if (ros::isShuttingDown())
    {   
        ROS_INFO_STREAM("Attempting to shutdown node cleanly...");
        switch(m_client.getState().state_)
        {
            case actionlib::SimpleClientGoalState::SUCCEEDED:
                ROS_INFO_STREAM("Scan succeeded!");
                break;

            case actionlib::SimpleClientGoalState::PENDING:
            case actionlib::SimpleClientGoalState::ACTIVE:
            case actionlib::SimpleClientGoalState::RECALLED:
            case actionlib::SimpleClientGoalState::REJECTED:
            case actionlib::SimpleClientGoalState::ABORTED:
            case actionlib::SimpleClientGoalState::LOST:
                ROS_FATAL_STREAM("Unknown state! Cancelling Scan...");
                m_client.cancelGoal();
                break;

            case actionlib::SimpleClientGoalState::PREEMPTED:
                m_client.cancelGoal();
                ROS_INFO_STREAM("Successfully canceled the scan!");    
                break;


        }
        // ros::spinOnce();
    }
    else
    {
        ROS_INFO_STREAM("Scan failed!");
    }
    
    m_sentGoal = false;
}


void RRC::feedback_cb(const riegl_vline_msgs::ScanFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM(
        std::setw(3) << feedback->progress << "% (" <<
                     feedback->t_elaps  << "s/" <<
                     feedback->t_calc   << "s)");
}

void RRC::init()
{
    ros::NodeHandle private_nh("~");
    // Connect to the action server
    ROS_INFO("Waiting for action server to start...");
    m_client.waitForServer();
    ROS_INFO("Action server started.");

    private_nh.param<std::string>("remote_type", m_button.remote_type, "heros");
    // button config
    private_nh.param<std::string>("location", m_button.loc, "AXES");
    // Innok Heros remote index 3 in axis
    private_nh.param("index", m_button.index, 3);
    // Innok Heros remote value option 4 button
    private_nh.param("value", m_button.value, -0.6299212574958801);

    // Obtain the parameters for goal
    int program;
    private_nh.param("program", program, 1);
    m_goal.program = static_cast<unsigned char>(program);

    bool near_range;
    private_nh.param("near_range", near_range, false);
    m_goal.near_range = static_cast<unsigned char>(near_range);

    private_nh.param("theta_min", m_goal.theta_min, 30.);
    private_nh.param("theta_max", m_goal.theta_max, 130.);
    private_nh.param("theta_incr", m_goal.theta_incr, 0.1);
    private_nh.param("phi_min", m_goal.phi_min, 0.);
    private_nh.param("phi_max", m_goal.phi_max, 360.);
    private_nh.param("phi_incr", m_goal.phi_incr, 0.1);

    private_nh.getParam("remote_type", m_button.remote_type);
    ROS_INFO_STREAM((m_button.remote_type == "heros" ? "Heros" : "PS3") << " remote connected");
}

void mySigintHandler(int sig)
{
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting Riegl Remote Control Node");

    ros::init(argc, argv, "riegl_remote_control_node", ros::init_options::NoSigintHandler);    
    signal(SIGINT, mySigintHandler);
    RRC rc; 

    while(ros::ok()) ros::spinOnce();

    ROS_ERROR_STREAM("Aborted");


    // Construct a signal set registered for process termination.
    // boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);

    // Start an asynchronous wait for one of the signals to occur.
    // signals.async_wait(boost::bind(&RRC::cancelScan, boost::ref(rc), _1));

    //signal(SIGINT, boost::bind(&RRC::cancelScan, boost::ref(rc), _1));
    
    // ros::spin();
    // while (ros::ok()) ros::spinOnce();
    // switch(rc.m_client.getState().state_)
    // {
    //     case actionlib::SimpleClientGoalState::SUCCEEDED:
    //         ROS_INFO_STREAM("Scan succeeded!");
    //         break;

    //     case actionlib::SimpleClientGoalState::PENDING:
    //     case actionlib::SimpleClientGoalState::ACTIVE:
    //     case actionlib::SimpleClientGoalState::RECALLED:
    //     case actionlib::SimpleClientGoalState::REJECTED:
    //     case actionlib::SimpleClientGoalState::ABORTED:
    //     case actionlib::SimpleClientGoalState::LOST:
    //         ROS_FATAL_STREAM("Unknown state! Cancelling Scan...");
    //         rc.m_client.cancelGoal();
    //         break;

    //     case actionlib::SimpleClientGoalState::PREEMPTED:
    //         rc.m_client.cancelGoal();
    //         ROS_INFO_STREAM("Successfully canceled the scan!");    
    //         break;


    // }

    return EXIT_SUCCESS;
}
