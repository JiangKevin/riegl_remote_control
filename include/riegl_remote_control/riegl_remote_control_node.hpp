#ifndef RIEGL_REMOTE_CONTROL_NODE
#define RIEGL_REMOTE_CONTROL_NODE

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <riegl_vline_msgs/ScanAction.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <boost/bind.hpp>
#include <string>

using Client = actionlib::SimpleActionClient<riegl_vline_msgs::ScanAction>;

class RRC
{
public:
    
    RRC();
    ~RRC();
    
    /// Listens to remote control and send goal to action server on trigger 
    void remote_cb(const sensor_msgs::Joy::ConstPtr& msg);

    /// Prints how far the scan has progressed in %
    void feedback_cb(const riegl_vline_msgs::ScanFeedbackConstPtr& feedback);
    
    /// Reads parameters for goal
    void init();

    // inline Client& client() const { return m_client; };

    // void cancelScan(int sig);
    
    Client m_client;

private:
    /// SimpleActionClient for goal handling
    // actionlib::SimpleActionClient<riegl_vline_msgs::ScanAction> m_client;
    
    /// Nodehandler to have access to params
    ros::NodeHandle m_nh;
    
    /// Goal to be defined
    riegl_vline_msgs::ScanGoal m_goal;
    int     m_program;
    bool    m_near_range;
    double  m_theta_min;
    double  m_theta_max;
    double  m_theta_incr;
    double  m_phi_min;
    double  m_phi_max;
    double  m_phi_incr;

    ros::Subscriber sub;

    /// Struct to describe button placement, value and index
    struct Button
    {   
        std::string remote_type;
        /// Location of button in sensor_msgs/Joy (axes/buttons)
        std::string loc;
        /// Index in axes/buttons array of sensor_msgs/Joy 
        int index;
        /// Value of button on press
        double value;
    } m_button;

    /// Used to determine whether scan is in progress or not
    bool    m_sentGoal;
};

#endif
