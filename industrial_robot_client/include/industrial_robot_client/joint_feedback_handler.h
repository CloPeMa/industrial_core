


#ifndef JOINT_HANDLER_H
#define JOINT_HANDLER_H

#include <string>
#include <vector>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/joint_data.h"
#include "simple_message/message_handler.h"
#include "simple_message/simple_message.h"
#include "simple_message/messages/joint_feedback_message.h"

namespace industrial_robot_client
{
namespace joint_feedback_handler
{

using industrial::joint_data::JointData;
using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

/**
 * \brief Message Handler that handles joint feedback messages for multiple
 * control groups.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackHandler : public industrial::message_handler::MessageHandler
{

    public:
        /** \brief Empty class constructor. */
        JointFeedbackHandler() {};

        /** \brief Initialize multi-group joint relay handler */
        bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

    protected:

        /** \brief ROS Node handle */
        ros::NodeHandle node_;

        ros::Publisher pub_joint_control_state_;
        ros::Publisher pub_joint_sensor_state_;

        /**
         * \brief Convert ROS-I JointMessage to std ROS messages.
         *
         * \param[in] msg_in
         * \param[out] control_state
         * \param[out] sensor_state
         *
         * \return true if ok, false in case of error
         */
        virtual bool create_messages(JointFeedbackMessage& msg_in,
                                     control_msgs::FollowJointTrajectoryFeedback* control_state,
                                     sensor_msgs::JointState* sensor_state);


        /**
         * \brief Get joint names for particular control group.
         *
         * \param[in]  robot_id
         * \param[out] joint_names
         *
         * \return true if ok, false in case of error
         */
        virtual bool get_joint_names(int robot_id, std::vector<std::string>* joint_names);

        /**
         * \brief internal callback
         */
        bool internalCB(JointFeedbackMessage& in);


    private:

        /**
         * \brief Internal callback called by MessageManager, it performs
         * conversion to the JointFeedbackMessage.
         */
        bool internalCB(SimpleMessage& in);

};

}
}


#endif
