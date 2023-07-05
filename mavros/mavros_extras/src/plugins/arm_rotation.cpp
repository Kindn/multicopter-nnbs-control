#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ArmRotation.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

namespace mavros
{
    namespace extra_plugins
    {
        class ArmRotationPlugin : public plugin::PluginBase
        {
        public:
            ArmRotationPlugin() : PluginBase(), arm_rotation_nh("~arm_rotation") {}
            void initialize(UAS &uas_) override
            {
                PluginBase::initialize(uas_);
                arm_rotation_pub = arm_rotation_nh.advertise<mavros_msgs::ArmRotation>("raw_data", 10);
                j1_pub = j1_nh.advertise<std_msgs::Float32>("/omni_am/stator1_j/pos_cmd", 10);
                j2_pub = j2_nh.advertise<std_msgs::Float32>("/omni_am/stator2_j/pos_cmd", 10);
                j3_pub = j3_nh.advertise<std_msgs::Float32>("/omni_am/stator3_j/pos_cmd", 10);
                j4_pub = j4_nh.advertise<std_msgs::Float32>("/omni_am/stator4_j/pos_cmd", 10);
                j5_pub = j5_nh.advertise<std_msgs::Float32>("/omni_am/stator5_j/pos_cmd", 10);
                j6_pub = j6_nh.advertise<std_msgs::Float32>("/omni_am/stator6_j/pos_cmd", 10);
            }
            Subscriptions get_subscriptions() override
            {
                return {make_handler(&ArmRotationPlugin::handle_arm_rotation)};
            }

        private:
            ros::NodeHandle arm_rotation_nh;
            ros::Publisher arm_rotation_pub;

            ros::NodeHandle j1_nh;
            ros::Publisher j1_pub;

            ros::NodeHandle j2_nh;
            ros::Publisher j2_pub;

            ros::NodeHandle j3_nh;
            ros::Publisher j3_pub;

            ros::NodeHandle j4_nh;
            ros::Publisher j4_pub;

            ros::NodeHandle j5_nh;
            ros::Publisher j5_pub;
            
            ros::NodeHandle j6_nh;
            ros::Publisher j6_pub;

            void handle_arm_rotation(const mavlink::mavlink_message_t *msg, mavlink::omni_am::msg::ARM_ROTATION &arm_rotation)
            {
                auto arm_rotation_msg = boost::make_shared<mavros_msgs::ArmRotation>();
                arm_rotation_msg->header.stamp = m_uas->synchronise_stamp(arm_rotation.timestamp);
                arm_rotation_msg->pos1 = arm_rotation.pos1;
                arm_rotation_msg->pos2 = arm_rotation.pos2;
                arm_rotation_msg->pos3 = arm_rotation.pos3;
                arm_rotation_msg->pos4 = arm_rotation.pos4;
                arm_rotation_msg->pos5 = arm_rotation.pos5;
                arm_rotation_msg->pos6 = arm_rotation.pos6;
                arm_rotation_msg->vel1 = arm_rotation.vel1;
                arm_rotation_msg->vel2 = arm_rotation.vel2;
                arm_rotation_msg->vel3 = arm_rotation.vel3;
                arm_rotation_msg->vel4 = arm_rotation.vel4;
                arm_rotation_msg->vel5 = arm_rotation.vel5;
                arm_rotation_msg->vel6 = arm_rotation.vel6;
                arm_rotation_pub.publish(arm_rotation_msg);

                std_msgs::Float32 j1_data;
                j1_data.data = arm_rotation.pos1;
                j1_pub.publish(j1_data);

                std_msgs::Float32 j2_data;
                j2_data.data = arm_rotation.pos2;
                j2_pub.publish(j2_data);

                std_msgs::Float32 j3_data;
                j3_data.data = arm_rotation.pos3;
                j3_pub.publish(j3_data);

                std_msgs::Float32 j4_data;
                j4_data.data = arm_rotation.pos4;
                j4_pub.publish(j4_data);

                std_msgs::Float32 j5_data;
                j5_data.data = arm_rotation.pos5;
                j5_pub.publish(j5_data);

                std_msgs::Float32 j6_data;
                j6_data.data = arm_rotation.pos6;
                j6_pub.publish(j6_data);

            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ArmRotationPlugin, mavros::plugin::PluginBase)