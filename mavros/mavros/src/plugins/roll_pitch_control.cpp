#include <mavros/mavros_plugin.h>
#include <mavros_msgs/RollPitchTarget.h>
#include <pluginlib/class_list_macros.h>


namespace mavros
{
    namespace std_plugins
    {
        class RollPitchControlPlugin : public plugin::PluginBase
        {
        public:
            RollPitchControlPlugin() : PluginBase(), rp_ctrl_nh("~roll_pitch_control") {}

            void initialize(UAS &uas_) override
            {
                PluginBase::initialize(uas_);
                
                rp_sp_sub = rp_ctrl_nh.subscribe("setpoints", 
                                                 10, 
                                                 &RollPitchControlPlugin::roll_pitch_sp_cb, 
                                                 this);
            }
            
            Subscriptions get_subscriptions()
            {
                return {/* RX disabled */ };
            }

        private:
            ros::NodeHandle rp_ctrl_nh;
            ros::Subscriber rp_sp_sub;

            void roll_pitch_sp_cb(const mavros_msgs::RollPitchTarget::ConstPtr &req)
            {
                mavlink::common::msg::OMNI_AM_ROLL_PITCH_SETPOINTS rp_sp;
                //mavlink::omni_am::msg::ROLL_PITCH_SETPOINTS rp_sp;

                /* ROS_INFO("Got.\n"); */

                rp_sp.roll_sp = req->roll_sp;
                rp_sp.pitch_sp = req->pitch_sp;

                UAS_FCU(m_uas)->send_message_ignore_drop(rp_sp);
            }
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::RollPitchControlPlugin, mavros::plugin::PluginBase)
