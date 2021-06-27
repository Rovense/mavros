
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/OpticalFlow.h> 
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

class flow_rad_msg;
class flow_rad_msg;
namespace mavros {
namespace extra_plugins{

class PX4FlowPlugin : public plugin::PluginBase {
public:
	PX4FlowPlugin() : PluginBase(),
		flow_nh("~px4flow"),
		ranger_fov(0.0),
		ranger_min_range(0.3),
		ranger_max_range(5.0)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		flow_nh.param<std::string>("frame_id", frame_id, "px4flow");

		flow_nh.param("ranger_fov", ranger_fov, 0.119428926);

		flow_nh.param("ranger_min_range", ranger_min_range, 0.3);
		flow_nh.param("ranger_max_range", ranger_max_range, 5.0);
		flow_pub = flow_nh.advertise<mavros_msgs::OpticalFlow>("raw/optical_flow", 10); 
		range_pub = flow_nh.advertise<sensor_msgs::Range>("ground_distance", 10);
		

		flow_sub = flow_nh.subscribe("raw/send", 1, &PX4FlowPlugin::send_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&PX4FlowPlugin::handle_optical_flow)
		};
	}

private:
	ros::NodeHandle flow_nh;

	std::string frame_id;

	double ranger_fov;
	double ranger_min_range;
	double ranger_max_range;

	ros::Publisher flow_pub;
	ros::Publisher range_pub;
	ros::Subscriber flow_sub;

	void handle_optical_flow(const mavlink::mavlink_message_t *msg, mavlink::common::msg::OPTICAL_FLOW &flow)
	{
		auto header = m_uas->synchronized_header(frame_id, flow.time_usec);

		auto flow_msg = boost::make_shared<mavros_msgs::OpticalFlow>();

		flow_msg->header = header;
		flow_msg->flow_x = flow.flow_x;

		flow_msg->flow_y = flow.flow_y;
		flow_msg->flow_comp_m_x = flow.flow_comp_m_x;

		flow_msg->flow_comp_m_y = flow.flow_comp_m_y;
		flow_msg->ground_distance = flow.ground_distance;
		flow_msg->quality = flow.quality;
        	flow_msg->flow_rate_x = flow.flow_rate_x;
        	flow_msg->flow_rate_y = flow.flow_rate_y;

		flow_pub.publish(flow_msg);

		auto range_msg = boost::make_shared<sensor_msgs::Range>();

		range_msg->header = header;

		range_msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
		range_msg->field_of_view = ranger_fov;
		range_msg->min_range = ranger_min_range;
		range_msg->max_range = ranger_max_range;
		range_msg->range = flow.ground_distance;

		range_pub.publish(range_msg);
	}

	void send_cb(const mavros_msgs::OpticalFlow::ConstPtr msg)
	{
		mavlink::common::msg::OPTICAL_FLOW flow_msg = {};


		flow_msg.time_usec = msg->header.stamp.toNSec() / 1000;
		flow_msg.sensor_id = 0;
		flow_msg.flow_x = msg->flow_x;
		flow_msg.flow_y = msg->flow_y;
		flow_msg.flow_comp_m_x = msg->flow_comp_m_x;
		flow_msg.flow_comp_m_y = msg->flow_comp_m_y;
		flow_msg.quality = msg->quality;
		flow_msg.ground_distance = msg->ground_distance;
        	flow_msg.flow_rate_x = msg->flow_rate_x;
        	flow_msg.flow_rate_y = msg->flow_rate_y;

		UAS_FCU(m_uas)->send_message_ignore_drop(flow_msg);
	}
};
}
}	

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::PX4FlowPlugin, mavros::plugin::PluginBase)
