/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace topic_tracker {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"topic_tracker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"topic_tracker");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
	ros::NodeHandle n;
	
	start();
	return true;
}

void QNode::run() {
	
	ros::Rate loop_rate(1);
	nh = new ros::NodeHandle();
	
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown();
}

void QNode::getCurrentTopicList() {
	ros::master::V_TopicInfo topic_list;
	ros::master::getTopics(topic_list);
	for (ros::master::V_TopicInfo::iterator it = topic_list.begin() ; it != topic_list.end(); it++) {
		const ros::master::TopicInfo& info = *it;
		if (std::find(subscribed_topic_list.begin(), subscribed_topic_list.end(), info.name) == subscribed_topic_list.end()) {
			subscribe(info.name);
			currently_monitored_topic_vector.push_back(info);
		}
	}
}

void QNode::subscribe(std::string const& topic) 
    {
		boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());

		ros::SubscribeOptions ops;
		ops.topic = topic;
		ops.queue_size = 1000;
		ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
		ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
		ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
			const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
				boost::bind(&QNode::doQueue, this, _1, topic, sub));

		ops.transport_hints = ops.transport_hints;
		*sub = nh->subscribe(ops);

		topic_last_timestamp_map[topic] = ros::Time(0.0);
		topic_monitor_window_size_limit[topic] = 0;

		subscribed_topic_list.push_back(topic);
}   

void QNode::doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, 
     			    std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber) 
    { 

		const topic_tools::ShapeShifter::ConstPtr& message = msg_event.getConstMessage();

		if (topic_header_stat_map[topic] != true) {
			const std::string message_definition = message->getMessageDefinition();

			if (message_definition.find("std_msgs/Header") != std::string::npos) {
				topic_header_stat_map[topic] = true;
			} else {
				topic_header_stat_map[topic] = false;
			}
		}
		
		ros::Time message_timestamp;
		ros::Time timestamp = msg_event.getReceiptTime();
		
		if (topic_monitor_window_size_limit[topic] != 0 && topic_time_diff_map[topic].size() >= topic_monitor_window_size_limit[topic]) {
			topic_timestamp_map[topic].erase(topic_timestamp_map[topic].begin());

			topic_time_diff_sum_map[topic] = topic_time_diff_sum_map[topic] - topic_time_diff_map[topic].at(0); 
			topic_time_diff_map[topic].erase(topic_time_diff_map[topic].begin());

			topic_bw_sum_map[topic] = topic_bw_sum_map[topic] - topic_bw_size_map[topic].at(0);
			topic_bw_size_map[topic].erase(topic_bw_size_map[topic].begin());

			topic_delay_sum_map[topic] = topic_delay_sum_map[topic] - topic_delay_list_map[topic].at(0);
			topic_delay_list_map[topic].erase(topic_delay_list_map[topic].begin());
		}

		if(topic_last_timestamp_map[topic] != ros::Time(0.0)){

			topic_timestamp_map[topic].push_back(timestamp);

			// Hz Calculate
			double time_diff = (timestamp - topic_last_timestamp_map[topic]).toSec();
			topic_time_diff_map[topic].push_back(time_diff);
			topic_time_diff_sum_map[topic] += time_diff;
		
			//BW Calculate
			uint64_t topic_msg_size = msg_event.getMessage()->size();
			topic_bw_size_map[topic].push_back(topic_msg_size);
			topic_bw_sum_map[topic] += topic_msg_size;

			// Delay Calculate
			if(topic_header_stat_map[topic] == true) {
				std_msgs::Header header;
				uint8_t buf[message->size()];
				ros::serialization::OStream stream(buf, message->size());
				message->write(stream);
				header.stamp.sec = ((uint32_t *)buf)[1];
				header.stamp.nsec = ((uint32_t *)buf)[2];
				message_timestamp = header.stamp;

				topic_header_stamp_filled_stat_map[topic] = true;

				double current_delay = (message_timestamp - timestamp).toSec();
				topic_delay_list_map[topic].push_back(current_delay);
				topic_delay_sum_map[topic] += current_delay;
			}
		} else {
			topic_header_stamp_filled_stat_map[topic] = false;
		}



		if (topic_time_diff_map[topic].size() > 5) {
			
			topic_hz_map[topic] = 1.0 / (topic_time_diff_sum_map[topic] / topic_time_diff_map[topic].size());
			
			double time_total_diff = (topic_timestamp_map[topic].at(topic_timestamp_map[topic].size() - 1) - topic_timestamp_map[topic].at(0)).toSec();
			topic_bw_map[topic] = topic_bw_sum_map[topic] / time_total_diff;

			if(topic_header_stat_map[topic] == true) {
				topic_delay_map[topic] = topic_delay_sum_map[topic] / topic_delay_list_map[topic].size();
			}
		}
		
		topic_last_timestamp_map[topic] = timestamp;
		       
	}

	void QNode::topicMonitorUpdate() {
		getCurrentTopicList();
	}

}  // namespace topic_tracker
