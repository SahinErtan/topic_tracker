/**
 * @file /include/topic_tracker/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef topic_tracker_QNODE_HPP_
#define topic_tracker_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/master.h>
#endif
#include <QThread>
#include <QStringListModel>

#include <string>
#include <vector>
#include <unordered_map>

#include <topic_tools/shape_shifter.h>
#include <std_msgs/Header.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace topic_tracker {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	void getCurrentTopicList();
	void subscribe(std::string const& topic);
	
	std::unordered_map<std::string, double> topic_hz_map;
	std::unordered_map<std::string, double> topic_bw_map;
	std::unordered_map<std::string, double> topic_delay_map;
	std::unordered_map<std::string, double> topic_cpu_usage_map;

	std::unordered_map<std::string, std::vector<ros::Time>> topic_timestamp_map;
	std::unordered_map<std::string, ros::Time> topic_last_timestamp_map;


	std::unordered_map<std::string, std::vector<double>> topic_time_diff_map;
	std::unordered_map<std::string, double> topic_time_diff_sum_map;

	std::unordered_map<std::string, std::vector<uint64_t>> topic_bw_size_map;
	std::unordered_map<std::string, uint64_t> topic_bw_sum_map;

	std::unordered_map<std::string, std::vector<double>> topic_delay_list_map;
	std::unordered_map<std::string, double> topic_delay_sum_map;

	std::unordered_map<std::string, int> topic_monitor_window_size_limit;

	std::vector<ros::master::TopicInfo> currently_monitored_topic_vector;

	std::unordered_map<std::string, bool> topic_header_stat_map;
	std::unordered_map<std::string, bool> topic_header_stamp_filled_stat_map;

	
public Q_SLOTS:
	void topicMonitorUpdate();

Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::NodeHandle *nh;
	std::vector<std::string> subscribed_topic_list;

	void doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, 
            std::string const& topic, boost::shared_ptr<ros::Subscriber> subscriber);
	
};

}  // namespace topic_tracker

#endif /* topic_tracker_QNODE_HPP_ */
