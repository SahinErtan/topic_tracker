/**
 * @file /include/topic_tracker/main_window.hpp
 *
 * @brief Qt based gui for topic_tracker.
 *
 * @date November 2010
 **/
#ifndef topic_tracker_MAIN_WINDOW_H
#define topic_tracker_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <QFileDialog>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace topic_tracker {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	void on_save_topic_monitor_report_button_clicked(bool click);

	void topicMonitorUpdate();
	void topicMonitorSearchFilter();

Q_SIGNALS:
	void checkNewTopics_EMIT();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

	QTimer *topic_monitor_update_timer;

	QString formatBwValueToText(double value);
	QString formatDelayValueToText(std::string topic, double value);

};

}  // namespace topic_tracker

#endif // topic_tracker_MAIN_WINDOW_H
