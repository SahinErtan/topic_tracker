/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"
#include "tracker.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace topic_tracker {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
	
    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	ui.dock_status->setVisible(false);

	topic_monitor_update_timer = new QTimer(this);

	QObject::connect(topic_monitor_update_timer, SIGNAL(timeout()), this, SLOT(topicMonitorUpdate()));
	QObject::connect(this, SIGNAL(checkNewTopics_EMIT()), &qnode, SLOT(topicMonitorUpdate()));
	QObject::connect(ui.topic_monitoring_search_textEdit, SIGNAL(textChanged()), this, SLOT(topicMonitorSearchFilter()));
	
	
	topic_monitor_update_timer->start(1000);
	
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "topic_tracker");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(true);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(true);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "topic_tracker");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::topicMonitorUpdate() {
	qnode.getCurrentTopicList();
	if (ui.topic_monitoring_tableWidget->rowCount() != qnode.currently_monitored_topic_vector.size()) {
		ui.topic_monitoring_tableWidget->setRowCount(qnode.currently_monitored_topic_vector.size());
	}

	double max_topic_hz = 0.0;
	double max_topic_bw = 0.0;

	double total_topic_hz = 0.0;
	double total_topic_bw = 0.0;

	for (int i = 0; i < qnode.currently_monitored_topic_vector.size(); i++) {
		ui.topic_monitoring_tableWidget->setItem(i,0,new QTableWidgetItem(QString::fromStdString(qnode.currently_monitored_topic_vector[i].name)));
		ui.topic_monitoring_tableWidget->setItem(i,1,new QTableWidgetItem(QString::fromStdString(qnode.currently_monitored_topic_vector[i].datatype)));
		ui.topic_monitoring_tableWidget->setItem(i,2,new QTableWidgetItem(QString::number(qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name], 'G', 4)));
		ui.topic_monitoring_tableWidget->setItem(i,3,new QTableWidgetItem(formatBwValueToText(qnode.topic_bw_map[qnode.currently_monitored_topic_vector[i].name])));
		ui.topic_monitoring_tableWidget->setItem(i,4,new QTableWidgetItem(formatDelayValueToText(qnode.currently_monitored_topic_vector[i].name, qnode.topic_delay_map[qnode.currently_monitored_topic_vector[i].name])));
		ui.topic_monitoring_tableWidget->setItem(i,5,new QTableWidgetItem(QString::number(qnode.topic_time_diff_map[qnode.currently_monitored_topic_vector[i].name].size())));

		// if (qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name] > 10000) {
		// 	ROS_WARN_STREAM(qnode.currently_monitored_topic_vector[i].name);
		// 	ROS_WARN_STREAM(qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name]);
		// }

		total_topic_hz += qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name];
		total_topic_bw += qnode.topic_bw_map[qnode.currently_monitored_topic_vector[i].name];

		if (max_topic_hz < qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name]) {
			max_topic_hz = qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name];
		}

		if (max_topic_bw < qnode.topic_bw_map[qnode.currently_monitored_topic_vector[i].name]) {
			max_topic_bw = qnode.topic_bw_map[qnode.currently_monitored_topic_vector[i].name];
		}


		ui.topic_tracker_total_hz_label->setText(QString::number(total_topic_hz, 'f', 2));
		ui.topic_tracker_max_hz_label->setText(QString::number(max_topic_hz, 'f', 2));


		ui.topic_tracker_total_bw_label->setText(formatBwValueToText(total_topic_bw));
		ui.topic_tracker_max_bw_label->setText(formatBwValueToText(max_topic_bw));
		
	}

	// ui.topic_monitoring_tableWidget->sortByColumn(ui.topic_monitoring_tableWidget->horizontalHeader()->sortIndicatorSection(), ui.topic_monitoring_tableWidget->horizontalHeader()->sortIndicatorOrder());
	ui.topic_monitoring_tableWidget->sortByColumn(0, Qt::AscendingOrder);
	topicMonitorSearchFilter();
}

void MainWindow::topicMonitorSearchFilter() {
	QString filter_str = ui.topic_monitoring_search_textEdit->toPlainText().toLower();
	for (int i = 0; i < ui.topic_monitoring_tableWidget->rowCount(); i++) {
		if (ui.topic_monitoring_tableWidget->item(i,0)->text().toLower().contains(filter_str)) {
			ui.topic_monitoring_tableWidget->setRowHidden(i, false);
		} else {
			ui.topic_monitoring_tableWidget->setRowHidden(i, true);
		}
	}
}

QString MainWindow::formatBwValueToText(double value) {
	if (value < 1000.0) {
		return QString::number(value, 'f', 2) + QString(" B/sec");
	} else if (value < 1000000.0) {
		return QString::number(value/1000.0, 'f', 2) + QString(" KB/sec");
	} else {
		return QString::number(value/1000000.0, 'f', 2) + QString(" MB/sec");
	}
}

QString MainWindow::formatDelayValueToText(std::string topic, double value) {
	if (qnode.topic_time_diff_map[topic].size() == 0) {
		return QString("Message Waiting...");
	} else if (!qnode.topic_header_stat_map[topic]) {
		return QString("Message does not have header");
	} else if (!qnode.topic_header_stamp_filled_stat_map[topic]) {
		return QString("Message have header but it's not filled");
	} else {
		return QString::number(value, 'f', 4);
	}
}

void MainWindow::on_save_topic_monitor_report_button_clicked(bool click) {
	QString filename = QFileDialog::getSaveFileName(this, "DialogTitle", "filename.csv", "CSV files (.csv);;Zip files (.zip, *.7z)", 0, 0); // getting the filename (full path)
	QFile data(filename);
	if(data.open(QFile::WriteOnly |QFile::Truncate))
	{
		QTextStream output(&data);
		output << "Name,Type,Hertz,Bandwidth,Delay,Window Size\n";
		for (int i = 0; i < qnode.currently_monitored_topic_vector.size(); i++) {
			size_t pos = 0;
			std::string topic_name = qnode.currently_monitored_topic_vector[i].name;
			std::string topic_type = qnode.currently_monitored_topic_vector[i].datatype;
			std::string topic_hz = std::to_string(qnode.topic_hz_map[qnode.currently_monitored_topic_vector[i].name]);
			pos = topic_hz.find(",");
			if (pos != std::string::npos){
				topic_hz = topic_hz.replace(pos, 1, ".");
			}
			std::string topic_bw = std::to_string(qnode.topic_bw_map[qnode.currently_monitored_topic_vector[i].name]);
			pos = topic_bw.find(",");
			if (pos != std::string::npos){
				topic_bw = topic_bw.replace(pos, 1, ".");
			}
			std::string topic_delay = formatDelayValueToText(qnode.currently_monitored_topic_vector[i].name, qnode.topic_delay_map[qnode.currently_monitored_topic_vector[i].name]).toStdString();
			pos = topic_delay.find(",");
			if (pos != std::string::npos){
				topic_delay = topic_delay.replace(pos, 1, ".");
			}

			std::string topic_window_size = std::to_string(qnode.topic_time_diff_map[qnode.currently_monitored_topic_vector[i].name].size());

			std::string text = topic_name + "," + topic_type + "," + topic_hz + "," + topic_bw + "," + topic_delay + "," + topic_window_size;
			output << text.c_str() << "\n";
		}

		output << ",,Total Hertz:," << ui.topic_tracker_total_hz_label->text().replace(",",".").toStdString().c_str() << ",\n"; 
		output << ",,Total Bandwidth:," << ui.topic_tracker_total_bw_label->text().replace(",",".").toStdString().c_str() << ",\n"; 
		output << ",,Maximum Hertz:," << ui.topic_tracker_max_hz_label->text().replace(",",".").toStdString().c_str() << ",\n"; 
		output << ",,Maximum Bandwidth:," << ui.topic_tracker_max_bw_label->text().replace(",",".").toStdString().c_str() << ",\n"; 
	}
	data.close();
}

}  // namespace topic_tracker

