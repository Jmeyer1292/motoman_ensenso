#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QMessageBox>
#include <QFuture>
#include <QtConcurrentRun>

#include "ensenso_panel.h"
#include <ros/publisher.h>
#include <grinding_ensenso_extrinsic_calibration/PerformEnsensoCalibration.h>
#include <grinding_ensenso_extrinsic_calibration/TestEnsensoCalibration.h>

namespace ensenso_rviz_plugin
{
EnsensoPanel::EnsensoPanel(QWidget* parent) :
    rviz::Panel(parent)
{
  // Create UI and layout elements
  QHBoxLayout* calib_pose_count_layout = new QHBoxLayout;
  calib_pose_count_layout->addWidget(new QLabel("Number of poses:"));
  number_of_poses_ = new QSpinBox;
  number_of_poses_->setMinimum(2);
  number_of_poses_->setMaximum(30); // More would really be useless
  number_of_poses_->setValue(10);
  calib_pose_count_layout->addWidget(number_of_poses_);

  QHBoxLayout* calib_pattern_size_layout = new QHBoxLayout;
  calib_pattern_size_layout->addWidget(new QLabel("Pattern grid spacing:"));
  grid_spacing_ = new QDoubleSpinBox;
  grid_spacing_->setSuffix(" mm");
  grid_spacing_->setMinimum(1.0);
  grid_spacing_->setValue(5.0);
  calib_pattern_size_layout->addWidget(grid_spacing_);

  QHBoxLayout* calib_pattern_distance_layout = new QHBoxLayout;
  calib_pattern_distance_layout->addWidget(new QLabel("Calibration pattern distance:"));
  calibration_plate_distance_ = new QDoubleSpinBox;
  calibration_plate_distance_->setSuffix(" mm");
  calibration_plate_distance_->setDecimals(0);
  calibration_plate_distance_->setMinimum(60.0);
  calibration_plate_distance_->setMaximum(1500.0);
  calibration_plate_distance_->setSingleStep(10.0);
  calibration_plate_distance_->setValue(300.0);
  calib_pattern_distance_layout->addWidget(calibration_plate_distance_);

  store_to_eeprom_ = new QCheckBox;
  store_to_eeprom_->setText("Store calibration matrix to EEPROM");
  store_to_eeprom_->setChecked(true);

  QHBoxLayout* start_erase_layout = new QHBoxLayout;
  start_calibration_ = new QPushButton("Start calibration");
  reset_calibration_ = new QPushButton("Erase calibration (EEPROM)");
  start_erase_layout->addWidget(start_calibration_);
  start_erase_layout->addWidget(reset_calibration_);

  test_calibration_ = new QPushButton ("Test calibration");

  QVBoxLayout* status_layout = new QVBoxLayout;
  status_layout->addWidget(new QLabel ("Status:"));
  status_ = new QLabel;
  status_layout->addWidget(status_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(calib_pose_count_layout);
  layout->addLayout(calib_pattern_size_layout);
  layout->addLayout(calib_pattern_distance_layout);
  layout->addWidget(store_to_eeprom_);
  layout->addStretch(1);
  layout->addLayout(start_erase_layout);
  layout->addWidget(test_calibration_);
  layout->addStretch(2);
  layout->addLayout(status_layout);
  setLayout(layout);

  // Connect UI elements to update the configuration file
  connect(number_of_poses_, SIGNAL(editingFinished()), this, SLOT(triggerSave()));
  connect(grid_spacing_, SIGNAL(editingFinished()), this, SLOT(triggerSave()));
  connect(calibration_plate_distance_, SIGNAL(editingFinished()), this, SLOT(triggerSave()));
  connect(store_to_eeprom_, SIGNAL(stateChanged(int)), this, SLOT(triggerSave()));

  // Connect handlers
  connect(start_calibration_, SIGNAL(released()), this, SLOT(startCalibrationButtonHandler()));
  connect(reset_calibration_, SIGNAL(released()), this, SLOT(resetCalibration()));
  connect(test_calibration_, SIGNAL(released()), this, SLOT(testCalibrationButtonHandler()));
  connect(this, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  connect(this, SIGNAL(newStatus(QString)),
          this, SLOT(newStatusHandler(const QString)));

  // Setup subscribers
  calib_status_sub_ = nh_.subscribe("ensenso_calibration_status", 1, &EnsensoPanel::newCalibrationStatusMessage, this);
  // Setup publishers
  reset_calib_pub_ = nh_.advertise<std_msgs::String>("reset_ensenso_calibration", 1);
  // Setup clients
  calib_client_ = nh_.serviceClient<grinding_ensenso_extrinsic_calibration::PerformEnsensoCalibration>("perform_ensenso_calibration");
  test_client_ = nh_.serviceClient<grinding_ensenso_extrinsic_calibration::TestEnsensoCalibration>("test_ensenso_calibration");
  QFuture<void> future = QtConcurrent::run(this, &EnsensoPanel::connectToServicesSubscribersPublishers); // Launch in it's own thread
}

void EnsensoPanel::newStatusHandler(const QString msg)
{
  status_->setText(msg);
}

void EnsensoPanel::newCalibrationStatusMessage(const std_msgs::String::ConstPtr& msg)
{
  Q_EMIT newStatusHandler(QString::fromStdString(msg->data));
}

void EnsensoPanel::connectToServicesSubscribersPublishers()
{
  Q_EMIT enablePanel(false);

  // Check calib_client_ connection
  Q_EMIT newStatus("Connecting to services");
  while (ros::ok())
  {
    if (calib_client_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel connected to the service " << calib_client_.getService());
      break;
    }
    else
    {
      ROS_WARN_STREAM("RViz panel could not connect to ROS service:\n\t" << calib_client_.getService());
      sleep(1);
    }
  }
  while (ros::ok())
  {
    if (test_client_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel connected to the service " << test_client_.getService());
      break;
    }
    else
    {
      ROS_WARN_STREAM("RViz panel could not connect to ROS service:\n\t" << test_client_.getService());
      sleep(1);
    }
  }

  // Check calib_status_sub_ number of publishers
  Q_EMIT newStatus("Checking subscriber publisher count");
  while (ros::ok())
  {
    if (calib_status_sub_.getNumPublishers() > 0)
    {
      ROS_INFO_STREAM(calib_status_sub_.getTopic() << " has at least 1 publisher");
      break;
    }
    else
    {
      ROS_WARN_STREAM(calib_status_sub_.getTopic() << " has no publisher");
      sleep(1);
    }
  }

  // Check reset_calib_pub_ subscribers count
  Q_EMIT newStatus("Checking subscribers count");
  while (ros::ok())
  {
    if (reset_calib_pub_.getNumSubscribers() > 0)
    {
      ROS_INFO_STREAM(reset_calib_pub_.getTopic() << " has at least 1 subscriber");
      break;
    }
    else
    {
      ROS_WARN_STREAM(reset_calib_pub_.getTopic() << " has no subscriber");
      sleep(1);
    }
  }

  ROS_WARN_STREAM("All services/subscribers/publishers connections have been made");
  Q_EMIT newStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

void EnsensoPanel::enablePanelHandler(bool enable)
{
  this->setEnabled(enable);
}

void EnsensoPanel::startCalibrationButtonHandler()
{
  if (number_of_poses_->value() < 5)
  {
    QMessageBox msgBox;
    msgBox.setWindowTitle("Number of poses is too small");
    msgBox.setText("EnsensoSDK requires at least 5 poses to compute extrinsic calibration.\n"
            "The value will be changed to 5.");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
    number_of_poses_->setValue(5);
  }

  perform_calib_srv_.request.number_of_poses = number_of_poses_->value();
  perform_calib_srv_.request.grid_spacing = grid_spacing_->value();
  perform_calib_srv_.request.calibration_plate_distance = calibration_plate_distance_->value();
  perform_calib_srv_.request.store_to_EEPROM = store_to_eeprom_->isChecked();
  QFuture<void> future = QtConcurrent::run(this, &EnsensoPanel::startCalibration);
}

void EnsensoPanel::startCalibration()
{
  Q_EMIT enablePanel(false); // Grey UI until calibration is done

  // Call service to calibrate the robot
  calib_client_.call(perform_calib_srv_);
  if (!perform_calib_srv_.response.return_status)
    ROS_ERROR_STREAM(perform_calib_srv_.response.return_message);

  // Display calibration result and re-enable UI
  Q_EMIT enablePanel(true); // Make UI available again
}

void EnsensoPanel::resetCalibration()
{
  // Publish empty message to trigger reset of extrinsic calibration
  std_msgs::String msg;
  msg.data = "";
  reset_calib_pub_.publish(msg);
}

void EnsensoPanel::testCalibrationButtonHandler()
{
  if (number_of_poses_->value() > 4)
  {
    QMessageBox msgBox;
    msgBox.setWindowTitle("Number of poses is big");
    msgBox.setText("Displaying a lot of point clouds can significantly slow down RViz, do you want to continue?");
    msgBox.setStandardButtons(QMessageBox::Ok|QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    if (msgBox.exec() == QMessageBox::Cancel)
      return; // Abort
  }

  test_calib_srv_.request.number_of_poses = number_of_poses_->value();
  test_calib_srv_.request.calibration_plate_distance = calibration_plate_distance_->value();
  QFuture<void> future = QtConcurrent::run(this, &EnsensoPanel::testCalibration);
}

void EnsensoPanel::testCalibration()
{
  Q_EMIT enablePanel(false); // Grey UI until calibration is done

  // Call service to calibrate the robot
  test_client_.call(test_calib_srv_);
  if (!test_calib_srv_.response.return_status)
    ROS_ERROR_STREAM(test_calib_srv_.response.return_message);

  // Display calibration result and re-enable UI
  Q_EMIT enablePanel(true); // Make UI available again
}

void EnsensoPanel::triggerSave()
{
  Q_EMIT configChanged();
}

void EnsensoPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("number_of_poses", number_of_poses_->value());
  config.mapSetValue("grid_spacing", grid_spacing_->value());
  config.mapSetValue("calibration_plate_distance", calibration_plate_distance_->value());
  config.mapSetValue("store_to_eeprom", store_to_eeprom_->isChecked());
}

void EnsensoPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  float tmp;
  if (config.mapGetFloat("number_of_poses", &tmp))
    number_of_poses_->setValue(tmp);
  if (config.mapGetFloat("grid_spacing", &tmp))
    grid_spacing_->setValue(tmp);
  if (config.mapGetFloat("calibration_plate_distance", &tmp))
    calibration_plate_distance_->setValue(tmp);
  bool store_eeprom;
  if (config.mapGetBool("store_to_eeprom", &store_eeprom))
    store_to_eeprom_->setChecked(store_eeprom);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ensenso_rviz_plugin::EnsensoPanel, rviz::Panel)
