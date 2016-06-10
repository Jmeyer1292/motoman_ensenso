#ifndef ENSENSO_PANEL_H
#define ENSENSO_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include <rviz/panel.h>
#endif

#include <grinding_ensenso_extrinsic_calibration/PerformEnsensoCalibration.h> // Forward declaration impossible
#include <grinding_ensenso_extrinsic_calibration/TestEnsensoCalibration.h>

class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;

namespace ensenso_rviz_plugin
{
/**
 * RViz panel for the Ensenso calibration
 */
class EnsensoPanel : public rviz::Panel
{
Q_OBJECT
public:
  /**
   * Default constructor
   * @param parent
   */
  EnsensoPanel(QWidget* parent = 0);

  /**
   * Load all configuration data for this panel from the given Config object
   * @param config
   */
  virtual void load(const rviz::Config& config);

  /**
   * Save all configuration data from this panel to the given Config object
   * @param config
   */
  virtual void save(rviz::Config config) const;

Q_SIGNALS:
  /**
   * Enable or disable the whole panel
   * @param[in] True to enable, false to disable
   */
  void enablePanel(bool);
  /**
   * Upates the \ref status_ label with a message
   * @param[in] the message to be displayed
   */
  void newStatus(const QString);

public Q_SLOTS:
  /**
   * Sends a message through \reset_calib_pub_ to rest the extrinsic calibration
   */
  virtual void resetCalibration();
  /**
   * Sends a service call in a separate thread to start calibration.
   * The GUI is grayed until the service call returns.
   */
  virtual void startCalibration();
  /**
   * Sends a service call in a separate thread to test the calibration.
   * The GUI is grayed until the service call returns.
   */
  virtual void testCalibration();

protected Q_SLOTS:
  /**
   * Default RViz panel member, trigger user parameter saving
   */
  virtual void triggerSave();
  /**
   * Connects to the services, subscribers and wait for publishers.
   * The GUI is grayed until all connections are successful.
   */
  void connectToServicesSubscribersPublishers();
  /**
   * Fills the service request for the calibration and starts \ref startCalibration in a separate thread.
   */
  void startCalibrationButtonHandler();
  /**
   * Enable or disable the whole panel
   * @param[in] True to enable, false to disable
   */
  void enablePanelHandler(bool);
  /**
   * Handles message received by the @ref calib_status_sub_ subscriber
   * @param[in] msg the message
   */
  void newCalibrationStatusMessage(const std_msgs::String::ConstPtr& msg);
  /**
   * Handles updating the GUI with new messages
   * @param[in] the message
   */
  void newStatusHandler(const QString);
  void testCalibrationButtonHandler();

protected:
  /** Node handle */
  ros::NodeHandle nh_;
  /** Calibration service client */
  ros::ServiceClient calib_client_;
  /** Test calibration service client */
  ros::ServiceClient test_client_;
  /** PerformEnsensoCalibration service */
  grinding_ensenso_extrinsic_calibration::PerformEnsensoCalibration perform_calib_srv_;
  /** TestEnsensoCalibration service */
  grinding_ensenso_extrinsic_calibration::TestEnsensoCalibration test_calib_srv_;
  /** Calibration status subscriber */
  ros::Subscriber calib_status_sub_;
  /** Reset calibration publisher */
  ros::Publisher reset_calib_pub_;

  QSpinBox* number_of_poses_;
  QDoubleSpinBox* grid_spacing_;
  QDoubleSpinBox* calibration_plate_distance_;
  QCheckBox* store_to_eeprom_;
  QPushButton* start_calibration_, *reset_calibration_, *test_calibration_;
  QLabel* status_;
};

}  // end namespace

#endif // TELEOP_PANEL_H
