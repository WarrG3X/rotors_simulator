#include "rotors_gazebo_plugins/ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{

    

    if (!ros::isInitialized())
    {
      ROS_INFO("ROS should be initialized first!");
      return;
    }

   if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
  
  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  ROS_INFO("The Contact plugin has been loaded!");

  node_handle_ = new ros::NodeHandle("");

  // this->publisher_topic_name = _sdf->GetElement("TopicName")->GetValue()->GetAsString() ;

  // ROS_INFO("%s",this->publisher_topic_name.c_str());

  // this->publisher_topic_name = "/bebop2/base_link/contact_sensor";
  // this->_publisher= this->node_handle_->advertise<std_msgs::Int16>(this->publisher_topic_name.c_str(), 2);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // this->parentSensor->SetUpdateRate(10);
  
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  std_msgs::Int16 message;
  bool collision = false;

  message.data = this->parentSensor->Contacts().contact_size();

  if(message.data >0)
    collision = true;

  if(collision == true)  
    ros::param::set("/bebop2/lee_position_controller_node/collision",collision);

}