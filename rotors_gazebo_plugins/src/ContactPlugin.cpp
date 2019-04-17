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

  bool collision = false;
  bool targetreached = false;
  int size = 0;
  msgs::Contacts contacts;


  contacts = this->parentSensor->Contacts();
  size = contacts.contact_size();
 
  for (unsigned int i = 0; i < size; ++i)
  {
      std::string s1 = contacts.contact(i).collision1();
      // ROS_INFO("%s",s1.c_str());
      if (s1.find("target") != std::string::npos)
      {
        targetreached = true;
      }
      else
      {
        collision = true;
        targetreached = false;
        break;
      }
  }


  if(size > 0 && !collision)
    ros::param::set("/bebop2/targetreached",targetreached);

  if(collision == true)
    ros::param::set("/bebop2/collision",collision);


  // std_msgs::Int16 message;
  // bool collision = false;

  // message.data = this->parentSensor->Contacts().contact_size();

  // if(message.data >0)
  //   collision = true;

  // if(collision == true)  
  //   ros::param::set("/bebop2/collision",collision);

}