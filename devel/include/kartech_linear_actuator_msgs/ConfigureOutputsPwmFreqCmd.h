// Generated by gencpp from file kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd.msg
// DO NOT EDIT!


#ifndef KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_CONFIGUREOUTPUTSPWMFREQCMD_H
#define KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_CONFIGUREOUTPUTSPWMFREQCMD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace kartech_linear_actuator_msgs
{
template <class ContainerAllocator>
struct ConfigureOutputsPwmFreqCmd_
{
  typedef ConfigureOutputsPwmFreqCmd_<ContainerAllocator> Type;

  ConfigureOutputsPwmFreqCmd_()
    : header()
    , confirm(false)
    , min_pwm_pct(0)
    , max_pwm_pct(0)
    , pwm_freq(0)  {
    }
  ConfigureOutputsPwmFreqCmd_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , confirm(false)
    , min_pwm_pct(0)
    , max_pwm_pct(0)
    , pwm_freq(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _confirm_type;
  _confirm_type confirm;

   typedef uint8_t _min_pwm_pct_type;
  _min_pwm_pct_type min_pwm_pct;

   typedef uint8_t _max_pwm_pct_type;
  _max_pwm_pct_type max_pwm_pct;

   typedef uint16_t _pwm_freq_type;
  _pwm_freq_type pwm_freq;





  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigureOutputsPwmFreqCmd_

typedef ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<std::allocator<void> > ConfigureOutputsPwmFreqCmd;

typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd > ConfigureOutputsPwmFreqCmdPtr;
typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd const> ConfigureOutputsPwmFreqCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.confirm == rhs.confirm &&
    lhs.min_pwm_pct == rhs.min_pwm_pct &&
    lhs.max_pwm_pct == rhs.max_pwm_pct &&
    lhs.pwm_freq == rhs.pwm_freq;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kartech_linear_actuator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "177ba95b80ad87cfd885201c32903f9c";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x177ba95b80ad87cfULL;
  static const uint64_t static_value2 = 0xd885201c32903f9cULL;
};

template<class ContainerAllocator>
struct DataType< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"bool confirm\n"
"uint8 min_pwm_pct   # The minimum motor drive duty cycle in percent (0-100). Default is 20%.\n"
"uint8 max_pwm_pct   # The maximum motor drive duty cycle in percent (0-100). Default is 90%.\n"
"uint16 pwm_freq     # The frequency of the PWM outputs in Hz. Default is 2000Hz.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.confirm);
      stream.next(m.min_pwm_pct);
      stream.next(m.max_pwm_pct);
      stream.next(m.pwm_freq);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigureOutputsPwmFreqCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "confirm: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.confirm);
    s << indent << "min_pwm_pct: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.min_pwm_pct);
    s << indent << "max_pwm_pct: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.max_pwm_pct);
    s << indent << "pwm_freq: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.pwm_freq);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_CONFIGUREOUTPUTSPWMFREQCMD_H
