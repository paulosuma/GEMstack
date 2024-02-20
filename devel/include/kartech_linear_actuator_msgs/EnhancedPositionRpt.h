// Generated by gencpp from file kartech_linear_actuator_msgs/EnhancedPositionRpt.msg
// DO NOT EDIT!


#ifndef KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_ENHANCEDPOSITIONRPT_H
#define KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_ENHANCEDPOSITIONRPT_H


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
struct EnhancedPositionRpt_
{
  typedef EnhancedPositionRpt_<ContainerAllocator> Type;

  EnhancedPositionRpt_()
    : header()
    , shaft_extension(0.0)
    , motor_overload_error(false)
    , clutch_overload_error(false)
    , motor_open_load_error(false)
    , clutch_open_load_error(false)
    , position_reach_error(false)
    , hardware_warning_1_error(false)
    , hardware_warning_2_error(false)
    , motor_current(0)  {
    }
  EnhancedPositionRpt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , shaft_extension(0.0)
    , motor_overload_error(false)
    , clutch_overload_error(false)
    , motor_open_load_error(false)
    , clutch_open_load_error(false)
    , position_reach_error(false)
    , hardware_warning_1_error(false)
    , hardware_warning_2_error(false)
    , motor_current(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _shaft_extension_type;
  _shaft_extension_type shaft_extension;

   typedef uint8_t _motor_overload_error_type;
  _motor_overload_error_type motor_overload_error;

   typedef uint8_t _clutch_overload_error_type;
  _clutch_overload_error_type clutch_overload_error;

   typedef uint8_t _motor_open_load_error_type;
  _motor_open_load_error_type motor_open_load_error;

   typedef uint8_t _clutch_open_load_error_type;
  _clutch_open_load_error_type clutch_open_load_error;

   typedef uint8_t _position_reach_error_type;
  _position_reach_error_type position_reach_error;

   typedef uint8_t _hardware_warning_1_error_type;
  _hardware_warning_1_error_type hardware_warning_1_error;

   typedef uint8_t _hardware_warning_2_error_type;
  _hardware_warning_2_error_type hardware_warning_2_error;

   typedef uint16_t _motor_current_type;
  _motor_current_type motor_current;





  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> const> ConstPtr;

}; // struct EnhancedPositionRpt_

typedef ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<std::allocator<void> > EnhancedPositionRpt;

typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::EnhancedPositionRpt > EnhancedPositionRptPtr;
typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::EnhancedPositionRpt const> EnhancedPositionRptConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.shaft_extension == rhs.shaft_extension &&
    lhs.motor_overload_error == rhs.motor_overload_error &&
    lhs.clutch_overload_error == rhs.clutch_overload_error &&
    lhs.motor_open_load_error == rhs.motor_open_load_error &&
    lhs.clutch_open_load_error == rhs.clutch_open_load_error &&
    lhs.position_reach_error == rhs.position_reach_error &&
    lhs.hardware_warning_1_error == rhs.hardware_warning_1_error &&
    lhs.hardware_warning_2_error == rhs.hardware_warning_2_error &&
    lhs.motor_current == rhs.motor_current;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kartech_linear_actuator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b5d14804230789155d91f65364c956fd";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb5d1480423078915ULL;
  static const uint64_t static_value2 = 0x5d91f65364c956fdULL;
};

template<class ContainerAllocator>
struct DataType< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kartech_linear_actuator_msgs/EnhancedPositionRpt";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float64 shaft_extension     # The current shaft position in 0.001\" increments.\n"
"bool motor_overload_error\n"
"bool clutch_overload_error\n"
"bool motor_open_load_error\n"
"bool clutch_open_load_error\n"
"bool position_reach_error\n"
"bool hardware_warning_1_error\n"
"bool hardware_warning_2_error\n"
"uint16 motor_current        # The current motor current in mA.\n"
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

  static const char* value(const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.shaft_extension);
      stream.next(m.motor_overload_error);
      stream.next(m.clutch_overload_error);
      stream.next(m.motor_open_load_error);
      stream.next(m.clutch_open_load_error);
      stream.next(m.position_reach_error);
      stream.next(m.hardware_warning_1_error);
      stream.next(m.hardware_warning_2_error);
      stream.next(m.motor_current);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EnhancedPositionRpt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kartech_linear_actuator_msgs::EnhancedPositionRpt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "shaft_extension: ";
    Printer<double>::stream(s, indent + "  ", v.shaft_extension);
    s << indent << "motor_overload_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motor_overload_error);
    s << indent << "clutch_overload_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.clutch_overload_error);
    s << indent << "motor_open_load_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motor_open_load_error);
    s << indent << "clutch_open_load_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.clutch_open_load_error);
    s << indent << "position_reach_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.position_reach_error);
    s << indent << "hardware_warning_1_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hardware_warning_1_error);
    s << indent << "hardware_warning_2_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hardware_warning_2_error);
    s << indent << "motor_current: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.motor_current);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_ENHANCEDPOSITIONRPT_H
