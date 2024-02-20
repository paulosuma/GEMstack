// Generated by gencpp from file kartech_linear_actuator_msgs/UniqueDeviceIdRpt.msg
// DO NOT EDIT!


#ifndef KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_UNIQUEDEVICEIDRPT_H
#define KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_UNIQUEDEVICEIDRPT_H


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
struct UniqueDeviceIdRpt_
{
  typedef UniqueDeviceIdRpt_<ContainerAllocator> Type;

  UniqueDeviceIdRpt_()
    : header()
    , actuator_id_first_6(0)
    , actuator_id_last_6(0)  {
    }
  UniqueDeviceIdRpt_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , actuator_id_first_6(0)
    , actuator_id_last_6(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint64_t _actuator_id_first_6_type;
  _actuator_id_first_6_type actuator_id_first_6;

   typedef uint64_t _actuator_id_last_6_type;
  _actuator_id_last_6_type actuator_id_last_6;





  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> const> ConstPtr;

}; // struct UniqueDeviceIdRpt_

typedef ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<std::allocator<void> > UniqueDeviceIdRpt;

typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt > UniqueDeviceIdRptPtr;
typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt const> UniqueDeviceIdRptConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.actuator_id_first_6 == rhs.actuator_id_first_6 &&
    lhs.actuator_id_last_6 == rhs.actuator_id_last_6;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kartech_linear_actuator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ea8eb311cb86c91d9fa6aff8968d0ee0";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xea8eb311cb86c91dULL;
  static const uint64_t static_value2 = 0x9fa6aff8968d0ee0ULL;
};

template<class ContainerAllocator>
struct DataType< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kartech_linear_actuator_msgs/UniqueDeviceIdRpt";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"uint64 actuator_id_first_6    # The first six bytes of the unique ID of this actuator.\n"
"uint64 actuator_id_last_6     # The last six bytes of the unique ID of this actuator.\n"
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

  static const char* value(const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.actuator_id_first_6);
      stream.next(m.actuator_id_last_6);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UniqueDeviceIdRpt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kartech_linear_actuator_msgs::UniqueDeviceIdRpt_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "actuator_id_first_6: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.actuator_id_first_6);
    s << indent << "actuator_id_last_6: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.actuator_id_last_6);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_UNIQUEDEVICEIDRPT_H
