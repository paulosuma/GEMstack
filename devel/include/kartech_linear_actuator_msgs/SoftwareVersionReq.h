// Generated by gencpp from file kartech_linear_actuator_msgs/SoftwareVersionReq.msg
// DO NOT EDIT!


#ifndef KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_SOFTWAREVERSIONREQ_H
#define KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_SOFTWAREVERSIONREQ_H


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
struct SoftwareVersionReq_
{
  typedef SoftwareVersionReq_<ContainerAllocator> Type;

  SoftwareVersionReq_()
    : header()
    , confirm(false)  {
    }
  SoftwareVersionReq_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , confirm(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _confirm_type;
  _confirm_type confirm;





  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> const> ConstPtr;

}; // struct SoftwareVersionReq_

typedef ::kartech_linear_actuator_msgs::SoftwareVersionReq_<std::allocator<void> > SoftwareVersionReq;

typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::SoftwareVersionReq > SoftwareVersionReqPtr;
typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::SoftwareVersionReq const> SoftwareVersionReqConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.confirm == rhs.confirm;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kartech_linear_actuator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d73ec12e18c7d20276159c8210d67b94";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd73ec12e18c7d202ULL;
  static const uint64_t static_value2 = 0x76159c8210d67b94ULL;
};

template<class ContainerAllocator>
struct DataType< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kartech_linear_actuator_msgs/SoftwareVersionReq";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"bool confirm\n"
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

  static const char* value(const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.confirm);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SoftwareVersionReq_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kartech_linear_actuator_msgs::SoftwareVersionReq_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "confirm: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.confirm);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_SOFTWAREVERSIONREQ_H
