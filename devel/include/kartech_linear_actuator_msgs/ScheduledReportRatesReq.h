// Generated by gencpp from file kartech_linear_actuator_msgs/ScheduledReportRatesReq.msg
// DO NOT EDIT!


#ifndef KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_SCHEDULEDREPORTRATESREQ_H
#define KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_SCHEDULEDREPORTRATESREQ_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <kartech_linear_actuator_msgs/ReportIndex.h>
#include <kartech_linear_actuator_msgs/ReportIndex.h>

namespace kartech_linear_actuator_msgs
{
template <class ContainerAllocator>
struct ScheduledReportRatesReq_
{
  typedef ScheduledReportRatesReq_<ContainerAllocator> Type;

  ScheduledReportRatesReq_()
    : header()
    , confirm(false)
    , index_1()
    , index_1_report_time(0)
    , index_2()
    , index_2_report_time(0)  {
    }
  ScheduledReportRatesReq_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , confirm(false)
    , index_1(_alloc)
    , index_1_report_time(0)
    , index_2(_alloc)
    , index_2_report_time(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _confirm_type;
  _confirm_type confirm;

   typedef  ::kartech_linear_actuator_msgs::ReportIndex_<ContainerAllocator>  _index_1_type;
  _index_1_type index_1;

   typedef uint16_t _index_1_report_time_type;
  _index_1_report_time_type index_1_report_time;

   typedef  ::kartech_linear_actuator_msgs::ReportIndex_<ContainerAllocator>  _index_2_type;
  _index_2_type index_2;

   typedef uint16_t _index_2_report_time_type;
  _index_2_report_time_type index_2_report_time;





  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> const> ConstPtr;

}; // struct ScheduledReportRatesReq_

typedef ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<std::allocator<void> > ScheduledReportRatesReq;

typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq > ScheduledReportRatesReqPtr;
typedef boost::shared_ptr< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq const> ScheduledReportRatesReqConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.confirm == rhs.confirm &&
    lhs.index_1 == rhs.index_1 &&
    lhs.index_1_report_time == rhs.index_1_report_time &&
    lhs.index_2 == rhs.index_2 &&
    lhs.index_2_report_time == rhs.index_2_report_time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator1> & lhs, const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kartech_linear_actuator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
{
  static const char* value()
  {
    return "26225aeadc02f4f458a0546ea8c99d87";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x26225aeadc02f4f4ULL;
  static const uint64_t static_value2 = 0x58a0546ea8c99d87ULL;
};

template<class ContainerAllocator>
struct DataType< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kartech_linear_actuator_msgs/ScheduledReportRatesReq";
  }

  static const char* value(const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"bool confirm\n"
"kartech_linear_actuator_msgs/ReportIndex index_1\n"
"uint16 index_1_report_time                       # How often to publish the requested report in ms.\n"
"kartech_linear_actuator_msgs/ReportIndex index_2 # If this is set to REPORT_NONE_INDEX then only the first index will be reported.\n"
"uint16 index_2_report_time                       # Ignored if index_2 is set to REPORT_NONE_INDEX.\n"
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
"\n"
"================================================================================\n"
"MSG: kartech_linear_actuator_msgs/ReportIndex\n"
"uint8 POSITION_REPORT_INDEX = 128\n"
"uint8 MOTOR_CURRENT_REPORT_INDEX = 129\n"
"uint8 ENHANCED_POSITION_REPORT_INDEX = 152\n"
"uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167\n"
"uint8 SOFTWARE_REVISION_REPORT_INDEX = 229\n"
"uint8 ZEROING_MESSAGE_REPORT_INDEX = 238\n"
"\n"
"uint8 report_index\n"
;
  }

  static const char* value(const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.confirm);
      stream.next(m.index_1);
      stream.next(m.index_1_report_time);
      stream.next(m.index_2);
      stream.next(m.index_2_report_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScheduledReportRatesReq_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kartech_linear_actuator_msgs::ScheduledReportRatesReq_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "confirm: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.confirm);
    s << indent << "index_1: ";
    s << std::endl;
    Printer< ::kartech_linear_actuator_msgs::ReportIndex_<ContainerAllocator> >::stream(s, indent + "  ", v.index_1);
    s << indent << "index_1_report_time: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.index_1_report_time);
    s << indent << "index_2: ";
    s << std::endl;
    Printer< ::kartech_linear_actuator_msgs::ReportIndex_<ContainerAllocator> >::stream(s, indent + "  ", v.index_2);
    s << indent << "index_2_report_time: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.index_2_report_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KARTECH_LINEAR_ACTUATOR_MSGS_MESSAGE_SCHEDULEDREPORTRATESREQ_H
