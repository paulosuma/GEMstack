// Generated by gencpp from file mobileye_560_660_msgs/TsrVisionOnly.msg
// DO NOT EDIT!


#ifndef MOBILEYE_560_660_MSGS_MESSAGE_TSRVISIONONLY_H
#define MOBILEYE_560_660_MSGS_MESSAGE_TSRVISIONONLY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace mobileye_560_660_msgs
{
template <class ContainerAllocator>
struct TsrVisionOnly_
{
  typedef TsrVisionOnly_<ContainerAllocator> Type;

  TsrVisionOnly_()
    : header()
    , vision_only_sign_type_display1(0)
    , vision_only_supplementary_sign_type_display1(0)
    , vision_only_sign_type_display2(0)
    , vision_only_supplementary_sign_type_display2(0)
    , vision_only_sign_type_display3(0)
    , vision_only_supplementary_sign_type_display3(0)
    , vision_only_sign_type_display4(0)
    , vision_only_supplementary_sign_type_display4(0)  {
    }
  TsrVisionOnly_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vision_only_sign_type_display1(0)
    , vision_only_supplementary_sign_type_display1(0)
    , vision_only_sign_type_display2(0)
    , vision_only_supplementary_sign_type_display2(0)
    , vision_only_sign_type_display3(0)
    , vision_only_supplementary_sign_type_display3(0)
    , vision_only_sign_type_display4(0)
    , vision_only_supplementary_sign_type_display4(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _vision_only_sign_type_display1_type;
  _vision_only_sign_type_display1_type vision_only_sign_type_display1;

   typedef uint8_t _vision_only_supplementary_sign_type_display1_type;
  _vision_only_supplementary_sign_type_display1_type vision_only_supplementary_sign_type_display1;

   typedef uint8_t _vision_only_sign_type_display2_type;
  _vision_only_sign_type_display2_type vision_only_sign_type_display2;

   typedef uint8_t _vision_only_supplementary_sign_type_display2_type;
  _vision_only_supplementary_sign_type_display2_type vision_only_supplementary_sign_type_display2;

   typedef uint8_t _vision_only_sign_type_display3_type;
  _vision_only_sign_type_display3_type vision_only_sign_type_display3;

   typedef uint8_t _vision_only_supplementary_sign_type_display3_type;
  _vision_only_supplementary_sign_type_display3_type vision_only_supplementary_sign_type_display3;

   typedef uint8_t _vision_only_sign_type_display4_type;
  _vision_only_sign_type_display4_type vision_only_sign_type_display4;

   typedef uint8_t _vision_only_supplementary_sign_type_display4_type;
  _vision_only_supplementary_sign_type_display4_type vision_only_supplementary_sign_type_display4;





  typedef boost::shared_ptr< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> const> ConstPtr;

}; // struct TsrVisionOnly_

typedef ::mobileye_560_660_msgs::TsrVisionOnly_<std::allocator<void> > TsrVisionOnly;

typedef boost::shared_ptr< ::mobileye_560_660_msgs::TsrVisionOnly > TsrVisionOnlyPtr;
typedef boost::shared_ptr< ::mobileye_560_660_msgs::TsrVisionOnly const> TsrVisionOnlyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator1> & lhs, const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vision_only_sign_type_display1 == rhs.vision_only_sign_type_display1 &&
    lhs.vision_only_supplementary_sign_type_display1 == rhs.vision_only_supplementary_sign_type_display1 &&
    lhs.vision_only_sign_type_display2 == rhs.vision_only_sign_type_display2 &&
    lhs.vision_only_supplementary_sign_type_display2 == rhs.vision_only_supplementary_sign_type_display2 &&
    lhs.vision_only_sign_type_display3 == rhs.vision_only_sign_type_display3 &&
    lhs.vision_only_supplementary_sign_type_display3 == rhs.vision_only_supplementary_sign_type_display3 &&
    lhs.vision_only_sign_type_display4 == rhs.vision_only_sign_type_display4 &&
    lhs.vision_only_supplementary_sign_type_display4 == rhs.vision_only_supplementary_sign_type_display4;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator1> & lhs, const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mobileye_560_660_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
{
  static const char* value()
  {
    return "84f9582e1cda52683c53338cffe795f0";
  }

  static const char* value(const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x84f9582e1cda5268ULL;
  static const uint64_t static_value2 = 0x3c53338cffe795f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mobileye_560_660_msgs/TsrVisionOnly";
  }

  static const char* value(const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"uint8 vision_only_sign_type_display1\n"
"uint8 vision_only_supplementary_sign_type_display1\n"
"uint8 vision_only_sign_type_display2\n"
"uint8 vision_only_supplementary_sign_type_display2\n"
"uint8 vision_only_sign_type_display3\n"
"uint8 vision_only_supplementary_sign_type_display3\n"
"uint8 vision_only_sign_type_display4\n"
"uint8 vision_only_supplementary_sign_type_display4\n"
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

  static const char* value(const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vision_only_sign_type_display1);
      stream.next(m.vision_only_supplementary_sign_type_display1);
      stream.next(m.vision_only_sign_type_display2);
      stream.next(m.vision_only_supplementary_sign_type_display2);
      stream.next(m.vision_only_sign_type_display3);
      stream.next(m.vision_only_supplementary_sign_type_display3);
      stream.next(m.vision_only_sign_type_display4);
      stream.next(m.vision_only_supplementary_sign_type_display4);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TsrVisionOnly_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mobileye_560_660_msgs::TsrVisionOnly_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vision_only_sign_type_display1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_sign_type_display1);
    s << indent << "vision_only_supplementary_sign_type_display1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_supplementary_sign_type_display1);
    s << indent << "vision_only_sign_type_display2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_sign_type_display2);
    s << indent << "vision_only_supplementary_sign_type_display2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_supplementary_sign_type_display2);
    s << indent << "vision_only_sign_type_display3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_sign_type_display3);
    s << indent << "vision_only_supplementary_sign_type_display3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_supplementary_sign_type_display3);
    s << indent << "vision_only_sign_type_display4: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_sign_type_display4);
    s << indent << "vision_only_supplementary_sign_type_display4: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.vision_only_supplementary_sign_type_display4);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOBILEYE_560_660_MSGS_MESSAGE_TSRVISIONONLY_H
