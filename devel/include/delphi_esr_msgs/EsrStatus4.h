// Generated by gencpp from file delphi_esr_msgs/EsrStatus4.msg
// DO NOT EDIT!


#ifndef DELPHI_ESR_MSGS_MESSAGE_ESRSTATUS4_H
#define DELPHI_ESR_MSGS_MESSAGE_ESRSTATUS4_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace delphi_esr_msgs
{
template <class ContainerAllocator>
struct EsrStatus4_
{
  typedef EsrStatus4_<ContainerAllocator> Type;

  EsrStatus4_()
    : header()
    , canmsg()
    , truck_target_det(false)
    , lr_only_grating_lobe_det(false)
    , sidelobe_blockage(false)
    , partial_blockage(false)
    , mr_lr_mode(0)
    , rolling_count_3(0)
    , path_id_acc(0)
    , path_id_cmbb_move(0)
    , path_id_cmbb_stat(0)
    , path_id_fcw_move(0)
    , path_id_fcw_stat(0)
    , auto_align_angle(0.0)
    , path_id_acc_stat(0)  {
    }
  EsrStatus4_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , canmsg(_alloc)
    , truck_target_det(false)
    , lr_only_grating_lobe_det(false)
    , sidelobe_blockage(false)
    , partial_blockage(false)
    , mr_lr_mode(0)
    , rolling_count_3(0)
    , path_id_acc(0)
    , path_id_cmbb_move(0)
    , path_id_cmbb_stat(0)
    , path_id_fcw_move(0)
    , path_id_fcw_stat(0)
    , auto_align_angle(0.0)
    , path_id_acc_stat(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _canmsg_type;
  _canmsg_type canmsg;

   typedef uint8_t _truck_target_det_type;
  _truck_target_det_type truck_target_det;

   typedef uint8_t _lr_only_grating_lobe_det_type;
  _lr_only_grating_lobe_det_type lr_only_grating_lobe_det;

   typedef uint8_t _sidelobe_blockage_type;
  _sidelobe_blockage_type sidelobe_blockage;

   typedef uint8_t _partial_blockage_type;
  _partial_blockage_type partial_blockage;

   typedef uint8_t _mr_lr_mode_type;
  _mr_lr_mode_type mr_lr_mode;

   typedef uint8_t _rolling_count_3_type;
  _rolling_count_3_type rolling_count_3;

   typedef uint8_t _path_id_acc_type;
  _path_id_acc_type path_id_acc;

   typedef uint8_t _path_id_cmbb_move_type;
  _path_id_cmbb_move_type path_id_cmbb_move;

   typedef uint8_t _path_id_cmbb_stat_type;
  _path_id_cmbb_stat_type path_id_cmbb_stat;

   typedef uint8_t _path_id_fcw_move_type;
  _path_id_fcw_move_type path_id_fcw_move;

   typedef uint8_t _path_id_fcw_stat_type;
  _path_id_fcw_stat_type path_id_fcw_stat;

   typedef float _auto_align_angle_type;
  _auto_align_angle_type auto_align_angle;

   typedef uint8_t _path_id_acc_stat_type;
  _path_id_acc_stat_type path_id_acc_stat;





  typedef boost::shared_ptr< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> const> ConstPtr;

}; // struct EsrStatus4_

typedef ::delphi_esr_msgs::EsrStatus4_<std::allocator<void> > EsrStatus4;

typedef boost::shared_ptr< ::delphi_esr_msgs::EsrStatus4 > EsrStatus4Ptr;
typedef boost::shared_ptr< ::delphi_esr_msgs::EsrStatus4 const> EsrStatus4ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator1> & lhs, const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.canmsg == rhs.canmsg &&
    lhs.truck_target_det == rhs.truck_target_det &&
    lhs.lr_only_grating_lobe_det == rhs.lr_only_grating_lobe_det &&
    lhs.sidelobe_blockage == rhs.sidelobe_blockage &&
    lhs.partial_blockage == rhs.partial_blockage &&
    lhs.mr_lr_mode == rhs.mr_lr_mode &&
    lhs.rolling_count_3 == rhs.rolling_count_3 &&
    lhs.path_id_acc == rhs.path_id_acc &&
    lhs.path_id_cmbb_move == rhs.path_id_cmbb_move &&
    lhs.path_id_cmbb_stat == rhs.path_id_cmbb_stat &&
    lhs.path_id_fcw_move == rhs.path_id_fcw_move &&
    lhs.path_id_fcw_stat == rhs.path_id_fcw_stat &&
    lhs.auto_align_angle == rhs.auto_align_angle &&
    lhs.path_id_acc_stat == rhs.path_id_acc_stat;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator1> & lhs, const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace delphi_esr_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6d073b78c0d621fce59ffa9fb7c576de";
  }

  static const char* value(const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6d073b78c0d621fcULL;
  static const uint64_t static_value2 = 0xe59ffa9fb7c576deULL;
};

template<class ContainerAllocator>
struct DataType< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
{
  static const char* value()
  {
    return "delphi_esr_msgs/EsrStatus4";
  }

  static const char* value(const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"# ESR Status4\n"
"string      canmsg\n"
"\n"
"bool        truck_target_det\n"
"bool        lr_only_grating_lobe_det\n"
"bool        sidelobe_blockage\n"
"bool        partial_blockage\n"
"uint8       mr_lr_mode\n"
"uint8       rolling_count_3\n"
"uint8       path_id_acc\n"
"uint8       path_id_cmbb_move\n"
"uint8       path_id_cmbb_stat\n"
"uint8       path_id_fcw_move\n"
"uint8       path_id_fcw_stat\n"
"float32     auto_align_angle\n"
"uint8       path_id_acc_stat\n"
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

  static const char* value(const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.canmsg);
      stream.next(m.truck_target_det);
      stream.next(m.lr_only_grating_lobe_det);
      stream.next(m.sidelobe_blockage);
      stream.next(m.partial_blockage);
      stream.next(m.mr_lr_mode);
      stream.next(m.rolling_count_3);
      stream.next(m.path_id_acc);
      stream.next(m.path_id_cmbb_move);
      stream.next(m.path_id_cmbb_stat);
      stream.next(m.path_id_fcw_move);
      stream.next(m.path_id_fcw_stat);
      stream.next(m.auto_align_angle);
      stream.next(m.path_id_acc_stat);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EsrStatus4_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::delphi_esr_msgs::EsrStatus4_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "canmsg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.canmsg);
    s << indent << "truck_target_det: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.truck_target_det);
    s << indent << "lr_only_grating_lobe_det: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.lr_only_grating_lobe_det);
    s << indent << "sidelobe_blockage: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sidelobe_blockage);
    s << indent << "partial_blockage: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.partial_blockage);
    s << indent << "mr_lr_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mr_lr_mode);
    s << indent << "rolling_count_3: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rolling_count_3);
    s << indent << "path_id_acc: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_id_acc);
    s << indent << "path_id_cmbb_move: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_id_cmbb_move);
    s << indent << "path_id_cmbb_stat: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_id_cmbb_stat);
    s << indent << "path_id_fcw_move: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_id_fcw_move);
    s << indent << "path_id_fcw_stat: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_id_fcw_stat);
    s << indent << "auto_align_angle: ";
    Printer<float>::stream(s, indent + "  ", v.auto_align_angle);
    s << indent << "path_id_acc_stat: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_id_acc_stat);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DELPHI_ESR_MSGS_MESSAGE_ESRSTATUS4_H
