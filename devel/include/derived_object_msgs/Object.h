// Generated by gencpp from file derived_object_msgs/Object.msg
// DO NOT EDIT!


#ifndef DERIVED_OBJECT_MSGS_MESSAGE_OBJECT_H
#define DERIVED_OBJECT_MSGS_MESSAGE_OBJECT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Polygon.h>
#include <shape_msgs/SolidPrimitive.h>

namespace derived_object_msgs
{
template <class ContainerAllocator>
struct Object_
{
  typedef Object_<ContainerAllocator> Type;

  Object_()
    : header()
    , id(0)
    , detection_level(0)
    , object_classified(false)
    , pose()
    , twist()
    , accel()
    , polygon()
    , shape()
    , classification(0)
    , classification_certainty(0)
    , classification_age(0)  {
    }
  Object_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , detection_level(0)
    , object_classified(false)
    , pose(_alloc)
    , twist(_alloc)
    , accel(_alloc)
    , polygon(_alloc)
    , shape(_alloc)
    , classification(0)
    , classification_certainty(0)
    , classification_age(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _id_type;
  _id_type id;

   typedef uint8_t _detection_level_type;
  _detection_level_type detection_level;

   typedef uint8_t _object_classified_type;
  _object_classified_type object_classified;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::geometry_msgs::Accel_<ContainerAllocator>  _accel_type;
  _accel_type accel;

   typedef  ::geometry_msgs::Polygon_<ContainerAllocator>  _polygon_type;
  _polygon_type polygon;

   typedef  ::shape_msgs::SolidPrimitive_<ContainerAllocator>  _shape_type;
  _shape_type shape;

   typedef uint8_t _classification_type;
  _classification_type classification;

   typedef uint8_t _classification_certainty_type;
  _classification_certainty_type classification_certainty;

   typedef uint32_t _classification_age_type;
  _classification_age_type classification_age;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(OBJECT_DETECTED)
  #undef OBJECT_DETECTED
#endif
#if defined(_WIN32) && defined(OBJECT_TRACKED)
  #undef OBJECT_TRACKED
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_UNKNOWN)
  #undef CLASSIFICATION_UNKNOWN
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_UNKNOWN_SMALL)
  #undef CLASSIFICATION_UNKNOWN_SMALL
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_UNKNOWN_MEDIUM)
  #undef CLASSIFICATION_UNKNOWN_MEDIUM
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_UNKNOWN_BIG)
  #undef CLASSIFICATION_UNKNOWN_BIG
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_PEDESTRIAN)
  #undef CLASSIFICATION_PEDESTRIAN
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_BIKE)
  #undef CLASSIFICATION_BIKE
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_CAR)
  #undef CLASSIFICATION_CAR
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_TRUCK)
  #undef CLASSIFICATION_TRUCK
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_MOTORCYCLE)
  #undef CLASSIFICATION_MOTORCYCLE
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_OTHER_VEHICLE)
  #undef CLASSIFICATION_OTHER_VEHICLE
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_BARRIER)
  #undef CLASSIFICATION_BARRIER
#endif
#if defined(_WIN32) && defined(CLASSIFICATION_SIGN)
  #undef CLASSIFICATION_SIGN
#endif

  enum {
    OBJECT_DETECTED = 0u,
    OBJECT_TRACKED = 1u,
    CLASSIFICATION_UNKNOWN = 0u,
    CLASSIFICATION_UNKNOWN_SMALL = 1u,
    CLASSIFICATION_UNKNOWN_MEDIUM = 2u,
    CLASSIFICATION_UNKNOWN_BIG = 3u,
    CLASSIFICATION_PEDESTRIAN = 4u,
    CLASSIFICATION_BIKE = 5u,
    CLASSIFICATION_CAR = 6u,
    CLASSIFICATION_TRUCK = 7u,
    CLASSIFICATION_MOTORCYCLE = 8u,
    CLASSIFICATION_OTHER_VEHICLE = 9u,
    CLASSIFICATION_BARRIER = 10u,
    CLASSIFICATION_SIGN = 11u,
  };


  typedef boost::shared_ptr< ::derived_object_msgs::Object_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::derived_object_msgs::Object_<ContainerAllocator> const> ConstPtr;

}; // struct Object_

typedef ::derived_object_msgs::Object_<std::allocator<void> > Object;

typedef boost::shared_ptr< ::derived_object_msgs::Object > ObjectPtr;
typedef boost::shared_ptr< ::derived_object_msgs::Object const> ObjectConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::derived_object_msgs::Object_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::derived_object_msgs::Object_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::derived_object_msgs::Object_<ContainerAllocator1> & lhs, const ::derived_object_msgs::Object_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.detection_level == rhs.detection_level &&
    lhs.object_classified == rhs.object_classified &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist &&
    lhs.accel == rhs.accel &&
    lhs.polygon == rhs.polygon &&
    lhs.shape == rhs.shape &&
    lhs.classification == rhs.classification &&
    lhs.classification_certainty == rhs.classification_certainty &&
    lhs.classification_age == rhs.classification_age;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::derived_object_msgs::Object_<ContainerAllocator1> & lhs, const ::derived_object_msgs::Object_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace derived_object_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::derived_object_msgs::Object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::derived_object_msgs::Object_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::derived_object_msgs::Object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::derived_object_msgs::Object_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::derived_object_msgs::Object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::derived_object_msgs::Object_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::derived_object_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "89f16600c45de0e26a6a4fd168ef66e0";
  }

  static const char* value(const ::derived_object_msgs::Object_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x89f16600c45de0e2ULL;
  static const uint64_t static_value2 = 0x6a6a4fd168ef66e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::derived_object_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "derived_object_msgs/Object";
  }

  static const char* value(const ::derived_object_msgs::Object_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::derived_object_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents a detected or tracked object with reference coordinate frame and timestamp.\n"
"\n"
"std_msgs/Header header\n"
"\n"
"# The id of the object (presumably from the detecting sensor).\n"
"uint32 id\n"
"\n"
"# A Detected object is one which has been seen in at least one scan/frame of a sensor.\n"
"# A Tracked object is one which has been correlated over multiple scans/frames of a sensor.\n"
"# An object which is detected can only be assumed to have valid pose and shape properties.\n"
"# An object which is tracked should also be assumed to have valid twist and accel properties.\n"
"uint8 detection_level\n"
"uint8 OBJECT_DETECTED=0\n"
"uint8 OBJECT_TRACKED=1\n"
"\n"
"# A Classified object is one which has been identified as a certain object type.\n"
"# Classified objects should have valid classification, classification_certainty, and classification_age properties.\n"
"bool object_classified\n"
"\n"
"# The detected position and orientation of the object.\n"
"geometry_msgs/Pose pose\n"
"\n"
"# The detected linear and angular velocities of the object.\n"
"geometry_msgs/Twist twist\n"
"\n"
"# The detected linear and angular accelerations of the object.\n"
"geometry_msgs/Accel accel\n"
"\n"
"# (OPTIONAL) The polygon defining the detection points at the outer edges of the object.\n"
"geometry_msgs/Polygon polygon\n"
"\n"
"# A shape conforming to the outer bounding edges of the object.\n"
"shape_msgs/SolidPrimitive shape\n"
"\n"
"# The type of classification given to this object.\n"
"uint8 classification\n"
"uint8 CLASSIFICATION_UNKNOWN=0\n"
"uint8 CLASSIFICATION_UNKNOWN_SMALL=1\n"
"uint8 CLASSIFICATION_UNKNOWN_MEDIUM=2\n"
"uint8 CLASSIFICATION_UNKNOWN_BIG=3\n"
"uint8 CLASSIFICATION_PEDESTRIAN=4\n"
"uint8 CLASSIFICATION_BIKE=5\n"
"uint8 CLASSIFICATION_CAR=6\n"
"uint8 CLASSIFICATION_TRUCK=7\n"
"uint8 CLASSIFICATION_MOTORCYCLE=8\n"
"uint8 CLASSIFICATION_OTHER_VEHICLE=9\n"
"uint8 CLASSIFICATION_BARRIER=10\n"
"uint8 CLASSIFICATION_SIGN=11\n"
"\n"
"# The certainty of the classification from the originating sensor.\n"
"# Higher value indicates greater certainty (MAX=255).\n"
"# It is recommended that a native sensor value be scaled to 0-255 for interoperability.\n"
"uint8 classification_certainty\n"
"\n"
"# The number of scans/frames from the sensor that this object has been classified as the current classification.\n"
"uint32 classification_age\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Accel\n"
"# This expresses acceleration in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"================================================================================\n"
"MSG: shape_msgs/SolidPrimitive\n"
"# Define box, sphere, cylinder, cone \n"
"# All shapes are defined to have their bounding boxes centered around 0,0,0.\n"
"\n"
"uint8 BOX=1\n"
"uint8 SPHERE=2\n"
"uint8 CYLINDER=3\n"
"uint8 CONE=4\n"
"\n"
"# The type of the shape\n"
"uint8 type\n"
"\n"
"\n"
"# The dimensions of the shape\n"
"float64[] dimensions\n"
"\n"
"# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array\n"
"\n"
"# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding\n"
"# sides of the box.\n"
"uint8 BOX_X=0\n"
"uint8 BOX_Y=1\n"
"uint8 BOX_Z=2\n"
"\n"
"\n"
"# For the SPHERE type, only one component is used, and it gives the radius of\n"
"# the sphere.\n"
"uint8 SPHERE_RADIUS=0\n"
"\n"
"\n"
"# For the CYLINDER and CONE types, the center line is oriented along\n"
"# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component\n"
"# of dimensions gives the height of the cylinder (cone).  The\n"
"# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the\n"
"# radius of the base of the cylinder (cone).  Cone and cylinder\n"
"# primitives are defined to be circular. The tip of the cone is\n"
"# pointing up, along +Z axis.\n"
"\n"
"uint8 CYLINDER_HEIGHT=0\n"
"uint8 CYLINDER_RADIUS=1\n"
"\n"
"uint8 CONE_HEIGHT=0\n"
"uint8 CONE_RADIUS=1\n"
;
  }

  static const char* value(const ::derived_object_msgs::Object_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::derived_object_msgs::Object_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.detection_level);
      stream.next(m.object_classified);
      stream.next(m.pose);
      stream.next(m.twist);
      stream.next(m.accel);
      stream.next(m.polygon);
      stream.next(m.shape);
      stream.next(m.classification);
      stream.next(m.classification_certainty);
      stream.next(m.classification_age);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Object_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::derived_object_msgs::Object_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::derived_object_msgs::Object_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "detection_level: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.detection_level);
    s << indent << "object_classified: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.object_classified);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "accel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Accel_<ContainerAllocator> >::stream(s, indent + "  ", v.accel);
    s << indent << "polygon: ";
    s << std::endl;
    Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "  ", v.polygon);
    s << indent << "shape: ";
    s << std::endl;
    Printer< ::shape_msgs::SolidPrimitive_<ContainerAllocator> >::stream(s, indent + "  ", v.shape);
    s << indent << "classification: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.classification);
    s << indent << "classification_certainty: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.classification_certainty);
    s << indent << "classification_age: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.classification_age);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DERIVED_OBJECT_MSGS_MESSAGE_OBJECT_H
