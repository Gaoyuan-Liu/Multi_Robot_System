// Generated by gencpp from file hector_uav_msgs/PoseFeedback.msg
// DO NOT EDIT!


#ifndef HECTOR_UAV_MSGS_MESSAGE_POSEFEEDBACK_H
#define HECTOR_UAV_MSGS_MESSAGE_POSEFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace hector_uav_msgs
{
template <class ContainerAllocator>
struct PoseFeedback_
{
  typedef PoseFeedback_<ContainerAllocator> Type;

  PoseFeedback_()
    : current_pose()  {
    }
  PoseFeedback_(const ContainerAllocator& _alloc)
    : current_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _current_pose_type;
  _current_pose_type current_pose;





  typedef boost::shared_ptr< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct PoseFeedback_

typedef ::hector_uav_msgs::PoseFeedback_<std::allocator<void> > PoseFeedback;

typedef boost::shared_ptr< ::hector_uav_msgs::PoseFeedback > PoseFeedbackPtr;
typedef boost::shared_ptr< ::hector_uav_msgs::PoseFeedback const> PoseFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hector_uav_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'hector_uav_msgs': ['/home/liu/Multi_Robot_System/src/hector_quadrotor/hector_uav_msgs/msg', '/home/liu/Multi_Robot_System/devel/share/hector_uav_msgs/msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dd7058fae6e1bf2400513fe092a44c92";
  }

  static const char* value(const ::hector_uav_msgs::PoseFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdd7058fae6e1bf24ULL;
  static const uint64_t static_value2 = 0x00513fe092a44c92ULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hector_uav_msgs/PoseFeedback";
  }

  static const char* value(const ::hector_uav_msgs::PoseFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"geometry_msgs/PoseStamped current_pose\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
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
;
  }

  static const char* value(const ::hector_uav_msgs::PoseFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PoseFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hector_uav_msgs::PoseFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hector_uav_msgs::PoseFeedback_<ContainerAllocator>& v)
  {
    s << indent << "current_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.current_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HECTOR_UAV_MSGS_MESSAGE_POSEFEEDBACK_H
