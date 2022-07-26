// Generated by gencpp from file mycobot_communication/MycobotGripperStatus.msg
// DO NOT EDIT!


#ifndef MYCOBOT_COMMUNICATION_MESSAGE_MYCOBOTGRIPPERSTATUS_H
#define MYCOBOT_COMMUNICATION_MESSAGE_MYCOBOTGRIPPERSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mycobot_communication
{
template <class ContainerAllocator>
struct MycobotGripperStatus_
{
  typedef MycobotGripperStatus_<ContainerAllocator> Type;

  MycobotGripperStatus_()
    : Status(false)  {
    }
  MycobotGripperStatus_(const ContainerAllocator& _alloc)
    : Status(false)  {
  (void)_alloc;
    }



   typedef uint8_t _Status_type;
  _Status_type Status;





  typedef boost::shared_ptr< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> const> ConstPtr;

}; // struct MycobotGripperStatus_

typedef ::mycobot_communication::MycobotGripperStatus_<std::allocator<void> > MycobotGripperStatus;

typedef boost::shared_ptr< ::mycobot_communication::MycobotGripperStatus > MycobotGripperStatusPtr;
typedef boost::shared_ptr< ::mycobot_communication::MycobotGripperStatus const> MycobotGripperStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator1> & lhs, const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator2> & rhs)
{
  return lhs.Status == rhs.Status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator1> & lhs, const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mycobot_communication

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "513e93c68ef2f26ff494445b932bb052";
  }

  static const char* value(const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x513e93c68ef2f26fULL;
  static const uint64_t static_value2 = 0xf494445b932bb052ULL;
};

template<class ContainerAllocator>
struct DataType< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mycobot_communication/MycobotGripperStatus";
  }

  static const char* value(const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool Status\n"
;
  }

  static const char* value(const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MycobotGripperStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mycobot_communication::MycobotGripperStatus_<ContainerAllocator>& v)
  {
    s << indent << "Status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MYCOBOT_COMMUNICATION_MESSAGE_MYCOBOTGRIPPERSTATUS_H
