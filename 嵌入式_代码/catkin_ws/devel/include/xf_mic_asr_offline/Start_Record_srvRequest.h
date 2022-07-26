// Generated by gencpp from file xf_mic_asr_offline/Start_Record_srvRequest.msg
// DO NOT EDIT!


#ifndef XF_MIC_ASR_OFFLINE_MESSAGE_START_RECORD_SRVREQUEST_H
#define XF_MIC_ASR_OFFLINE_MESSAGE_START_RECORD_SRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xf_mic_asr_offline
{
template <class ContainerAllocator>
struct Start_Record_srvRequest_
{
  typedef Start_Record_srvRequest_<ContainerAllocator> Type;

  Start_Record_srvRequest_()
    : whether_start(0)  {
    }
  Start_Record_srvRequest_(const ContainerAllocator& _alloc)
    : whether_start(0)  {
  (void)_alloc;
    }



   typedef int8_t _whether_start_type;
  _whether_start_type whether_start;





  typedef boost::shared_ptr< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct Start_Record_srvRequest_

typedef ::xf_mic_asr_offline::Start_Record_srvRequest_<std::allocator<void> > Start_Record_srvRequest;

typedef boost::shared_ptr< ::xf_mic_asr_offline::Start_Record_srvRequest > Start_Record_srvRequestPtr;
typedef boost::shared_ptr< ::xf_mic_asr_offline::Start_Record_srvRequest const> Start_Record_srvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator1> & lhs, const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator2> & rhs)
{
  return lhs.whether_start == rhs.whether_start;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator1> & lhs, const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xf_mic_asr_offline

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "083775b02a45b03a74932fe630ed4019";
  }

  static const char* value(const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x083775b02a45b03aULL;
  static const uint64_t static_value2 = 0x74932fe630ed4019ULL;
};

template<class ContainerAllocator>
struct DataType< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xf_mic_asr_offline/Start_Record_srvRequest";
  }

  static const char* value(const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 whether_start\n"
;
  }

  static const char* value(const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.whether_start);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Start_Record_srvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xf_mic_asr_offline::Start_Record_srvRequest_<ContainerAllocator>& v)
  {
    s << indent << "whether_start: ";
    Printer<int8_t>::stream(s, indent + "  ", v.whether_start);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XF_MIC_ASR_OFFLINE_MESSAGE_START_RECORD_SRVREQUEST_H
