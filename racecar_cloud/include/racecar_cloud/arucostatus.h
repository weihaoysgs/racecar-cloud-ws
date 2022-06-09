// Generated by gencpp from file visionmsg/arucostatus.msg
// DO NOT EDIT!


#ifndef VISIONMSG_MESSAGE_ARUCOSTATUS_H
#define VISIONMSG_MESSAGE_ARUCOSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace visionmsg
{
template <class ContainerAllocator>
struct arucostatus_
{
  typedef arucostatus_<ContainerAllocator> Type;

  arucostatus_()
    : have_aruco(0)
    , aruco_distance(0.0)  {
    }
  arucostatus_(const ContainerAllocator& _alloc)
    : have_aruco(0)
    , aruco_distance(0.0)  {
  (void)_alloc;
    }



   typedef int16_t _have_aruco_type;
  _have_aruco_type have_aruco;

   typedef float _aruco_distance_type;
  _aruco_distance_type aruco_distance;





  typedef boost::shared_ptr< ::visionmsg::arucostatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visionmsg::arucostatus_<ContainerAllocator> const> ConstPtr;

}; // struct arucostatus_

typedef ::visionmsg::arucostatus_<std::allocator<void> > arucostatus;

typedef boost::shared_ptr< ::visionmsg::arucostatus > arucostatusPtr;
typedef boost::shared_ptr< ::visionmsg::arucostatus const> arucostatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visionmsg::arucostatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visionmsg::arucostatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::visionmsg::arucostatus_<ContainerAllocator1> & lhs, const ::visionmsg::arucostatus_<ContainerAllocator2> & rhs)
{
  return lhs.have_aruco == rhs.have_aruco &&
    lhs.aruco_distance == rhs.aruco_distance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::visionmsg::arucostatus_<ContainerAllocator1> & lhs, const ::visionmsg::arucostatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace visionmsg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::visionmsg::arucostatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visionmsg::arucostatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visionmsg::arucostatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visionmsg::arucostatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visionmsg::arucostatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visionmsg::arucostatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visionmsg::arucostatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "60817af0a15292c829bd1e8d97a8e7cc";
  }

  static const char* value(const ::visionmsg::arucostatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x60817af0a15292c8ULL;
  static const uint64_t static_value2 = 0x29bd1e8d97a8e7ccULL;
};

template<class ContainerAllocator>
struct DataType< ::visionmsg::arucostatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visionmsg/arucostatus";
  }

  static const char* value(const ::visionmsg::arucostatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visionmsg::arucostatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 have_aruco\n"
"float32 aruco_distance\n"
;
  }

  static const char* value(const ::visionmsg::arucostatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visionmsg::arucostatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.have_aruco);
      stream.next(m.aruco_distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct arucostatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visionmsg::arucostatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visionmsg::arucostatus_<ContainerAllocator>& v)
  {
    s << indent << "have_aruco: ";
    Printer<int16_t>::stream(s, indent + "  ", v.have_aruco);
    s << indent << "aruco_distance: ";
    Printer<float>::stream(s, indent + "  ", v.aruco_distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISIONMSG_MESSAGE_ARUCOSTATUS_H
