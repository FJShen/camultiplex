// Generated by gencpp from file camultiplex/TTest.msg
// DO NOT EDIT!


#ifndef CAMULTIPLEX_MESSAGE_TTEST_H
#define CAMULTIPLEX_MESSAGE_TTEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace camultiplex
{
template <class ContainerAllocator>
struct TTest_
{
  typedef TTest_<ContainerAllocator> Type;

  TTest_()
    : TString()
    , value(0)  {
    }
  TTest_(const ContainerAllocator& _alloc)
    : TString(_alloc)
    , value(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _TString_type;
  _TString_type TString;

   typedef uint32_t _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::camultiplex::TTest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::camultiplex::TTest_<ContainerAllocator> const> ConstPtr;

}; // struct TTest_

typedef ::camultiplex::TTest_<std::allocator<void> > TTest;

typedef boost::shared_ptr< ::camultiplex::TTest > TTestPtr;
typedef boost::shared_ptr< ::camultiplex::TTest const> TTestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::camultiplex::TTest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::camultiplex::TTest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace camultiplex

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'camultiplex': ['/home/nvidia/catkin_ws/src/camultiplex/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::camultiplex::TTest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camultiplex::TTest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camultiplex::TTest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camultiplex::TTest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camultiplex::TTest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camultiplex::TTest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::camultiplex::TTest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6dd10cf3b8877d89431c09fd24eede0b";
  }

  static const char* value(const ::camultiplex::TTest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6dd10cf3b8877d89ULL;
  static const uint64_t static_value2 = 0x431c09fd24eede0bULL;
};

template<class ContainerAllocator>
struct DataType< ::camultiplex::TTest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "camultiplex/TTest";
  }

  static const char* value(const ::camultiplex::TTest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::camultiplex::TTest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string TString\n\
uint32 value\n\
";
  }

  static const char* value(const ::camultiplex::TTest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::camultiplex::TTest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.TString);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TTest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::camultiplex::TTest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::camultiplex::TTest_<ContainerAllocator>& v)
  {
    s << indent << "TString: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.TString);
    s << indent << "value: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAMULTIPLEX_MESSAGE_TTEST_H
