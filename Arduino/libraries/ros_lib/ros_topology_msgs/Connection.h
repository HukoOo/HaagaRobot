#ifndef _ROS_ros_topology_msgs_Connection_h
#define _ROS_ros_topology_msgs_Connection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_topology_msgs
{

  class Connection : public ros::Msg
  {
    public:
      const char* destination;
      const char* topic;
      uint8_t direction;
      const char* transport;
      enum { IN = 1 };
      enum { OUT = 2 };
      enum { BOTH = 3 };

    Connection():
      destination(""),
      topic(""),
      direction(0),
      transport("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_destination = strlen(this->destination);
      memcpy(outbuffer + offset, &length_destination, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->destination, length_destination);
      offset += length_destination;
      uint32_t length_topic = strlen(this->topic);
      memcpy(outbuffer + offset, &length_topic, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->topic, length_topic);
      offset += length_topic;
      *(outbuffer + offset + 0) = (this->direction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction);
      uint32_t length_transport = strlen(this->transport);
      memcpy(outbuffer + offset, &length_transport, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->transport, length_transport);
      offset += length_transport;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_destination;
      memcpy(&length_destination, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_destination; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_destination-1]=0;
      this->destination = (char *)(inbuffer + offset-1);
      offset += length_destination;
      uint32_t length_topic;
      memcpy(&length_topic, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic-1]=0;
      this->topic = (char *)(inbuffer + offset-1);
      offset += length_topic;
      this->direction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->direction);
      uint32_t length_transport;
      memcpy(&length_transport, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_transport; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_transport-1]=0;
      this->transport = (char *)(inbuffer + offset-1);
      offset += length_transport;
     return offset;
    }

    const char * getType(){ return "ros_topology_msgs/Connection"; };
    const char * getMD5(){ return "bb176af5fc3e9873fcb695c8f523ec43"; };

  };

}
#endif