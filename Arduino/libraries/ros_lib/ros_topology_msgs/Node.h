#ifndef _ROS_ros_topology_msgs_Node_h
#define _ROS_ros_topology_msgs_Node_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros_topology_msgs/Connection.h"
#include "ros_topology_msgs/Service.h"

namespace ros_topology_msgs
{

  class Node : public ros::Msg
  {
    public:
      const char* name;
      const char* uri;
      uint8_t publishes_length;
      char* st_publishes;
      char* * publishes;
      uint8_t subscribes_length;
      char* st_subscribes;
      char* * subscribes;
      uint8_t connections_length;
      ros_topology_msgs::Connection st_connections;
      ros_topology_msgs::Connection * connections;
      uint8_t provides_length;
      ros_topology_msgs::Service st_provides;
      ros_topology_msgs::Service * provides;

    Node():
      name(""),
      uri(""),
      publishes_length(0), publishes(NULL),
      subscribes_length(0), subscribes(NULL),
      connections_length(0), connections(NULL),
      provides_length(0), provides(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_uri = strlen(this->uri);
      memcpy(outbuffer + offset, &length_uri, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->uri, length_uri);
      offset += length_uri;
      *(outbuffer + offset++) = publishes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < publishes_length; i++){
      uint32_t length_publishesi = strlen(this->publishes[i]);
      memcpy(outbuffer + offset, &length_publishesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->publishes[i], length_publishesi);
      offset += length_publishesi;
      }
      *(outbuffer + offset++) = subscribes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < subscribes_length; i++){
      uint32_t length_subscribesi = strlen(this->subscribes[i]);
      memcpy(outbuffer + offset, &length_subscribesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->subscribes[i], length_subscribesi);
      offset += length_subscribesi;
      }
      *(outbuffer + offset++) = connections_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < connections_length; i++){
      offset += this->connections[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = provides_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < provides_length; i++){
      offset += this->provides[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_uri;
      memcpy(&length_uri, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uri-1]=0;
      this->uri = (char *)(inbuffer + offset-1);
      offset += length_uri;
      uint8_t publishes_lengthT = *(inbuffer + offset++);
      if(publishes_lengthT > publishes_length)
        this->publishes = (char**)realloc(this->publishes, publishes_lengthT * sizeof(char*));
      offset += 3;
      publishes_length = publishes_lengthT;
      for( uint8_t i = 0; i < publishes_length; i++){
      uint32_t length_st_publishes;
      memcpy(&length_st_publishes, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_publishes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_publishes-1]=0;
      this->st_publishes = (char *)(inbuffer + offset-1);
      offset += length_st_publishes;
        memcpy( &(this->publishes[i]), &(this->st_publishes), sizeof(char*));
      }
      uint8_t subscribes_lengthT = *(inbuffer + offset++);
      if(subscribes_lengthT > subscribes_length)
        this->subscribes = (char**)realloc(this->subscribes, subscribes_lengthT * sizeof(char*));
      offset += 3;
      subscribes_length = subscribes_lengthT;
      for( uint8_t i = 0; i < subscribes_length; i++){
      uint32_t length_st_subscribes;
      memcpy(&length_st_subscribes, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_subscribes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_subscribes-1]=0;
      this->st_subscribes = (char *)(inbuffer + offset-1);
      offset += length_st_subscribes;
        memcpy( &(this->subscribes[i]), &(this->st_subscribes), sizeof(char*));
      }
      uint8_t connections_lengthT = *(inbuffer + offset++);
      if(connections_lengthT > connections_length)
        this->connections = (ros_topology_msgs::Connection*)realloc(this->connections, connections_lengthT * sizeof(ros_topology_msgs::Connection));
      offset += 3;
      connections_length = connections_lengthT;
      for( uint8_t i = 0; i < connections_length; i++){
      offset += this->st_connections.deserialize(inbuffer + offset);
        memcpy( &(this->connections[i]), &(this->st_connections), sizeof(ros_topology_msgs::Connection));
      }
      uint8_t provides_lengthT = *(inbuffer + offset++);
      if(provides_lengthT > provides_length)
        this->provides = (ros_topology_msgs::Service*)realloc(this->provides, provides_lengthT * sizeof(ros_topology_msgs::Service));
      offset += 3;
      provides_length = provides_lengthT;
      for( uint8_t i = 0; i < provides_length; i++){
      offset += this->st_provides.deserialize(inbuffer + offset);
        memcpy( &(this->provides[i]), &(this->st_provides), sizeof(ros_topology_msgs::Service));
      }
     return offset;
    }

    const char * getType(){ return "ros_topology_msgs/Node"; };
    const char * getMD5(){ return "e793d9f25902b39f1451f272b3499991"; };

  };

}
#endif