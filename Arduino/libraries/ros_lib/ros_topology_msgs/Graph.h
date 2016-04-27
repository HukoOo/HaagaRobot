#ifndef _ROS_ros_topology_msgs_Graph_h
#define _ROS_ros_topology_msgs_Graph_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros_topology_msgs/Node.h"
#include "ros_topology_msgs/Topic.h"

namespace ros_topology_msgs
{

  class Graph : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* master;
      uint8_t nodes_length;
      ros_topology_msgs::Node st_nodes;
      ros_topology_msgs::Node * nodes;
      uint8_t topics_length;
      ros_topology_msgs::Topic st_topics;
      ros_topology_msgs::Topic * topics;

    Graph():
      header(),
      master(""),
      nodes_length(0), nodes(NULL),
      topics_length(0), topics(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_master = strlen(this->master);
      memcpy(outbuffer + offset, &length_master, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->master, length_master);
      offset += length_master;
      *(outbuffer + offset++) = nodes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = topics_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < topics_length; i++){
      offset += this->topics[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_master;
      memcpy(&length_master, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_master; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_master-1]=0;
      this->master = (char *)(inbuffer + offset-1);
      offset += length_master;
      uint8_t nodes_lengthT = *(inbuffer + offset++);
      if(nodes_lengthT > nodes_length)
        this->nodes = (ros_topology_msgs::Node*)realloc(this->nodes, nodes_lengthT * sizeof(ros_topology_msgs::Node));
      offset += 3;
      nodes_length = nodes_lengthT;
      for( uint8_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(ros_topology_msgs::Node));
      }
      uint8_t topics_lengthT = *(inbuffer + offset++);
      if(topics_lengthT > topics_length)
        this->topics = (ros_topology_msgs::Topic*)realloc(this->topics, topics_lengthT * sizeof(ros_topology_msgs::Topic));
      offset += 3;
      topics_length = topics_lengthT;
      for( uint8_t i = 0; i < topics_length; i++){
      offset += this->st_topics.deserialize(inbuffer + offset);
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(ros_topology_msgs::Topic));
      }
     return offset;
    }

    const char * getType(){ return "ros_topology_msgs/Graph"; };
    const char * getMD5(){ return "01c216943d54a2b673ba09f40ec3fe51"; };

  };

}
#endif