#ifndef _ROS_ros_statistics_msgs_NodeStatistics_h
#define _ROS_ros_statistics_msgs_NodeStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace ros_statistics_msgs
{

  class NodeStatistics : public ros::Msg
  {
    public:
      const char* node;
      const char* host;
      const char* uri;
      const char* pid;
      ros::Time window_start;
      ros::Time window_stop;
      uint16_t samples;
      uint16_t threads;
      float cpu_load_mean;
      float cpu_load_std;
      float cpu_load_max;
      float virt_mem_mean;
      float virt_mem_std;
      float virt_mem_max;
      float real_mem_mean;
      float real_mem_std;
      float real_mem_max;

    NodeStatistics():
      node(""),
      host(""),
      uri(""),
      pid(""),
      window_start(),
      window_stop(),
      samples(0),
      threads(0),
      cpu_load_mean(0),
      cpu_load_std(0),
      cpu_load_max(0),
      virt_mem_mean(0),
      virt_mem_std(0),
      virt_mem_max(0),
      real_mem_mean(0),
      real_mem_std(0),
      real_mem_max(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_node = strlen(this->node);
      memcpy(outbuffer + offset, &length_node, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node, length_node);
      offset += length_node;
      uint32_t length_host = strlen(this->host);
      memcpy(outbuffer + offset, &length_host, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->host, length_host);
      offset += length_host;
      uint32_t length_uri = strlen(this->uri);
      memcpy(outbuffer + offset, &length_uri, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->uri, length_uri);
      offset += length_uri;
      uint32_t length_pid = strlen(this->pid);
      memcpy(outbuffer + offset, &length_pid, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->pid, length_pid);
      offset += length_pid;
      *(outbuffer + offset + 0) = (this->window_start.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->window_start.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->window_start.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->window_start.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->window_start.sec);
      *(outbuffer + offset + 0) = (this->window_start.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->window_start.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->window_start.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->window_start.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->window_start.nsec);
      *(outbuffer + offset + 0) = (this->window_stop.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->window_stop.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->window_stop.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->window_stop.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->window_stop.sec);
      *(outbuffer + offset + 0) = (this->window_stop.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->window_stop.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->window_stop.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->window_stop.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->window_stop.nsec);
      *(outbuffer + offset + 0) = (this->samples >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->samples >> (8 * 1)) & 0xFF;
      offset += sizeof(this->samples);
      *(outbuffer + offset + 0) = (this->threads >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->threads >> (8 * 1)) & 0xFF;
      offset += sizeof(this->threads);
      union {
        float real;
        uint32_t base;
      } u_cpu_load_mean;
      u_cpu_load_mean.real = this->cpu_load_mean;
      *(outbuffer + offset + 0) = (u_cpu_load_mean.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load_mean.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load_mean.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load_mean.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load_mean);
      union {
        float real;
        uint32_t base;
      } u_cpu_load_std;
      u_cpu_load_std.real = this->cpu_load_std;
      *(outbuffer + offset + 0) = (u_cpu_load_std.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load_std.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load_std.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load_std.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load_std);
      union {
        float real;
        uint32_t base;
      } u_cpu_load_max;
      u_cpu_load_max.real = this->cpu_load_max;
      *(outbuffer + offset + 0) = (u_cpu_load_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load_max);
      offset += serializeAvrFloat64(outbuffer + offset, this->virt_mem_mean);
      union {
        float real;
        uint32_t base;
      } u_virt_mem_std;
      u_virt_mem_std.real = this->virt_mem_std;
      *(outbuffer + offset + 0) = (u_virt_mem_std.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_virt_mem_std.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_virt_mem_std.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_virt_mem_std.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->virt_mem_std);
      offset += serializeAvrFloat64(outbuffer + offset, this->virt_mem_max);
      offset += serializeAvrFloat64(outbuffer + offset, this->real_mem_mean);
      union {
        float real;
        uint32_t base;
      } u_real_mem_std;
      u_real_mem_std.real = this->real_mem_std;
      *(outbuffer + offset + 0) = (u_real_mem_std.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_real_mem_std.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_real_mem_std.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_real_mem_std.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->real_mem_std);
      offset += serializeAvrFloat64(outbuffer + offset, this->real_mem_max);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_node;
      memcpy(&length_node, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node-1]=0;
      this->node = (char *)(inbuffer + offset-1);
      offset += length_node;
      uint32_t length_host;
      memcpy(&length_host, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_host; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_host-1]=0;
      this->host = (char *)(inbuffer + offset-1);
      offset += length_host;
      uint32_t length_uri;
      memcpy(&length_uri, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uri-1]=0;
      this->uri = (char *)(inbuffer + offset-1);
      offset += length_uri;
      uint32_t length_pid;
      memcpy(&length_pid, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pid-1]=0;
      this->pid = (char *)(inbuffer + offset-1);
      offset += length_pid;
      this->window_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->window_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->window_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->window_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->window_start.sec);
      this->window_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->window_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->window_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->window_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->window_start.nsec);
      this->window_stop.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->window_stop.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->window_stop.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->window_stop.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->window_stop.sec);
      this->window_stop.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->window_stop.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->window_stop.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->window_stop.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->window_stop.nsec);
      this->samples =  ((uint16_t) (*(inbuffer + offset)));
      this->samples |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->samples);
      this->threads =  ((uint16_t) (*(inbuffer + offset)));
      this->threads |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->threads);
      union {
        float real;
        uint32_t base;
      } u_cpu_load_mean;
      u_cpu_load_mean.base = 0;
      u_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_load_mean = u_cpu_load_mean.real;
      offset += sizeof(this->cpu_load_mean);
      union {
        float real;
        uint32_t base;
      } u_cpu_load_std;
      u_cpu_load_std.base = 0;
      u_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_load_std = u_cpu_load_std.real;
      offset += sizeof(this->cpu_load_std);
      union {
        float real;
        uint32_t base;
      } u_cpu_load_max;
      u_cpu_load_max.base = 0;
      u_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_load_max = u_cpu_load_max.real;
      offset += sizeof(this->cpu_load_max);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->virt_mem_mean));
      union {
        float real;
        uint32_t base;
      } u_virt_mem_std;
      u_virt_mem_std.base = 0;
      u_virt_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_virt_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_virt_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_virt_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->virt_mem_std = u_virt_mem_std.real;
      offset += sizeof(this->virt_mem_std);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->virt_mem_max));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->real_mem_mean));
      union {
        float real;
        uint32_t base;
      } u_real_mem_std;
      u_real_mem_std.base = 0;
      u_real_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_real_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_real_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_real_mem_std.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->real_mem_std = u_real_mem_std.real;
      offset += sizeof(this->real_mem_std);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->real_mem_max));
     return offset;
    }

    const char * getType(){ return "ros_statistics_msgs/NodeStatistics"; };
    const char * getMD5(){ return "7bd20bf65465385d0a0d97df0064e759"; };

  };

}
#endif