#ifndef _ROS_ros_statistics_msgs_HostStatistics_h
#define _ROS_ros_statistics_msgs_HostStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace ros_statistics_msgs
{

  class HostStatistics : public ros::Msg
  {
    public:
      const char* hostname;
      const char* ipaddress;
      ros::Time window_start;
      ros::Time window_stop;
      uint16_t samples;
      uint8_t cpu_load_mean_length;
      float st_cpu_load_mean;
      float * cpu_load_mean;
      uint8_t cpu_load_std_length;
      float st_cpu_load_std;
      float * cpu_load_std;
      uint8_t cpu_load_max_length;
      float st_cpu_load_max;
      float * cpu_load_max;
      float phymem_used_mean;
      float phymem_used_std;
      float phymem_used_max;
      float phymem_avail_mean;
      float phymem_avail_std;
      float phymem_avail_max;

    HostStatistics():
      hostname(""),
      ipaddress(""),
      window_start(),
      window_stop(),
      samples(0),
      cpu_load_mean_length(0), cpu_load_mean(NULL),
      cpu_load_std_length(0), cpu_load_std(NULL),
      cpu_load_max_length(0), cpu_load_max(NULL),
      phymem_used_mean(0),
      phymem_used_std(0),
      phymem_used_max(0),
      phymem_avail_mean(0),
      phymem_avail_std(0),
      phymem_avail_max(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_hostname = strlen(this->hostname);
      memcpy(outbuffer + offset, &length_hostname, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->hostname, length_hostname);
      offset += length_hostname;
      uint32_t length_ipaddress = strlen(this->ipaddress);
      memcpy(outbuffer + offset, &length_ipaddress, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->ipaddress, length_ipaddress);
      offset += length_ipaddress;
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
      *(outbuffer + offset++) = cpu_load_mean_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cpu_load_mean_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_load_meani;
      u_cpu_load_meani.real = this->cpu_load_mean[i];
      *(outbuffer + offset + 0) = (u_cpu_load_meani.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load_meani.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load_meani.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load_meani.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load_mean[i]);
      }
      *(outbuffer + offset++) = cpu_load_std_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cpu_load_std_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_load_stdi;
      u_cpu_load_stdi.real = this->cpu_load_std[i];
      *(outbuffer + offset + 0) = (u_cpu_load_stdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load_stdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load_stdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load_stdi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load_std[i]);
      }
      *(outbuffer + offset++) = cpu_load_max_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cpu_load_max_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_load_maxi;
      u_cpu_load_maxi.real = this->cpu_load_max[i];
      *(outbuffer + offset + 0) = (u_cpu_load_maxi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load_maxi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load_maxi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load_maxi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load_max[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->phymem_used_mean);
      offset += serializeAvrFloat64(outbuffer + offset, this->phymem_used_std);
      offset += serializeAvrFloat64(outbuffer + offset, this->phymem_used_max);
      offset += serializeAvrFloat64(outbuffer + offset, this->phymem_avail_mean);
      offset += serializeAvrFloat64(outbuffer + offset, this->phymem_avail_std);
      offset += serializeAvrFloat64(outbuffer + offset, this->phymem_avail_max);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_hostname;
      memcpy(&length_hostname, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hostname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hostname-1]=0;
      this->hostname = (char *)(inbuffer + offset-1);
      offset += length_hostname;
      uint32_t length_ipaddress;
      memcpy(&length_ipaddress, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ipaddress; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ipaddress-1]=0;
      this->ipaddress = (char *)(inbuffer + offset-1);
      offset += length_ipaddress;
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
      uint8_t cpu_load_mean_lengthT = *(inbuffer + offset++);
      if(cpu_load_mean_lengthT > cpu_load_mean_length)
        this->cpu_load_mean = (float*)realloc(this->cpu_load_mean, cpu_load_mean_lengthT * sizeof(float));
      offset += 3;
      cpu_load_mean_length = cpu_load_mean_lengthT;
      for( uint8_t i = 0; i < cpu_load_mean_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_load_mean;
      u_st_cpu_load_mean.base = 0;
      u_st_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_load_mean.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_load_mean = u_st_cpu_load_mean.real;
      offset += sizeof(this->st_cpu_load_mean);
        memcpy( &(this->cpu_load_mean[i]), &(this->st_cpu_load_mean), sizeof(float));
      }
      uint8_t cpu_load_std_lengthT = *(inbuffer + offset++);
      if(cpu_load_std_lengthT > cpu_load_std_length)
        this->cpu_load_std = (float*)realloc(this->cpu_load_std, cpu_load_std_lengthT * sizeof(float));
      offset += 3;
      cpu_load_std_length = cpu_load_std_lengthT;
      for( uint8_t i = 0; i < cpu_load_std_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_load_std;
      u_st_cpu_load_std.base = 0;
      u_st_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_load_std.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_load_std = u_st_cpu_load_std.real;
      offset += sizeof(this->st_cpu_load_std);
        memcpy( &(this->cpu_load_std[i]), &(this->st_cpu_load_std), sizeof(float));
      }
      uint8_t cpu_load_max_lengthT = *(inbuffer + offset++);
      if(cpu_load_max_lengthT > cpu_load_max_length)
        this->cpu_load_max = (float*)realloc(this->cpu_load_max, cpu_load_max_lengthT * sizeof(float));
      offset += 3;
      cpu_load_max_length = cpu_load_max_lengthT;
      for( uint8_t i = 0; i < cpu_load_max_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_load_max;
      u_st_cpu_load_max.base = 0;
      u_st_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_load_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_load_max = u_st_cpu_load_max.real;
      offset += sizeof(this->st_cpu_load_max);
        memcpy( &(this->cpu_load_max[i]), &(this->st_cpu_load_max), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phymem_used_mean));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phymem_used_std));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phymem_used_max));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phymem_avail_mean));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phymem_avail_std));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->phymem_avail_max));
     return offset;
    }

    const char * getType(){ return "ros_statistics_msgs/HostStatistics"; };
    const char * getMD5(){ return "db625f59b6c5c3e06353703e3237bba0"; };

  };

}
#endif