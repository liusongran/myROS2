#ifndef SRLIU_MY_PROFILE_HPP
#define SRLIU_MY_PROFILE_HPP

#include "stdint.h"
#include "stdio.h"
#include "time.h"

#define DO_TRACE                    1
#define DO_LOG                      0

#if (DO_TRACE==1)
    #define DT_PUBLISHER_HPP        0       //|level0|FILE: publiser.hpp
    #define DT_IPM_HPP              0       //|level1|FILE: intra_process_manager.hpp
    #define DT_SUB_IPM_BUFFER_HPP   0       //|level2|FILE: subscription_intra_process_buffer.hpp

    #define DT_RCLCPP_SIG_EXECUTOR  1       //|level0|FILE: single_threaded_executor.cpp
    #define DT_RCLCPP_MUL_EXECUTOR  1       //|level0|FILE: multi_threaded_executor.cpp
    #define DT_RCLCPP_EXECUTOR      1       //|level0|FILE: executor.cpp
#else
    #define DT_PUBLISHER_HPP        0       //|level0|FILE: publiser.hpp
    #define DT_IPM_HPP              0       //|level1|FILE: intra_process_manager.hpp
    #define DT_SUB_IPM_BUFFER_HPP   0       //|level2|FILE: subscription_intra_process_buffer.hpp

    #define DT_RCLCPP_SIG_EXECUTOR  0       //|level0|FILE: single_threaded_executor.cpp
    #define DT_RCLCPP_MUL_EXECUTOR  0       //|level0|FILE: multi_threaded_executor.cpp
    #define DT_RCLCPP_EXECUTOR      0       //|level0|FILE: executor.cpp
#endif

#if (DO_LOG==1)
    #define LOG_LIU(...)            printf(__VA_ARGS__)
#else
    #define LOG_LIU(...)            {}
#endif

#if (DT_RCLCPP_SIG_EXECUTOR==1)&&(DO_TRACE==1)
    #define TRACE_LIU_SIG(...)      {ulProber = get_clocktime(); printf(__VA_ARGS__);}
#else
    #define TRACE_LIU_SIG(...)      {}
#endif

#if (DT_RCLCPP_MUL_EXECUTOR==1)&&(DO_TRACE==1)
    #define TRACE_LIU_MUL(...)      {ulProber = get_clocktime(); printf(__VA_ARGS__);}
#else
    #define TRACE_LIU_MUL(...)      {}
#endif

#if (DT_RCLCPP_EXECUTOR==1)&&(DO_TRACE==1)
    #define TRACE_LIU_EXECUTOR(...) {ulProber = get_clocktime(); printf(__VA_ARGS__);}
#else
    #define TRACE_LIU_EXECUTOR(...) {}
#endif


static inline uint64_t get_clocktime() { 
  long int ns; 
  uint64_t all; 
  time_t sec; 
  struct timespec spec; 
  clock_gettime(CLOCK_REALTIME, &spec); 
  sec = spec.tv_sec; 
  ns = spec.tv_nsec; 
  all = (uint64_t) sec * 1000000000UL + (uint64_t) ns; 
  return all;  
} 

#endif //SRLIU_MY_PROFILE_HPP