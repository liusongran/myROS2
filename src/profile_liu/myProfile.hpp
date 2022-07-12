#ifndef SRLIU_MY_PROFILE_HPP
#define SRLIU_MY_PROFILE_HPP

#include "stdint.h"
#include "stdio.h"
#include "time.h"

#define DO_TRACE    1

#if (DO_TRACE==1)
    #define DT_PUBLISHER_HPP        0       //|level0|FILE: publiser.hpp
    #define DT_IPM_HPP              0       //|level1|FILE: intra_process_manager.hpp
    #define DT_SUB_IPM_BUFFER_HPP   0       //|level2|FILE: subscription_intra_process_buffer.hpp

    #define DT_SIG_EXECUTOR         1       //|level0|FILE: single_threaded_executor.cpp & executor.cpp

#else
    #define DT_PUBLISHER_HPP        0       //|level0|FILE: publiser.hpp
    #define DT_IPM_HPP              0       //|level1|FILE: intra_process_manager.hpp
    #define DT_SUB_IPM_BUFFER_HPP   0       //|level2|FILE: subscription_intra_process_buffer.hpp

    #define DT_SIG_EXECUTOR         0       //|level0|FILE: single_threaded_executor.cpp & executor.cpp

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