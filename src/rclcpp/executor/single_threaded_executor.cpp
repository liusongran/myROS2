// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/scope_exit.hpp"

#include "/home/srliu/myProject/mySpace/src/profile_liu/myProfile.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );

#if (DT_RCLCPP_SIG_EXECUTOR==1) //MARK:-start
  std::thread::id thread_id = std::this_thread::get_id();
  uint64_t ulProber = 0;
#endif
  while (rclcpp::ok(this->context_) && spinning.load()) {
TRACE_LIU_SIG("\n|THREAD: %d|rclcpp|single_threaded_executor.cpp|[spin]->[get_next_executable]|PT-1=> start-timestamp: %ld\n", (*(uint32_t*)&thread_id), ulProber);
    rclcpp::AnyExecutable any_executable;
    bool temp;
    temp = get_next_executable(any_executable);
TRACE_LIU_SIG("\n|THREAD: %d|rclcpp|single_threaded_executor.cpp|[spin]->[get_next_executable]|PT-1=> end-timestamp: %ld\n", (*(uint32_t*)&thread_id), ulProber);
    
    if (temp) {
TRACE_LIU_SIG("\n|THREAD: %d|rclcpp|single_threaded_executor.cpp|[spin]->[execute_any_executable]|PT-2=> start-timestamp: %ld\n", (*(uint32_t*)&thread_id), ulProber);
      execute_any_executable(any_executable);
TRACE_LIU_SIG("\n|THREAD: %d|rclcpp|single_threaded_executor.cpp|[spin]->[execute_any_executable]|PT-2=> end-timestamp: %ld\n", (*(uint32_t*)&thread_id), ulProber);
    }

  }
}
