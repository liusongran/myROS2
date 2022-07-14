# myROS2

### src
├── two_node_pipeline

    ├── CMakeLists.txt                  --> demos/intra_process_demo/CMakeLists.txt

    ├── two_node_pipeline.cpp           --> demos/intra_process_demo/src/two_node_pipeline/*

    ├── two_node_pipeline_inter1.cpp

    ├── two_node_pipeline_inter2.cpp

    └── two_node_pipeline_inter.cpp


### rcl


### rclcpp
├── executor

│   ├── multi_threaded_executor.cpp             --> rclcpp/rclcpp/src/rclcpp/executors/*

│   ├── single_threaded_executor.cpp

│   ├── static_executor_entities_collector.cpp

│   └── static_single_threaded_executor.cpp

├── pub

│   └── publisher.hpp                           --> rclcpp/rclcpp/include/rclcpp/publisher.hpp

├── sub

└── wait_set
