# ros2_practice
deciphering and practicing rclcpp and so on..


# 5 staps for writing node
1. rclcpp::init()
2. make node smart-pointer with rclcpp::Node::make_shared()
3. create Topic/Service/Action instance with functions inside the node
4. rclcpp::spin()
5. rclcpp::shutdown()


# Service
## server
    - callback function
    - make Service instance via node
## client
    - make Client instance via node
    - send request by calling client_ptr->wait_for_service()
    - wait server up(stop thread) by using rclcpp::ok() and while-loop 
    - wait result from server by using rclcpp::spin_until_future_complete() and enum





