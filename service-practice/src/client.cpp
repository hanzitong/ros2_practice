
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char* argv[]){
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "give me 2 argument!");
        return -1;
    }
    /*  5 staps for writing node
    1. rclcpp::init()
    2. make node smart-pointer with rclcpp::Node::make_shared()
    3. create topic/service/action with functions inside the node
    4. rclcpp::spin()
    5. rclcpp::shutdown()
    */

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node_ptr = rclcpp::Node::make_shared("add_two_ints_client");

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_ptr = 
        node_ptr -> create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    // gives while loop 1s to search service
    while (!client_ptr->wait_for_service(1s)) {
        if (!rclcpp::ok()){ // check this process is still running 
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting service.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting...");
    }

    auto result = client_ptr->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_ptr, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to call service");
    }

    rclcpp::shutdown();

    return 0;
}

