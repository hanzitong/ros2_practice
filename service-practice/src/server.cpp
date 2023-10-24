
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>


// callback function for treating service 
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response){

    response->sum = request->a + request->b;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "incomming request a:%ld b:%ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %ld", (long int)response->sum);

    // below sentence cause error
    // RCLCPP_INFO(rclcpp::get_logger(), "incomming request a:%ld b:%ld", request->a, request->b);
    // RCLCPP_INFO(rclcpp::get_logger(), "sending back response: %ld", (long int)response->sum);

}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    /* node declaration */
    std::shared_ptr<rclcpp::Node> node_ptr = rclcpp::Node::make_shared("add_two_ints_server");
    /* node declaration without syntax sugar (in this case, std::forward() is unnecessary) */
    // std::shared_ptr<rclcpp::Node> node_ptr = std::make_shared<rclcpp::Node>("add_two_ints_server");
        // This sentence makes a smart-pointer named "node_ptr" pointing an instance of rclcpp::Node.
        // rclcpp is a namespace where Node class is defined.
        // As a result, rclcpp::Node::make_shared() is a static class function of rclcpp::Node.
        // However we cannot find this definition inside the rclcpp::Node because this is defined by ros2 macro.
            // see rclcpp/rclcpp/include/macros.hpp
                /* 
                #define RCLCPP_SMART_PTR_ALIASES_ONLY(...) \
                    __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__) \
                    __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__) \
                    __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
                    __RCLCPP_MAKE_SHARED_DEFINITION(__VA_ARGS__)

                #define __RCLCPP_MAKE_SHARED_DEFINITION(...) \
                    template<typename ... Args> \
                    static std::shared_ptr<__VA_ARGS__> \
                    make_shared(Args && ... args) \
                    { \
                        return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
                    }
                */
            // __RCLCPP_MAKE_SHARED_DEFINITION() is defining rclcpp::Node::make_shared().
            // This macro adds std::make_share() to the target class as static function when compiling.
            // Above macro is nested in another macro called RCLCPP_SMART_PTR_ALIASES_ONLY macro.
            // RCLCPP_SMART_PTR_ALIASES_ONLY is used many places inside the rclcpp::Node definition.
            // These macros are a kind of "syntax sugar", a kind of wrapper, which makes us easy to read codes.

    /* service declaration via rclcpp::Node::create_service<>()*/
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_ptr = 
        node_ptr -> create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
    // the part of "Service<example_interfaces::srv::AddTwoInts>::SharedPtr" is template class.
    // the part of "example_interfaces::srv::AddTwoInts" is struct which is defined as ros2 srv.
    // in concise expression, Service<T>::SharedPtr is a ros-specific smart-pointer.
    // rclcpp::Service<ServiceT> is a class template.
    // rclcpp::Service<T>()::SharedPtr is a kind of alias grammar of C++11 (keyword: "typedef" "using").
    // so rclcpp::Service<T>::SharedPtr stands for std::shared_ptr<rclcpp::Service<T>>.


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ready to add two ints.");


    rclcpp::spin(node_ptr);     // void rclcpp::spin(rclcpp::Node::SharedPtr node_ptr)
    rclcpp::shutdown();

    return 0;
}


    // rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> pub_options;
    // // ここでpub_optionsをカスタマイズすることができます

    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10, pub_options);

    // auto timer_callback =


