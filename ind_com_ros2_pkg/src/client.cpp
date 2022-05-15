#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

#define PORTADDR 9009

///////////////////////////////////////////////////

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TCPClient : public rclcpp::Node
{
  public:
    TCPClient(char* c_ip)
    : Node("minimal_publisher"), count_(0)
    {
         //initializate socket
        ClientSock = socket(AF_INET, SOCK_STREAM, 0);
        if (ClientSock < 0)
        {
            printf("Imposible to open socket\n");
            exit(EXIT_FAILURE);
        }
        else
            printf("Socket available\n");

        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(PORTADDR);
        serverAddr.sin_addr.s_addr = inet_addr(c_ip);

        if (connect(ClientSock, (struct sockaddr*)& serverAddr, sizeof(serverAddr)) < 0)
        {
            printf("Imposible to connect.\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("Connected to the Server. \n");
        }

    count = 0;

        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        50ms, std::bind(&TCPClient::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        char buf[4096];
        std::string userInput;

        std::cout << "> ";
        userInput = std::to_string(count++);
        

        int sendRes = send(ClientSock, userInput.c_str(), userInput.size() + 1, 0);
        if(sendRes > 0 )
        {
            
        }

        memset(buf, 0, 4096);
        int bytesRes = recv(ClientSock, buf, 4096, 0);

        std::cout << "Server> " << std::string(buf, bytesRes) << "\r\n";
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;


    int ClientSock;
    struct sockaddr_in serverAddr;
    int count;
};

int main(int argc, char * argv[])
{
    if(argc > 2)
    {
        std::cerr << "Usage: ros2 run ind_com_ros2_pkg client <ip_address>" << std::endl;
        return 0;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TCPClient>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
