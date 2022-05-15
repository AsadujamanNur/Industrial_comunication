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

class TCPServer : public rclcpp::Node
{
  public:
    TCPServer()
    : Node("minimal_publisher"), count_(0)
    {

        //initializate socket
        ServerSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (ServerSocket < 0)
        {
            printf("Imposible to open socket\n");
            exit(EXIT_FAILURE);
        }
        else
            printf("Socket available\n");

        SocketAddress.sin_family = AF_INET;
        SocketAddress.sin_port = htons(PORTADDR);
        SocketAddress.sin_addr.s_addr = INADDR_ANY;

        //bind socket);
        if (bind(ServerSocket, (struct sockaddr*) & SocketAddress, sizeof(SocketAddress)) < 0)
        {
            printf("Imposible to bind connection\n");
            exit(EXIT_FAILURE);
        }
        else
            printf("Connection binded to the local port\n");
    
        //listen
        if(listen(ServerSocket, 5) < 0)
        {
            printf("Not listening the local port.\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("Listening to the local port. \n");
        }
    
        //accept connection

        ClientSocket = accept(ServerSocket, (struct sockaddr *) &SocketAddress, (socklen_t*) &addrLen);
        if(ClientSocket < 0){
            printf("Connection failed.\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("Connection started\n");
        }




        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        50ms, std::bind(&TCPServer::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        
        char buf[4096];

        memset(buf, 0, 4096);
        

        int byteRes = recv(ClientSocket, buf, 4096, 0);

        std::string inMes(buf, byteRes);

        //std::cout << "Received: " << inMes << std::endl;

        RCLCPP_INFO(this->get_logger(), "Publishing message from client: '%s'", inMes.c_str());

        send(ClientSocket, "message received", 17, 0);


        auto message = std_msgs::msg::String();

        message.data = inMes;
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;


    int ServerSocket, ClientSocket;
    struct sockaddr_in SocketAddress;
    int addrLen = sizeof(SocketAddress);
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TCPServer>());
    rclcpp::shutdown();
    return 0;
}
