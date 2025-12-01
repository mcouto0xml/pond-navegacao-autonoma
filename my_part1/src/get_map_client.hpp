#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <vector>
#include <utility>

class GetMapClient : public rclcpp::Node
{
public:
    GetMapClient();

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool request_sent_ = false;

    void send_request_once();
    void handle_response(rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future);

    void print_grid(const std::vector<std::vector<char>>& grid);

    // Função auxiliar para transformar coordenadas → direção ("up", "down"...)
    std::string direction_from_to(std::pair<int,int> a, std::pair<int,int> b);

    // Envia um único comando /move_command
    bool send_move_command(const std::string& direction);
};
