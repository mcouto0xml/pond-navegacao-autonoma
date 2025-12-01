#pragma once
#include <vector>
#include <string>
#include <set>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "sensor_handler.hpp"

class DFSMapper {
public:
    DFSMapper(std::shared_ptr<rclcpp::Node> node,
              std::shared_ptr<SensorHandler> sensor);

    bool step(); // chamada periódica via timer

private:
    struct Dir { std::string name; int dx; int dy; };

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<SensorHandler> sensor_;

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client;

    std::vector<Dir> dirs;
    std::vector<Pos> stack;
    std::set<Pos> visited;

    Pos current;

    int exploreNext();
    void backtrack();
    bool gridIsComplete();
    bool sendMove(const std::string &direction);
    std::vector<std::string> printMappedGrid();
    std::string direction(const Pos &from, const Pos &to);
};
