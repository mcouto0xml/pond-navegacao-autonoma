#pragma once
#include <map>
#include <set>
#include <memory>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"

enum class CellKind : int8_t { UNKNOWN=-1, FREE=0, WALL=1, GOAL=2 };

struct Pos {
    int x, y;
    bool operator<(const Pos &o) const {
        return (y < o.y) || (y == o.y && x < o.x);
    }
    bool operator==(const Pos &o) const { return x == o.x && y == o.y; }
};

struct Cell {
    bool known = false;
    CellKind kind = CellKind::UNKNOWN;
};

class SensorHandler {
public:
    explicit SensorHandler(std::shared_ptr<rclcpp::Node> node);

    void callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg);

    bool hasSensor(const Pos &p);
    void markSensorRead(const Pos &p);

    void updateNeighbors(const Pos &p);
    bool sensors_ready = false;

    std::map<Pos, Cell> grid;

private:
    cg_interfaces::msg::RobotSensors last_msg;
    std::shared_ptr<rclcpp::Node> node_;
    std::set<Pos> sensor_read;

    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sub;

    CellKind interpret(const std::string &s);
    void updateCell(const Pos &p, const std::string &val);
};
