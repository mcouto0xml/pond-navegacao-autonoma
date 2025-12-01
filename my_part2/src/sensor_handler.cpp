#include "sensor_handler.hpp"

SensorHandler::SensorHandler(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
    sub = node_->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors",
        rclcpp::QoS(10),
        std::bind(&SensorHandler::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "SensorHandler 3 inicializado.");
}


void SensorHandler::callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg)
{
    last_msg = *msg;
    sensors_ready = true;

    RCLCPP_INFO(node_->get_logger(), "[SensorHandler] Sensores recebidos! up=%s left=%s right=%s down=%s", msg->up.c_str(), msg->left.c_str(), msg->right.c_str(), msg->down.c_str());
}

bool SensorHandler::hasSensor(const Pos &p) {
    return sensor_read.find(p) != sensor_read.end();
}

void SensorHandler::markSensorRead(const Pos &p) {
    sensor_read.insert(p);
}

CellKind SensorHandler::interpret(const std::string &s) {
    if (s == "wall" || s == "b") return CellKind::WALL;
    if (s == "goal" || s == "t") return CellKind::GOAL;
    if (s == "free" || s == "f" || s == " ") return CellKind::FREE;
    return CellKind::UNKNOWN;
}

void SensorHandler::updateCell(const Pos &p, const std::string &val) {
    auto &cell = grid[p];
    cell.known = true;
    cell.kind = interpret(val);
}

void SensorHandler::updateNeighbors(const Pos &p) {
    if (!sensors_ready) return;

    updateCell({p.x, p.y-1}, last_msg.up);
    updateCell({p.x, p.y+1}, last_msg.down);
    updateCell({p.x-1, p.y}, last_msg.left);
    updateCell({p.x+1, p.y}, last_msg.right);

    updateCell({p.x-1, p.y-1}, last_msg.up_left);
    updateCell({p.x+1, p.y-1}, last_msg.up_right);
    updateCell({p.x-1, p.y+1}, last_msg.down_left);
    updateCell({p.x+1, p.y+1}, last_msg.down_right);
}
