#include "rclcpp/rclcpp.hpp"
#include "sensor_handler.hpp"
#include "dfs_mapper.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("maze_mapper");
    RCLCPP_INFO(node->get_logger(), "Nó maze_mapper iniciado.");

    auto sensor = std::make_shared<SensorHandler>(node);
    auto dfs    = std::make_shared<DFSMapper>(node, sensor);

    // Flag de parada
    bool finished = false;

    rclcpp::TimerBase::SharedPtr timer;

    timer = node->create_wall_timer(
        std::chrono::milliseconds(200),
        [dfs, &finished, node]() {
            bool result = dfs->step();
            if (!result) {
                RCLCPP_INFO(node->get_logger(),
                            "[TIMER] DFS terminou. Sinalizando parada...");
                finished = true;     // <-- apenas sinaliza!
            }
        }
    );

    // Loop do executor — sem sleeps! sem rate!
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    while (rclcpp::ok()) {
        exec.spin_some();

        if (finished) {
            RCLCPP_INFO(node->get_logger(), "[MAIN] Parando timer e encerrando...");
            timer->cancel();
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}