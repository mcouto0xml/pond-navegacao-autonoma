#include "get_map_client.hpp"
#include "algorithm.hpp"  // BFS
#include <thread>

GetMapClient::GetMapClient() : Node("get_map_client")
{
    client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    // Timer só para disparar uma vez o pedido de mapa
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&GetMapClient::send_request_once, this)
    );
}

void GetMapClient::send_request_once()
{
    if (request_sent_) return;

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Aguardando serviço /get_map...");
        return;
    }

    request_sent_ = true;
    timer_->cancel();

    auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();

    client_->async_send_request(
        req,
        std::bind(&GetMapClient::handle_response, this, std::placeholders::_1)
    );
}

void GetMapClient::handle_response(rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future)
{
    auto response = future.get();

    auto &flat = response->occupancy_grid_flattened;
    int rows = response->occupancy_grid_shape[0];
    int cols = response->occupancy_grid_shape[1];

    RCLCPP_INFO(this->get_logger(),
        "Mapa recebido! Dimensões: %d x %d (%ld células)",
        rows, cols, flat.size());

    // Converter para grid 2D
    std::vector<std::vector<char>> grid(rows, std::vector<char>(cols));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::string cell = flat[i * cols + j];
            grid[i][j] = cell.empty() ? '?' : cell[0];
        }
    }

    RCLCPP_INFO(this->get_logger(), "Mapa completo (grid 2D):");
    print_grid(grid);

    // Encontrar r (robô) e t (target)
    int r_x = -1, r_y = -1;
    int t_x = -1, t_y = -1;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (grid[i][j] == 'r') { r_x = i; r_y = j; }
            if (grid[i][j] == 't') { t_x = i; t_y = j; }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Robô em: (%d, %d)", r_x, r_y);
    RCLCPP_INFO(this->get_logger(), "Alvo em: (%d, %d)", t_x, t_y);

    // Rodar BFS
    auto path = bfs_shortest_path(grid, {r_x, r_y}, {t_x, t_y});

    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "Nenhum caminho encontrado!");
        this->get_node_base_interface()->get_context()->shutdown("done");

        return;
    }

    RCLCPP_INFO(this->get_logger(), "Caminho encontrado com %ld passos:", path.size()-1);
    for (auto &p : path)
        RCLCPP_INFO(this->get_logger(), "(%d, %d)", p.first, p.second);


    // =============================
    // Enviar movimentos ao robô
    // =============================

    for (size_t i = 1; i < path.size(); i++)
    {
        auto dir = direction_from_to(path[i-1], path[i]);

        if (dir == "none") {
            RCLCPP_ERROR(this->get_logger(), "Erro interno: direção inválida");
            break;
        }

        if (!send_move_command(dir)) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao enviar comando: %s", dir.c_str());
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(this->get_logger(), "Movimentação concluída.");
    this->get_node_base_interface()->get_context()->shutdown("done");

}



bool GetMapClient::send_move_command(const std::string& direction)
{
    if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Serviço /move_command indisponível.");
        return false;
    }

    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();        // Utiliza smart pointer para garantir que não seja destruido com outros elementos precisando
    req->direction = direction;

    RCLCPP_INFO(this->get_logger(), "Movendo agora: %s", direction.c_str());

    auto future = move_client_->async_send_request(req);
    
    // if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    //     RCLCPP_ERROR(this->get_logger(), "Timeout ao esperar resposta do /move_command");
    //     return false;
    // }


    return true;
}




std::string GetMapClient::direction_from_to(std::pair<int,int> a, std::pair<int,int> b)
{
    if (b.first == a.first - 1 && b.second == a.second) return "up";
    if (b.first == a.first + 1 && b.second == a.second) return "down";
    if (b.first == a.first && b.second == a.second - 1) return "left";
    if (b.first == a.first && b.second == a.second + 1) return "right";

    return "none";
}


void GetMapClient::print_grid(const std::vector<std::vector<char>>& grid)
{
    for (const auto& row : grid) {
        std::string line(row.begin(), row.end());
        RCLCPP_INFO(this->get_logger(), "%s", line.c_str());
    }
}
