#include "dfs_mapper.hpp"

DFSMapper::DFSMapper(std::shared_ptr<rclcpp::Node> node,
                     std::shared_ptr<SensorHandler> sh)
: node_(node),
  sensor_(sh),
  current({1, 1})
{
    move_client = node_->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    dirs = {
        {"up",    0, -1},
        {"down",  0,  1},
        {"left", -1,  0},
        {"right", 1,  0}
    };

    stack.push_back(current);
    visited.insert(current);

    RCLCPP_INFO(node_->get_logger(), "DFSMapper inicializado.");
}

bool DFSMapper::gridIsComplete()
{
    int count = 0;


    for (auto &entry : sensor_->grid) {
        if (entry.second.known) {
            count++;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "A contagem atual é: %d", count);
    return (count >= 29 * 29); // 841
}

std::string DFSMapper::direction(const Pos &f, const Pos &t)
{
    int dx = t.x - f.x;
    int dy = t.y - f.y;

    if (dx == 0 && dy == -1) return "up";
    if (dx == 0 && dy ==  1) return "down";
    if (dx == 1 && dy ==  0) return "right";
    if (dx == -1 && dy ==  0) return "left";
    return "";
}


bool DFSMapper::sendMove(const std::string &d)
{
    if (!move_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(),
                    "[DFS] Serviço /move_command indisponível.");
        return false;
    }

    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = d;

    RCLCPP_INFO(node_->get_logger(), "[DFS] Enviando movimento: %s", d.c_str());

    auto future = move_client->async_send_request(req);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}


int DFSMapper::exploreNext()
{
    RCLCPP_INFO(node_->get_logger(),
                "[DFS] Explorando vizinhos a partir de (%d,%d)",
                current.x, current.y);

    for (auto &d : dirs)
    {
        Pos n{current.x + d.dx, current.y + d.dy};

        // Já visitado → pule
        if (visited.count(n)) {
            continue;
        }

        // Descobrir tipo da célula
        CellKind kind = CellKind::UNKNOWN;
        auto it = sensor_->grid.find(n);

        if (it != sensor_->grid.end() && it->second.known) {
            kind = it->second.kind;
        }

        // Parede → ignore
        if (kind == CellKind::WALL) {
            continue;
        }

        if (kind == CellKind::GOAL && !gridIsComplete()) {
            RCLCPP_WARN(node_->get_logger(),
                        "[DFS] GOAL visto no sensor em (%d,%d), mas grid incompleto. NÃO entrar.",
                        n.x, n.y);
            continue;
        }

        if (gridIsComplete()) {
            RCLCPP_INFO(node_->get_logger(), "O MAPEAMENTO FOI COMPLETO!!");
            return 1;
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Tenta mover
        if (sendMove(d.name)) {
            current = n;
            stack.push_back(n);
            visited.insert(n);
            sensor_->sensors_ready = false;

            RCLCPP_INFO(node_->get_logger(),
                        "[DFS] Avançou para (%d,%d)", n.x, n.y);

            return 2;
        }
    }

    // Nenhum vizinho explorável

    return 3;
}


void DFSMapper::backtrack()
{

    Pos cur = stack.back();
    stack.pop_back();
    Pos prev = stack.back();

    std::string dir = direction(cur, prev);

    RCLCPP_INFO(node_->get_logger(),
                "[DFS] Backtracking: (%d,%d) → (%d,%d) via %s",
                cur.x, cur.y, prev.x, prev.y, dir.c_str());

    if (sendMove(dir)) {
        current = prev;
        sensor_->sensors_ready = false;
    }
}

std::vector<std::string> DFSMapper::printMappedGrid()
{
    const int W = 29;
    const int H = 29;

    std::vector<std::string> output;
    output.reserve(H);

    RCLCPP_INFO(node_->get_logger(), "=== MAPA 29x29 ===");

    for (int y = 0; y < H; y++) {
        std::stringstream line;

        for (int x = 0; x < W; x++) {

            Pos p{x, y};
            char symbol = '?'; // default: unknown

            auto it = sensor_->grid.find(p);
            if (it != sensor_->grid.end() && it->second.known) {
                switch (it->second.kind) {
                    case CellKind::WALL:  symbol = 'b'; break;
                    case CellKind::FREE:  symbol = 'f'; break;
                    case CellKind::GOAL:  symbol = 't'; break;
                    default:              symbol = '?'; break;
                }
            }

            // Robô na posição atual
            if (p.x == current.x && p.y == current.y) {
                symbol = 'r';
            }

            line << symbol;
            if (x < W - 1)
                line << ",";
        }

        const std::string row = line.str();
        output.push_back(row);

        // LOG igual antes
        RCLCPP_INFO(node_->get_logger(), "%s", row.c_str());
    }

    return output;
}


bool DFSMapper::step()
{
    if (!sensor_->sensors_ready) {
        RCLCPP_INFO(node_->get_logger(), "[DFS] Aguardando sensores...");
        // RCLCPP_INFO(node_->get_logger(),
        //     "[DEBUG] sensors_ready=%d, current=(%d,%d)",
        //     sensor_->sensors_ready ? 1 : 0,
        //     current.x, current.y);

        return true;
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // Ler sensores da célula atual apenas uma vez
    if (!sensor_->hasSensor(current)) {
        sensor_->updateNeighbors(current);
        sensor_->markSensorRead(current);
    }

    int result = exploreNext();


    if (result == 1) {
        RCLCPP_INFO(node_->get_logger(), "CABOU O MAPA BAITOLÃOOOOO");
        std::vector<std::string> map_grid = printMappedGrid();

        return false;
    }

    // Explora ou faz backtracking
    if (result == 3) {     
        backtrack();
    }

    return true;
}
