#include "algorithm.hpp"
#include <queue>
#include <algorithm> 

// --------------------------------------------------------
// BFS – Encontra o menor caminho
// --------------------------------------------------------
std::vector<std::pair<int,int>> bfs_shortest_path(
    const std::vector<std::vector<char>>& grid,
    std::pair<int,int> start,
    std::pair<int,int> goal)
{
    int rows = grid.size();
    int cols = grid[0].size();

    std::queue<std::pair<int,int>> q;
    q.push(start);

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    visited[start.first][start.second] = true;

    // Para reconstrução do caminho
    std::vector<std::vector<std::pair<int,int>>> parent(
        rows,
        std::vector<std::pair<int,int>>(cols, {-1, -1})
    );

    // movimentos possíveis
    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        if (x == goal.first && y == goal.second) {
            break; 
        }

        for (int k = 0; k < 4; ++k) {
            int nx = x + dx[k];
            int ny = y + dy[k];

            // Fora do mapa
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols) continue;

            // Parede
            if (grid[nx][ny] == 'b') continue;

            if (!visited[nx][ny]) {
                visited[nx][ny] = true;
                parent[nx][ny] = {x, y};
                q.push({nx, ny});
            }
        }
    }
    // Reconstrução do caminho
    std::vector<std::pair<int,int>> path;
    std::pair<int,int> cur = goal;

    // Se o goal não tem pai e não é o start → sem caminho
    if (parent[goal.first][goal.second].first == -1 &&
        parent[goal.first][goal.second].second == -1 &&
        !(goal == start))
    {
        return path;
    }

    while (!(cur.first == -1 && cur.second == -1)) {
        path.push_back(cur);
        cur = parent[cur.first][cur.second];
    }

    std::reverse(path.begin(), path.end());
    return path;
}


// --------------------------------------------------------
// direction_from_to – Converte movimento em comando string
// --------------------------------------------------------
std::string direction_from_to(std::pair<int,int> a, std::pair<int,int> b)
{
    if (b.first == a.first - 1 && b.second == a.second) return "up";
    if (b.first == a.first + 1 && b.second == a.second) return "down";
    if (b.first == a.first && b.second == a.second - 1) return "left";
    if (b.first == a.first && b.second == a.second + 1) return "right";

    return "none"; 
}
