#pragma once

#include <vector>
#include <utility>
#include <string>

// BFS que retorna o caminho mais curto de start até goal
// Cada posição é (linha, coluna)
std::vector<std::pair<int,int>> bfs_shortest_path(
    const std::vector<std::vector<char>>& grid,
    std::pair<int,int> start,
    std::pair<int,int> goal);

// Converte dois pontos consecutivos em uma direção (up, down, left, right)
std::string direction_from_to(std::pair<int,int> a, std::pair<int,int> b);
