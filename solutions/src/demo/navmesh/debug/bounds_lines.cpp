/**
 * @file bounds_lines.cpp
 * @brief 地形とタイルの AABB を線分に変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "bounds_lines.h"
#include <array>
#include <cmath>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kVerticesPerBox = std::size_t{24};

/**
 * @brief AABB の１２本の辺を線分頂点として追加する。
 * @param i_bounds 描画する範囲。
 * @param io_vertices 追加先の頂点配列。
 */
void AppendBox(const Bounds& i_bounds, std::vector<std::array<float, 3>>& io_vertices)
{
    // AABB の8頂点を作り、12辺の端点を線分描画用に2頂点ずつ追加する。
    const auto& lo = i_bounds.minimum;
    const auto& hi = i_bounds.maximum;
    const auto corners = std::array<std::array<float, 3>, 8>{{
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z},
        {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}}};
    constexpr auto edges = std::array<std::array<std::size_t, 2>, 12>{{
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
    for (const auto& edge : edges)
    {
        io_vertices.push_back(corners[edge[0]]);
        io_vertices.push_back(corners[edge[1]]);
    }
}
}

/**
 * @brief 地形とタイルの枠を線分として生成する。
 * @param i_bounds ワールド空間の地形範囲。
 * @param i_columns X 方向のタイル数。
 * @param i_rows Z 方向のタイル数。
 * @param i_showTerrain 地形全体の枠を生成する場合は true。
 * @param i_showTiles タイル枠を生成する場合は true。
 * @return 地形とタイルの線分頂点。
 * @throws std::invalid_argument 範囲またはタイル数が不正な場合。
 */
BoundsLines BuildBoundsLines(const Bounds& i_bounds, const std::size_t i_columns, const std::size_t i_rows,
                            const bool i_showTerrain, const bool i_showTiles)
{
    // 描画に非有限値を渡さず、平坦な地形の高さゼロは許容する。
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (!std::isfinite(i_bounds.minimum[axis]) || !std::isfinite(i_bounds.maximum[axis]) ||
            i_bounds.minimum[axis] > i_bounds.maximum[axis])
        {
            throw std::invalid_argument("デバッグ表示の地形範囲が不正です。");
        }
    }
    if (i_columns == 0 || i_rows == 0 || i_columns > kMaxDebugTilesPerAxis || i_rows > kMaxDebugTilesPerAxis)
    {
        throw std::invalid_argument("デバッグ表示のタイル分割数が不正です。");
    }
    auto result = BoundsLines{};
    // 表示を要求された形状だけを作り、未使用の頂点配列は空のままにする。
    if (i_showTerrain)
    {
        result.terrain.reserve(kVerticesPerBox);
        AppendBox(i_bounds, result.terrain);
    }
    if (i_showTiles)
    {
        // 既存の格子計算を使い、構築時と同じ境界に枠を置く。
        const auto grid = TileGrid(i_bounds, i_columns, i_rows);
        result.tiles.reserve(i_columns * i_rows * kVerticesPerBox);
        for (auto row = std::size_t{}; row < i_rows; ++row)
        {
            for (auto column = std::size_t{}; column < i_columns; ++column)
            {
                AppendBox(grid.TileBounds(column, row), result.tiles);
            }
        }
    }
    return result;
}
}
