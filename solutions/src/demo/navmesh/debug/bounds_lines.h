/**
 * @file bounds_lines.h
 * @brief 地形範囲とタイル枠のデバッグ描画用頂点を作成する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_BOUNDS_LINES_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_BOUNDS_LINES_H

#include "../terrain/terrain.h"
#include <array>
#include <cstddef>
#include <vector>

namespace demo::navmesh::debug
{
inline constexpr auto kMaxDebugTilesPerAxis = std::size_t{64};

/**
 * @brief 色ごとに分けた線分頂点。隣接する２頂点が１本の線分。
 */
struct BoundsLines
{
    std::vector<std::array<float, 3>> terrain; // 地形全体の枠。
    std::vector<std::array<float, 3>> tiles;   // 各タイルの枠。
    std::vector<std::array<float, 3>> links;   // ポリゴン間の接続。
};

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
BoundsLines BuildBoundsLines(const Bounds& i_bounds, std::size_t i_columns, std::size_t i_rows,
                            bool i_showTerrain, bool i_showTiles);
}
#endif
