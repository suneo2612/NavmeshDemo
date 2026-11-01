/**
 * @file grid_position.h
 * @brief ボクセル格子の頂点を描画用のワールド座標へ変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEBUG_GRID_POSITION_H
#define NAVMESH_DEMO_DEBUG_GRID_POSITION_H

#include "../navmesh.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace demo::navmesh::debug
{
/**
 * @brief 共通格子の頂点を端の短いセルも考慮してワールド座標に変換する。
 * @param i_navmesh 構築済みの地形とボクセル設定。
 * @param i_x 共通格子上の X 座標。
 * @param i_z 共通格子上の Z 座標。
 * @param i_floor 地形最低 Y からの高さセル数。
 * @param i_offset 描画時に高さへ加える値。
 * @return 描画用のワールド座標。
 * @throws std::overflow_error 座標が表現できない場合。
 */
std::array<float, 3> GridPosition(const Navmesh& i_navmesh, double i_x, double i_z,
                                  std::uint32_t i_floor, double i_offset);

/**
 * @brief 整数格子の頂点をワールド座標に変換する。
 * @param i_navmesh 構築済みの地形とボクセル設定。
 * @param i_x 共通格子上の X 座標。
 * @param i_z 共通格子上の Z 座標。
 * @param i_floor 地形最低 Y からの高さセル数。
 * @param i_offset 描画時に高さへ加える値。
 * @return 描画用のワールド座標。
 * @throws std::overflow_error 座標が表現できない場合。
 */
std::array<float, 3> GridPosition(const Navmesh& i_navmesh, std::size_t i_x, std::size_t i_z,
                                  std::uint32_t i_floor, double i_offset);
}
#endif
