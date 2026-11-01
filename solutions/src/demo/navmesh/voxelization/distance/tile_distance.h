/**
 * @file tile_distance.h
 * @brief タイルの管理範囲における距離場と平滑化を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_VOXELIZATION_DISTANCE_TILE_DISTANCE_H
#define NAVMESH_DEMO_VOXELIZATION_DISTANCE_TILE_DISTANCE_H

#include "../distance_field.h"
#include <optional>

namespace demo::navmesh::voxelization
{
/**
 * @brief 管理範囲の距離場と一度平滑化した値。
 * @details 管理範囲外と境界に到達できない面は最大整数値で表す。
 */
struct TileDistance
{
    DistanceField raw;                   // 平滑化前の距離。
    DistanceField smoothed;              // 平滑化後の距離。
    std::optional<std::uint32_t> maximum; // 管理範囲内の有限な最大距離。
};

/**
 * @brief 管理範囲内の歩行面を前後に走査して距離を求め、平滑化する。
 * @param i_field 管理範囲の外側にも接続判定用の面を持つ歩行面。
 * @param i_managed 各面が管理範囲内なら 1、外側なら 0。
 * @param i_cellSize XZ セルの幅。
 * @param i_agentRadius キャラクターの半径。0 以上。
 * @return 入力面に対応する距離場。
 * @throws std::invalid_argument 入力が不正、または距離伝播に矛盾がある場合。
 * @throws std::length_error 距離が表現できる範囲を超えた場合。
 * @details 管理範囲外の接続先は距離の伝播に使わないが、境界の誤判定を防ぐため
 * 直交方向の接続の有無には使用する。
 */
TileDistance BuildTileDistance(const WalkableField& i_field,
                               const std::vector<std::uint8_t>& i_managed,
                               float i_cellSize, float i_agentRadius);
}
#endif
