/**
 * @file distance_field.h
 * @brief 歩行面の境界までの距離場を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_DISTANCE_FIELD_H
#define NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_DISTANCE_FIELD_H

#include "walkable_field.h"
#include <cstdint>
#include <vector>

namespace demo::navmesh::voxelization
{
/**
 * @brief 歩行面ごとの境界までの距離。
 * @details 距離は格子上の近似値で、直交移動を 2、斜め移動を 3 とする。
 * 配列の添字は入力の WalkableField::spans と対応する。
 */
struct DistanceField
{
    std::vector<std::uint32_t> distances; // 歩行面から最寄り境界までの距離。
};

/**
 * @brief 歩行できない方向を持つ面を境界として距離場を構築する。
 * @param i_field 八方向の双方向接続を持つ歩行面。
 * @return 対応する面ごとの距離場。境界の距離は 0。
 * @throws std::invalid_argument 歩行面や隣接接続が不正な場合。
 * @throws std::length_error 距離が表現できる範囲を超えた場合。
 * @details 直交四方向のいずれかに移動できない面を境界とし、
 * 接続されている面だけを通って距離を伝播する。複数階層の接続も考慮する。
 */
DistanceField BuildDistanceField(const WalkableField& i_field);
}
#endif
