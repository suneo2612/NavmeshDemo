/**
 * @file filtered_walkable.h
 * @brief 侵食面を除いた歩行面接続を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_VOXELIZATION_EROSION_FILTERED_WALKABLE_H
#define NAVMESH_DEMO_VOXELIZATION_EROSION_FILTERED_WALKABLE_H

#include "../walkable_field.h"

namespace demo::navmesh::voxelization
{
/**
 * @brief 侵食面の接続を除き、残った面の斜め接続を再確認する。
 * @param i_field 侵食前の歩行面と接続。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @return 添字と面を維持し、侵食面の接続を除いた歩行面。侵食面は配列に残る。
 * @throws std::invalid_argument 入力が不正な場合。
 */
WalkableField FilterErodedWalkable(const WalkableField& i_field, const std::vector<std::uint8_t>& i_eroded);
}
#endif
