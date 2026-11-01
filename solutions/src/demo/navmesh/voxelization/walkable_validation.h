/**
 * @file walkable_validation.h
 * @brief 歩行面と隣接情報の整合性を検証する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_WALKABLE_VALIDATION_H
#define NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_WALKABLE_VALIDATION_H

#include "walkable_field.h"

namespace demo::navmesh::voxelization
{
/**
 * @brief 歩行面の格納順、添字、方向と双方向接続を検証する。
 * @param i_field 確認対象。
 * @throws std::invalid_argument データの整合性が壊れている場合。
 */
void ValidateWalkableField(const WalkableField& i_field);
}
#endif
