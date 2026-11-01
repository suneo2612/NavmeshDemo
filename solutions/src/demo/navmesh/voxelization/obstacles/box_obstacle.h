/**
 * @file box_obstacle.h
 * @brief 箱型障害物を高さ場の占有区間として扱う。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_VOXELIZATION_OBSTACLES_BOX_OBSTACLE_H
#define NAVMESH_DEMO_VOXELIZATION_OBSTACLES_BOX_OBSTACLE_H

#include "../tiles/tile_regions.h"
#include <cstdint>

namespace demo::navmesh::voxelization
{
/**
 * @brief ワールド空間の箱型障害物。
 */
struct BoxObstacle
{
    std::uint64_t id = 0; // 個別削除に用いる識別子。
    Bounds bounds;        // 占有するワールド空間の範囲。
};

/**
 * @brief 障害物の範囲を検証する。
 * @param i_bounds 箱の範囲。
 * @throws std::invalid_argument 非有限値または幅のない箱の場合。
 */
void ValidateBoxObstacle(const Bounds& i_bounds);

/**
 * @brief タイルの管理セルと障害物が XZ 上で接触するか判定する。
 * @param i_field 共通格子を表す地形の高さ場。
 * @param i_managed 管理セルの範囲。
 * @param i_bounds 検索対象の箱。
 * @return 接触または重複する場合は true。
 * @throws std::invalid_argument 高さ場、セル範囲、または箱が不正な場合。
 * @details 床より上の障害物も頭上空間を塞ぐため Y 範囲では除外しない。
 */
bool ObstacleTouches(const Heightfield& i_field, const CellRect& i_managed, const Bounds& i_bounds);

/**
 * @brief 作業範囲に入る障害物を作業範囲の高さ場に重ねる。
 * @param i_base 元の共通高さ場。
 * @param i_region 対象タイルのセル範囲。
 * @param i_obstacles 検証済み障害物の一覧。
 * @return 作業範囲の座標を原点とする高さ場。高さの原点は元の地形と同じ。
 * @throws std::invalid_argument 高さ場、セル範囲、または障害物が不正な場合。
 * @throws std::length_error 障害物の高さが表現できる範囲を超えた場合。
 */
Heightfield ApplyBoxObstacles(const Heightfield& i_base, const TileRegion& i_region,
                              const std::vector<BoxObstacle>& i_obstacles);
}
#endif
