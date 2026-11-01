/**
 * @file obstacle_picking.h
 * @brief 画面位置から地形と障害物を選択する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_OBSTACLE_PICKING_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_OBSTACLE_PICKING_H

#include "../navmesh.h"
#include "../../../render/camera.h"
#include <optional>

namespace demo::navmesh::debug
{
/**
 * @brief ワールド空間の選択用の光線。
 */
struct PickRay
{
    general::vec::Vector3 origin;    // カメラ位置。
    general::vec::Vector3 direction; // 光線の方向。
};

/**
 * @brief 光線と地形の最も近い交点。
 */
struct TerrainHit
{
    general::vec::Vector3 position; // 交点のワールド座標。
    double distance = 0;            // 光線の開始位置からの距離。
};

/**
 * @brief 画面のマウス位置から投影に対応した光線を作る。
 * @param i_camera カメラ。
 * @param i_x ウィンドウ内の横座標。
 * @param i_y ウィンドウ内の縦座標。
 * @param i_width ウィンドウの論理幅。
 * @param i_height ウィンドウの論理高さ。
 * @param i_aspect フレームバッファーの横縦比。
 * @return 画面外の場合は空、それ以外はワールド空間の光線。
 * @throws std::invalid_argument 画面寸法または座標が不正な場合。
 */
std::optional<PickRay> ScreenPickRay(const render::Camera& i_camera, float i_x, float i_y,
                                     float i_width, float i_height, float i_aspect);

/**
 * @brief 地形の三角形と光線の最も近い交点を求める。
 * @param i_terrain ワールド空間の地形。
 * @param i_ray 検索する光線。
 * @return 交差しない場合は空。
 * @throws std::invalid_argument 光線が不正な場合。
 */
std::optional<TerrainHit> PickTerrain(const Terrain& i_terrain, const PickRay& i_ray);

/**
 * @brief 光線上の最も手前の障害物を探す。
 * @param i_obstacles ワールド空間の障害物。
 * @param i_ray 検索する光線。
 * @param i_maxDistance この距離より手前だけを対象とする。
 * @return 対象がない場合は空、それ以外は障害物の識別子。
 * @throws std::invalid_argument 光線または距離が不正な場合。
 */
std::optional<std::uint64_t> PickObstacle(const std::vector<voxelization::BoxObstacle>& i_obstacles,
                                          const PickRay& i_ray, double i_maxDistance);
}
#endif
