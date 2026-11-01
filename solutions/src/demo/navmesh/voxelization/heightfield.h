/**
 * @file heightfield.h
 * @brief 三角形の占有範囲を高さ方向の区間として記録する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_HEIGHTFIELD_H
#define NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_HEIGHTFIELD_H

#include "../terrain/terrain.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace demo::navmesh::voxelization
{
/**
 * @brief 地形のラスタライズに必要な設定。
 */
struct RasterSettings
{
    float cellSize = 0.3F;       // XZ 方向のセル幅。
    float cellHeight = 0.2F;     // 高さ方向の刻み幅。
    float maxSlopeDegrees = 45; // 歩行面とみなす最大傾斜角。
};

/**
 * @brief ボクセル化の寸法と最大傾斜を検証する。
 * @param i_settings 検証する設定。
 * @throws std::invalid_argument 設定が不正な場合。
 */
void ValidateRasterSettings(const RasterSettings& i_settings);

/**
 * @brief ある XZ セルの地形が占める高さの半開区間。
 * @details 高さは Heightfield の最低 Y 座標からのセル数。
 */
struct SolidSpan
{
    std::uint32_t minimum = 0; // 占有区間の下端。
    std::uint32_t maximum = 0; // 占有区間の上端。
    bool walkableTop = false;  // 区間上端の面が傾斜条件を満たすか。
};

/**
 * @brief XZ の各セルに高さ方向の占有区間を保持する。
 */
struct Heightfield
{
    Bounds bounds;                               // 入力地形の範囲。
    RasterSettings settings;                     // 適用したボクセル化の設定。
    std::size_t width = 0;                       // X 方向のセル数。
    std::size_t depth = 0;                       // Z 方向のセル数。
    std::vector<std::vector<SolidSpan>> columns; // Z 行・X 列順の占有区間。

    /**
     * @brief 指定した XZ セルにある地形スパンを取得する。
     * @param i_x X 方向のセル添字。
     * @param i_z Z 方向のセル添字。
     * @return 指定セルの地形スパンを高さ順に格納した配列への参照。
     * @throws std::out_of_range セルが範囲外の場合。
     */
    const std::vector<SolidSpan>& Column(std::size_t i_x, std::size_t i_z) const;
};

/**
 * @brief 地形三角形を XZ セルへ切り取り、Y 方向の占有区間を構築する。
 * @param i_terrain ワールド空間の地形。
 * @param i_settings セル幅・高さ・最大傾斜角。
 * @return 複数階層の地形を保持する高さ場。
 * @throws std::invalid_argument 入力や設定が不正な場合。
 * @throws std::length_error セル数などが安全上の上限を超える場合。
 * @details 歩行面判定は反時計回りの上向き面を対象とする。
 * 区間の量子化は地形範囲の最低 Y を基準とし、面の厚みを少なくとも１セルとする。
 * セル境界に接する三角形は両側のセルに記録するため、境界では保守的に占有される。
 */
Heightfield Rasterize(const Terrain& i_terrain, const RasterSettings& i_settings);
}
#endif
