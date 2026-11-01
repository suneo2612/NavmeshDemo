/**
 * @file voxel_instances.h
 * @brief タイルの占有区間と距離を描画用ボクセルに変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_VOXEL_INSTANCES_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_VOXEL_INSTANCES_H

#include "../navmesh.h"
#include <array>
#include <optional>
#include <vector>

namespace demo::navmesh::debug
{
/**
 * @brief ボクセルに適用する距離の表示方法。
 */
enum class VoxelColorMode
{
    Terrain,  // 距離を表示しない。
    Raw,      // 平滑化前の距離。
    Smoothed, // 平滑化後の距離。
    Regions   // タイル内の領域ごとの色。
};

/**
 * @brief 共通の単位立方体に適用するワールド座標と色。
 */
struct VoxelInstance
{
    std::array<float, 3> minimum; // ワールド空間の最小座標。
    std::array<float, 3> size;    // 各軸の大きさ。
    std::array<float, 3> color;   // 表示色。
};

/**
 * @brief 全タイルの所有面から表示用の有限最大距離を求める。
 * @param i_navmesh 構築済みの地形とタイル。
 * @param i_mode 元または平滑化後の距離。
 * @param i_afterErosion 侵食後の距離場を使用する場合は true。
 * @return 有限距離がない場合または全てゼロの場合は 0。
 * @throws std::invalid_argument 距離の種類またはボクセル結果が不正な場合。
 */
std::uint32_t MaximumVoxelDistance(const Navmesh& i_navmesh, VoxelColorMode i_mode,
                                   bool i_afterErosion = false);

/**
 * @brief 所有セルの占有区間だけをボクセルに展開する。
 * @param i_navmesh 構築済みの地形とタイル。
 * @param i_mode 距離の表示方法。
 * @param i_fixedMaximum 指定時はこの値を白の基準に固定する。
 * @param i_showEroded 侵食面を赤色で表示する場合は true。
 * @param i_afterErosion 侵食後の距離場を使用する場合は true。
 * @return 距離に応じて着色したボクセル。
 * @throws std::invalid_argument ボクセル結果が存在しない場合。
 * @throws std::length_error ボクセル数が表示の上限を超える場合。
 * @throws std::overflow_error 描画する座標を表現できない場合。
 * @details 距離表示で基準未指定時は全タイルの所有面における有限最大値で正規化する。
 * 歩行面を持つ占有区間の最上段のみ距離色にし、指定時は侵食面を赤色にする。
 * 侵食後の距離表示または領域表示で赤色を無効にした場合、侵食面は歩行面以外の色で表示する。
 */
std::vector<VoxelInstance> BuildVoxelInstances(const Navmesh& i_navmesh, VoxelColorMode i_mode,
                                               std::optional<std::uint32_t> i_fixedMaximum = std::nullopt,
                                               bool i_showEroded = false, bool i_afterErosion = false);
}
#endif
