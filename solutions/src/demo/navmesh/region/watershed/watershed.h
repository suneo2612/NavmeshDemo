/**
 * @file watershed.h
 * @brief 距離場に基づくタイル所有面の流域分割を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_REGION_WATERSHED_WATERSHED_H
#define NAVMESH_DEMO_REGION_WATERSHED_WATERSHED_H

#include "../region.h"
#include "../../voxelization/distance_field.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace demo::navmesh::region
{
/**
 * @brief 侵食後の距離場の極大部から所有面を流域に分割する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_distance 面ごとの平滑化後の距離。最大整数値は未到達。
 * @return 面と同じ添字を持つタイル内の領域 ID。0 は非対象。
 * @throws std::invalid_argument 入力や未到達面の接続が不正な場合。
 * @throws std::length_error 領域数が上限を超えた場合。
 * @details 四方向接続で同じ距離の平坦部をまとめ、周囲に高い距離がない平坦部を
 * 始点として高い距離から順に広げる。競合は添字と領域 ID の順で決める。
 * 未到達の接続成分は有限距離の高さと比較せず、平坦な独立領域として扱う。
 * 一つの領域に同じ XZ セルの複数階層を含めない。
 */
Regions BuildWatershedRegions(const voxelization::WalkableField& i_field,
                               const std::vector<std::size_t>& i_coreSpans,
                               const std::vector<std::uint8_t>& i_eroded,
                               const voxelization::DistanceField& i_distance);
}
#endif
