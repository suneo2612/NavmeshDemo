/**
 * @file region.h
 * @brief タイルごとの領域分割方式と共通の結果を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_REGION_REGION_H
#define NAVMESH_DEMO_REGION_REGION_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace demo::navmesh::voxelization { struct WalkableField; }

namespace demo::navmesh::region
{
/**
 * @brief 歩行面を分割する方法。
 */
enum class PartitionMethod
{
    Monotone,  // 走査行を用いる単調分割。
    Watershed  // 境界距離の高い場所から広げる流域分割。
};

/**
 * @brief タイル内で完結する領域の識別子。
 */
struct Regions
{
    std::vector<std::uint32_t> ids; // 歩行面ごとの領域。0 は領域なし。
    std::uint32_t count = 0;       // タイル内の領域数。
};

/**
 * @brief 分割方式が対応しているか検証する。
 * @param i_method 確認する分割方式。
 * @throws std::invalid_argument 対応していない場合。
 */
void ValidatePartitionMethod(PartitionMethod i_method);

/**
 * @brief 所有面と領域の被覆、階層の重複および四方向の接続を検証する。
 * @param i_field 侵食後の歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 面ごとの領域。
 * @return 領域 ID を添字とする面数。添字 0 は未使用。
 * @throws std::invalid_argument 入力や領域が不正な場合。
 */
std::vector<std::size_t> ValidateRegions(const voxelization::WalkableField& i_field,
                                         const std::vector<std::size_t>& i_coreSpans,
                                         const std::vector<std::uint8_t>& i_eroded,
                                         const Regions& i_regions);
}
#endif
