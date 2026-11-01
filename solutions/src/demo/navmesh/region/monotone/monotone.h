/**
 * @file monotone.h
 * @brief タイル所有面の単調領域分割を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_REGION_MONOTONE_MONOTONE_H
#define NAVMESH_DEMO_REGION_MONOTONE_MONOTONE_H

#include "../../voxelization/walkable_field.h"
#include "../region.h"

namespace demo::navmesh::region
{
using MonotoneRegions = Regions;

/**
 * @brief 所有範囲の侵食されていない歩行面を単調領域に分割する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @return 面と同じ添字を持つタイル内の領域 ID。0 は非対象。
 * @throws std::invalid_argument 面、接続または侵食結果が不正な場合。
 * @throws std::length_error 領域数が上限を超える場合。
 * @details X 方向の連続列を Z 行ごとに作り、前の行と一対一のときだけ継続する。
 * 分岐や合流では新しい領域とするため、各行は一つの連続列に収まる。
 */
MonotoneRegions BuildMonotoneRegions(const voxelization::WalkableField& i_field,
                                     const std::vector<std::size_t>& i_coreSpans,
                                     const std::vector<std::uint8_t>& i_eroded);
}
#endif
