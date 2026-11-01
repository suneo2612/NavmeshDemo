/**
 * @file core_erosion.h
 * @brief キャラクター半径に応じたタイル所有面の侵食を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_VOXELIZATION_EROSION_CORE_EROSION_H
#define NAVMESH_DEMO_VOXELIZATION_EROSION_CORE_EROSION_H

#include "../tiles/tile_regions.h"
#include "../walkable_field.h"
#include <cstdint>
#include <vector>

namespace demo::navmesh::voxelization
{
/**
 * @brief 所有セル全体から歩行不能境界までの距離を調べる。
 * @param i_heightfield タイルの作業範囲の高さ場。範囲は共通地形の座標。
 * @param i_region タイルの所有・管理・作業範囲。
 * @param i_walkable 作業範囲の歩行面と接続。
 * @param i_managed 各面が管理範囲内なら 1。
 * @param i_coreSpans 所有範囲の歩行面の添字。
 * @param i_agentRadius キャラクター半径。
 * @return 歩行面と同じ添字の配列。所有面が侵食された場合は 1、その他は 0。
 * @throws std::invalid_argument 入力が不正な場合。
 * @throws std::length_error 判定回数の上限を超えた場合。
 * @details 高さ方向のキャラクター範囲が接する四方向の歩行不能境界に対し、
 * セル矩形と境界線の最短距離が半径以下なら侵食する。十分離れた階層は区別する。
 * 近接している別階層の境界は保守的に侵食する場合がある。
 * 所有範囲外は判定対象ではなく、戻り値の 0 は歩行可能を意味しない。
 */
std::vector<std::uint8_t> BuildCoreErosion(const Heightfield& i_heightfield,
                                           const TileRegion& i_region,
                                           const WalkableField& i_walkable,
                                           const std::vector<std::uint8_t>& i_managed,
                                           const std::vector<std::size_t>& i_coreSpans,
                                           float i_agentRadius);
}
#endif
