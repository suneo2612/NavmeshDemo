/**
 * @file contour.h
 * @brief タイル内の領域境界と穴の輪郭を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_CONTOUR_CONTOUR_H
#define NAVMESH_DEMO_CONTOUR_CONTOUR_H

#include "../region/region.h"
#include "../voxelization/walkable_field.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace demo::navmesh::contour
{
/**
 * @brief 輪郭を構成する、歩行面の一辺。
 * @details 辺の高さは参照先の歩行面の floor。隣接先は同じ面の neighbors から参照できる。
 */
struct Edge
{
    std::size_t span;                    // retained.spans 内の境界面。
    voxelization::Direction direction;   // 面から見た辺の方向。
};

/**
 * @brief 四方向の辺をたどった閉じた輪郭。
 * @details XZ 平面上の面積が正なら外周、負なら穴。0 は重なった辺を含む退化輪郭。
 * 辺の接触や段差を省略せず保持するため、輪郭が一点で自身に接する場合もある。
 */
struct Contour
{
    std::uint32_t region;                // タイル内の領域 ID。
    std::vector<Edge> edges;             // 領域が右側になる順番の境界辺。
    std::int64_t signedAreaTwice = 0;    // XZ セル座標における符号付き面積の２倍。
};

/**
 * @brief 同じ高さで接続する面ごとの輪郭と元の領域。
 * @details patches の ID を contours の region と各配列の添字に使う。
 * 添字 0 は未使用で、元の領域 ID は sourceRegions から参照できる。
 */
struct FlatContours
{
    region::Regions patches;                  // 同じ領域・高さで接続する面の識別子。
    std::vector<std::uint32_t> sourceRegions; // 分割前の領域 ID。
    std::vector<std::uint32_t> floors;         // 地形の最低 Y からの高さセル数。
    std::vector<Contour> contours;            // 平坦な連結面ごとの境界。
    std::vector<std::vector<std::size_t>> selectedEdges; // 輪郭ごとの、保持する頂点から始まる元の辺の添字。
};

/**
 * @brief 所有領域の境界辺を接続に沿って閉じた輪郭にまとめる。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 外周・穴・退化輪郭。座標と高さは入力の歩行面から参照する。
 * @throws std::invalid_argument 入力や領域の接続が不正な場合。
 * @throws std::length_error 輪郭または面積が上限を超える場合。
 * @details タイル外の面は同一領域に含めず、タイル境界に辺を残す。
 * 面の高さの異なる辺は省略せず、元の面と方向をそのまま保持する。
 */
std::vector<Contour> BuildContours(const voxelization::WalkableField& i_field,
                                   const std::vector<std::size_t>& i_coreSpans,
                                   const std::vector<std::uint8_t>& i_eroded,
                                   const region::Regions& i_regions);

/**
 * @brief 領域を同じ高さの連結面に分割し、それぞれの輪郭を作る。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 平坦な連結面ごとの輪郭と、元の領域・高さの対応。
 * @throws std::invalid_argument 入力や領域の接続が不正な場合。
 * @throws std::length_error 領域または輪郭が上限を超える場合。
 * @details 段差を越える接続は別の輪郭に分ける。
 * 通行できない辺は同じ高さでも境界として残す。
 */
FlatContours BuildFlatContours(const voxelization::WalkableField& i_field,
                               const std::vector<std::size_t>& i_coreSpans,
                               const std::vector<std::uint8_t>& i_eroded,
                               const region::Regions& i_regions);
}
#endif
