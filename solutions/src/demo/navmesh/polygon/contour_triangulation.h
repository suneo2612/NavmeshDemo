/**
 * @file contour_triangulation.h
 * @brief 平坦な輪郭を境界制約付きの三角形に分割する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_POLYGON_CONTOUR_TRIANGULATION_H
#define NAVMESH_DEMO_POLYGON_CONTOUR_TRIANGULATION_H

#include "../contour/contour.h"
#include "polygon_mesh.h"
#include <array>
#include <cstdint>
#include <vector>

namespace demo::navmesh::polygon
{
/**
 * @brief 境界制約を持つ上向きの三角形。
 */
struct ContourTriangle
{
    std::array<std::uint32_t, 3> vertices; // 頂点配列の添字。
    std::array<bool, 3> boundary;          // 各頂点から次の頂点への辺が境界なら true。
    std::uint32_t patch;                   // 同一高さの連結領域の ID。
};

/**
 * @brief 平坦な輪郭とそれを覆う制約付き三角形。
 */
struct ContourTriangulation
{
    contour::FlatContours flat;              // 三角形の元となる連結面と輪郭。
    std::vector<Vertex> vertices;             // タイル作業範囲の格子頂点。
    std::vector<ContourTriangle> triangles;   // 境界を越えない上向き三角形。
};

/**
 * @brief 外周と穴の選択済み頂点から一つの平坦な領域を三角形に分割する。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_flat 平坦な領域と選択済みの輪郭点。
 * @param i_patch 分割する平坦な領域の識別子。
 * @return 対象領域の三角形。ほかの領域の三角形は含まない。
 * @throws std::invalid_argument 輪郭の連結または分割に失敗した場合。
 * @throws std::length_error 頂点数または座標が上限を超える場合。
 */
ContourTriangulation TriangulateSelectedPatch(const voxelization::WalkableField& i_field,
                                               const contour::FlatContours& i_flat,
                                               std::uint32_t i_patch);

/**
 * @brief 地形端の短いセルを考慮して、選択済み輪郭を三角形に分割する。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_flat 平坦な領域と選択済みの輪郭点。
 * @param i_patch 分割する平坦な領域の識別子。
 * @param i_metric 地形端を含むセル寸法。
 * @return 対象領域の三角形。
 */
ContourTriangulation TriangulateSelectedPatch(const voxelization::WalkableField& i_field,
                                               const contour::FlatContours& i_flat,
                                               std::uint32_t i_patch, const GridMetric& i_metric);

/**
 * @brief 同じ高さの輪郭から境界を保持する三角形を生成する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 平坦な輪郭と三角形。
 * @throws std::invalid_argument 入力や輪郭が不正な場合。
 * @throws std::length_error 頂点または三角形が上限を超える場合。
 * @details 直線上の不要点を省いた輪郭頂点から三角形を作り、穴や通行不能辺を境界として残す。
 * 単位格子を使用する。地形端が短い場合はセル寸法を受け取るオーバーロードを使用する。
 */
ContourTriangulation TriangulateContours(const voxelization::WalkableField& i_field,
                                         const std::vector<std::size_t>& i_coreSpans,
                                         const std::vector<std::uint8_t>& i_eroded,
                                         const region::Regions& i_regions);

/**
 * @brief 地形端の短いセルを考慮して、全平坦領域の輪郭を三角形に分割する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @param i_metric 地形端を含むセル寸法。
 * @return 平坦な輪郭と三角形。
 */
ContourTriangulation TriangulateContours(const voxelization::WalkableField& i_field,
                                         const std::vector<std::size_t>& i_coreSpans,
                                         const std::vector<std::uint8_t>& i_eroded,
                                         const region::Regions& i_regions, const GridMetric& i_metric);

}
#endif
