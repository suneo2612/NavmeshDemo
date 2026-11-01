/**
 * @file convex_merge.h
 * @brief 三角形を共有辺に沿って凸多角形に結合する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_POLYGON_CONVEX_MERGE_H
#define NAVMESH_DEMO_POLYGON_CONVEX_MERGE_H

#include "contour_triangulation.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace demo::navmesh::polygon
{
/**
 * @brief 三角形を結合した一つの凸多角形。
 */
struct MergedPolygon
{
    std::vector<std::uint32_t> vertices;  // 反時計回りに見える上向きの角。
    std::vector<std::size_t> triangles;   // 元の三角形の添字。
    std::uint32_t patch;                  // 同じ高さの連結領域の ID。
};

/**
 * @brief 凸多角形と三角形からの対応。
 */
struct ConvexMerge
{
    std::vector<MergedPolygon> polygons;       // 結合後の凸多角形。
    std::vector<std::uint32_t> trianglePolygons; // 元の三角形ごとのポリゴン添字。
};

/**
 * @brief 長い共有辺を優先し、凸性と頂点数上限を守って結合する。
 * @param i_mesh 共通頂点と境界制約を持つ三角形。
 * @param i_metric 地形端を含む実距離の換算情報。
 * @return 凸多角形と三角形の対応。
 * @throws std::invalid_argument 三角形または寸法が不正な場合。
 * @throws std::length_error 頂点またはポリゴン数が上限を超える場合。
 * @throws std::overflow_error 凸性や辺の長さを表現できない場合。
 * @details 境界制約のある辺は結合しない。すべての共有辺は端点が一致する必要がある。
 */
ConvexMerge MergeContourTriangles(const ContourTriangulation& i_mesh, const GridMetric& i_metric);
}
#endif
