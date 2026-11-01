/**
 * @file polygon_mesh.h
 * @brief 歩行面を覆う三角形と凸多角形のデータを定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_POLYGON_POLYGON_MESH_H
#define NAVMESH_DEMO_POLYGON_POLYGON_MESH_H

#include "../region/region.h"
#include "../voxelization/walkable_field.h"
#include "../voxelization/tiles/tile_regions.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace demo::navmesh::polygon
{
struct ContourTriangulation;
inline constexpr auto kNoPolygon = std::numeric_limits<std::uint32_t>::max();
inline constexpr auto kMaxConvexVertices = std::size_t{6};

/**
 * @brief 格子座標を地形端で切り詰めた実距離に変換する寸法。
 */
struct GridMetric
{
    double cellSize;        // XZ 方向のセル幅。
    double extentX;         // 地形の X 方向の幅。
    double extentZ;         // 地形の Z 方向の幅。
    std::size_t beginX;     // 共通格子でのタイル作業範囲の X 開始位置。
    std::size_t beginZ;     // 共通格子でのタイル作業範囲の Z 開始位置。
};

/**
 * @brief タイル作業範囲の格子座標で表した頂点。
 * @details 小数部分は該当セル内の位置を表す。地形端の短いセル内でも 0～1 で補間する。
 */
struct Vertex
{
    double x;            // 作業範囲の左端からの X 座標。
    double z;            // 作業範囲の奥端からの Z 座標。
    std::uint32_t floor; // 地形の最低 Y からの高さセル数。
};

/**
 * @brief 一つの凸多角形を構成する三角形。
 */
struct Triangle
{
    std::array<std::uint32_t, 3> vertices; // 頂点配列の添字。上向きの巻き順。
    std::uint32_t polygon;                 // 所属する凸多角形の添字。
    std::array<bool, 3> boundary{};        // 各辺が領域の輪郭なら true。
};

/**
 * @brief 同じ高さの連結する歩行面をまとめた凸多角形。
 */
struct Polygon
{
    std::vector<std::uint32_t> vertices;   // 頂点配列の添字。上向きの巻き順。
    std::uint32_t region;                  // タイル内の領域 ID。
};

/**
 * @brief タイル所有面の三角形と凸多角形。
 * @details 一つの歩行面が複数のポリゴンに分割される場合がある。
 * 格子頂点は異なる高さや連結領域の間で重複する場合がある。
 */
struct PolygonMesh
{
    std::vector<Vertex> vertices;              // 作業範囲を原点とする格子頂点。
    std::vector<Triangle> triangles;           // 凸多角形を構成する上向き三角形。
    std::vector<Polygon> polygons;             // 領域ごとの凸多角形。
    std::vector<std::vector<std::uint32_t>> spanPolygons; // 面に面積をもって重なるポリゴン。
};

/**
 * @brief 格子上の三角形とセルが正の面積で重なるか判定する。
 * @param i_triangle 時計回りまたは反時計回りの三角形。
 * @param i_x セルの X 座標。
 * @param i_z セルの Z 座標。
 * @return 面積をもって重なる場合は true。辺や点だけの接触は false。
 * @throws std::invalid_argument 三角形が退化している場合。
 * @throws std::length_error 座標が安全に扱える範囲を超える場合。
 * @throws std::overflow_error 小数座標の向きを正確に判定できない場合。
 */
bool TriangleOverlapsCell(const std::array<Vertex, 3>& i_triangle, std::size_t i_x, std::size_t i_z);

/**
 * @brief 三点の XZ 座標から向きを正確に判定する。
 * @param i_a 最初の頂点。
 * @param i_b 二番目の頂点。
 * @param i_cx 三番目の頂点の X 座標。
 * @param i_cz 三番目の頂点の Z 座標。
 * @return 反時計回りなら 1、時計回りなら -1、同一直線上なら 0。
 * @throws std::overflow_error 値または正確な演算結果を表現できない場合。
 */
int ExactOrientation(const Vertex& i_a, const Vertex& i_b, double i_cx, double i_cz);

/**
 * @brief 小数を含む共通格子の座標を地形の端で切り詰めた距離へ変換する。
 * @param i_coordinate 共通格子の座標。
 * @param i_cellSize セルの幅。
 * @param i_extent 地形の幅。
 * @return 地形の最小座標からの距離。
 * @throws std::invalid_argument 座標または寸法が不正な場合。
 * @throws std::overflow_error 距離を表現できない場合。
 */
double GridCoordinate(double i_coordinate, double i_cellSize, double i_extent);

/**
 * @brief 構築済みの制約付き三角形を凸多角形に結合し、歩行面に対応付ける。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @param i_triangulation 対象の平坦な領域を重複なく覆う三角形。
 * @param i_metric 地形端を含む格子の実距離。
 * @return 凸多角形と、それを覆う三角形。
 * @throws std::invalid_argument 入力や三角形の接続が不正な場合。
 * @throws std::length_error 頂点またはポリゴンの数が上限を超える場合。
 * @throws std::overflow_error 座標や辺の長さを表現できない場合。
 * @details 呼び出し側は、三角形の重複がなく境界を越えていないことを保証する。
 */
PolygonMesh BuildPolygonMesh(const voxelization::WalkableField& i_field,
                             const std::vector<std::size_t>& i_coreSpans,
                             const std::vector<std::uint8_t>& i_eroded,
                             const region::Regions& i_regions, ContourTriangulation i_triangulation,
                             const GridMetric& i_metric);

/**
 * @brief 所有面を穴を埋めずに分割し、三角形と凸多角形を作る。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @param i_metric 地形端を含む格子の実距離。
 * @return 凸多角形と、それを覆う三角形。
 * @throws std::invalid_argument 入力や領域が不正な場合。
 * @throws std::length_error 頂点またはポリゴンの数が上限を超える場合。
 * @throws std::overflow_error 座標や辺の長さを表現できない場合。
 * @details 同じ領域・高さで四方向に接続する面の輪郭から三角形を作る。
 * 輪郭の穴、接点、通行できない共通辺を越えて結合しない。
 * 高さの異なる面は結合せず、段差は別のポリゴンとなる。
 * ポリゴン間の接続はタイル全体の構築後に別途生成する。
 */
PolygonMesh BuildPolygonMesh(const voxelization::WalkableField& i_field,
                             const std::vector<std::size_t>& i_coreSpans,
                             const std::vector<std::uint8_t>& i_eroded,
                             const region::Regions& i_regions, const GridMetric& i_metric);

/**
 * @brief 単位格子を使って所有面から凸多角形を生成する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 凸多角形と、それを覆う三角形。
 */
PolygonMesh BuildPolygonMesh(const voxelization::WalkableField& i_field,
                             const std::vector<std::size_t>& i_coreSpans,
                             const std::vector<std::uint8_t>& i_eroded,
                             const region::Regions& i_regions);
}
#endif
