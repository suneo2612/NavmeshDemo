/**
 * @file polygon_triangles.cpp
 * @brief 凸多角形の三角形を描画用頂点へ展開する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "polygon_triangles.h"
#include "grid_position.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kHeightOffset = 0.02;
constexpr auto kEdgeHeightOffset = 0.03;
constexpr auto kMaxVertices = std::size_t{6000000};
constexpr auto kTriangleVertices = std::size_t{3};
constexpr auto kLinkHeightOffset = 0.06;
}

/**
 * @brief 全タイルの三角形をワールド座標の頂点配列に展開する。
 * @param i_navmesh 構築済みの地形とタイル。
 * @return 三頂点ごとに上向きの三角形。
 * @throws std::invalid_argument タイルがない場合。
 * @throws std::length_error 描画頂点数が上限を超える場合。
 * @throws std::overflow_error ワールド座標を表現できない場合。
 * @details 歩行面より少し上に描画し、端の短いセルを地形範囲に合わせる。
 */
std::vector<std::array<float, 3>> BuildPolygonTriangles(const Navmesh& i_navmesh)
{
    const auto* tiles = i_navmesh.VoxelTiles();
    if (!tiles) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    auto result = std::vector<std::array<float, 3>>{};
    // 描画上限を確認しながら、三角形の各頂点をタイル位置と床高さからワールド座標へ変換する。
    for (const auto& tile : tiles->Tiles())
    {
        const auto& mesh = tile.polygonMesh;
        if (mesh.triangles.size() > (kMaxVertices - result.size()) / kTriangleVertices)
        {
            throw std::length_error("表示するポリゴンの頂点数が上限を超えました。");
        }
        for (const auto& triangle : mesh.triangles)
        {
            // 頂点順を保って描画位置を求め、座標変換後も上向きの面になることを検証する。
            auto points = std::array<std::array<float, 3>, 3>{};
            for (auto corner = std::size_t{}; corner < points.size(); ++corner)
            {
                const auto& vertex = mesh.vertices.at(triangle.vertices[corner]);
                points[corner] = GridPosition(i_navmesh, tile.region.work.beginX + vertex.x,
                    tile.region.work.beginZ + vertex.z, vertex.floor, kHeightOffset);
            }
            const auto dx1 = static_cast<double>(points[1][0]) - points[0][0];
            const auto dz1 = static_cast<double>(points[1][2]) - points[0][2];
            const auto dx2 = static_cast<double>(points[2][0]) - points[0][0];
            const auto dz2 = static_cast<double>(points[2][2]) - points[0][2];
            if (dz1 * dx2 - dx1 * dz2 <= 0)
            {
                throw std::overflow_error("ポリゴンを描画座標で表現できません。");
            }
            result.insert(result.end(), points.begin(), points.end());
        }
    }
    return result;
}

/**
 * @brief 全タイルの凸多角形の境界辺をワールド座標の線分に変換する。
 * @param i_navmesh 構築済みの地形とタイル。
 * @return terrain に各ポリゴンの境界辺。
 * @throws std::invalid_argument タイルがない場合。
 * @throws std::length_error 描画頂点数が上限を超える場合。
 * @throws std::overflow_error ワールド座標を表現できない場合。
 */
BoundsLines BuildPolygonLines(const Navmesh& i_navmesh)
{
    const auto* tiles = i_navmesh.VoxelTiles();
    if (!tiles) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    auto result = BoundsLines{};
    // 各凸多角形の頂点数と出力上限を確認し、隣り合う頂点および末尾から先頭への辺を作る。
    for (const auto& tile : tiles->Tiles())
    {
        const auto& mesh = tile.polygonMesh;
        for (const auto& polygon : mesh.polygons)
        {
            if (polygon.vertices.size() < 3 || polygon.vertices.size() > polygon::kMaxConvexVertices)
            {
                throw std::invalid_argument("表示するポリゴンの頂点数が不正です。");
            }
            if (polygon.vertices.size() > (kMaxVertices - result.terrain.size()) / 2)
            {
                throw std::length_error("表示するポリゴン辺の頂点数が上限を超えました。");
            }
            auto corners = std::vector<std::array<float, 3>>(polygon.vertices.size());
            for (auto index = std::size_t{}; index < corners.size(); ++index)
            {
                const auto& vertex = mesh.vertices.at(polygon.vertices[index]);
                corners[index] = GridPosition(i_navmesh, tile.region.work.beginX + vertex.x,
                    tile.region.work.beginZ + vertex.z, vertex.floor, kEdgeHeightOffset);
            }
            for (auto index = std::size_t{}; index < corners.size(); ++index)
            {
                if (corners[index] == corners[(index + 1) % corners.size()])
                {
                    throw std::overflow_error("ポリゴン辺を描画座標で表現できません。");
                }
                result.terrain.push_back(corners[index]);
                result.terrain.push_back(corners[(index + 1) % corners.size()]);
            }
        }
    }
    return result;
}

/**
 * @brief 接続された凸ポリゴンの中心同士を１本の線で結ぶ。
 * @param i_navmesh リンクを持つ構築結果。
 * @return links にポリゴンの組ごとの線分。
 * @throws std::invalid_argument 接続の参照が不正な場合。
 * @throws std::length_error 頂点数が上限を超える場合。
 * @throws std::overflow_error 描画座標を表現できない場合。
 */
BoundsLines BuildPolygonLinkLines(const Navmesh& i_navmesh)
{
    const auto* tiles = i_navmesh.VoxelTiles();
    if (!tiles) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    auto result = BoundsLines{};
    const auto& graph = tiles->Links();
    if (graph.links.size() > kMaxVertices / 2) { throw std::length_error("表示する接続線が上限を超えました。"); }
    result.links.reserve(graph.links.size() * 2);
    // 各リンクの両側のポリゴン中心を求め、接続関係を視覚的に確認できる線分にする。
    for (const auto& link : graph.links)
    {
        for (const auto& ref : link.polygons)
        {
            const auto& tile = tiles->Tiles().at(ref.tile);
            const auto& mesh = tile.polygonMesh;
            const auto& poly = mesh.polygons.at(ref.polygon);
            if (poly.vertices.size() < 3) { throw std::invalid_argument("接続先のポリゴンが不正です。"); }
            auto sum = std::array<double, 3>{};
            for (const auto vertexIndex : poly.vertices)
            {
                const auto& vertex = mesh.vertices.at(vertexIndex);
                const auto point = GridPosition(i_navmesh, tile.region.work.beginX + vertex.x,
                    tile.region.work.beginZ + vertex.z, vertex.floor, kLinkHeightOffset);
                for (auto axis = std::size_t{}; axis < 3; ++axis) { sum[axis] += point[axis]; }
            }
            auto center = std::array<float, 3>{};
            for (auto axis = std::size_t{}; axis < 3; ++axis)
            {
                const auto value = sum[axis] / static_cast<double>(poly.vertices.size());
                if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
                {
                    throw std::overflow_error("接続線の中心を表現できません。");
                }
                center[axis] = static_cast<float>(value);
            }
            result.links.push_back(center);
        }
    }
    return result;
}
}
