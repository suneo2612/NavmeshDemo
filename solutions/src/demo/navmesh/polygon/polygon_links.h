/**
 * @file polygon_links.h
 * @brief ポリゴン間の歩行可能な接続を保持する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_POLYGON_POLYGON_LINKS_H
#define NAVMESH_DEMO_POLYGON_POLYGON_LINKS_H

#include "polygon_mesh.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace demo::navmesh::voxelization { struct VoxelTile; }

namespace demo::navmesh::polygon
{
/**
 * @brief タイルとポリゴンの添字。
 */
struct PolygonRef
{
    std::size_t tile;      // 行優先タイル配列の添字。
    std::uint32_t polygon; // タイル内のポリゴン添字。
    bool operator==(const PolygonRef& i_other) const = default;
};

/**
 * @brief 接続する凸ポリゴンの辺と、通行可能な共有区間。
 */
struct PolygonPortal
{
    std::array<std::size_t, 2> edges;    // 各ポリゴンの接続辺の添字。
    std::array<double, 2> begin;        // 共通格子における XZ 始点。
    std::array<double, 2> end;          // 共通格子における XZ 終点。
    bool operator==(const PolygonPortal& i_other) const = default;
};

/**
 * @brief 一組の凸ポリゴン間の双方向接続と通行可能な共有辺。
 */
struct PolygonLink
{
    std::array<PolygonRef, 2> polygons; // 接続する二つのポリゴン。
    std::vector<PolygonPortal> portals;  // それぞれの辺における通行可能な区間。
    bool operator==(const PolygonLink& i_other) const = default;
};

/**
 * @brief ポリゴンごとのリンク参照を含む接続全体。
 */
struct PolygonLinkGraph
{
    std::vector<PolygonLink> links; // 接続を重複なく格納する配列。
    std::vector<std::vector<std::vector<std::size_t>>> adjacency; // タイル・ポリゴンごとのリンク添字。
    bool operator==(const PolygonLinkGraph& i_other) const = default;
};

/**
 * @brief 再構築中のタイルとポリゴンの組み合わせ。
 */
struct PolygonLinkTile
{
    const voxelization::VoxelTile* tile; // 歩行面とタイル範囲。
    const PolygonMesh* mesh;            // 新しいポリゴン。
    const region::Regions* regions;     // 新しい領域。
};

/**
 * @brief 凸ポリゴン間の通行可能な共有辺を集約し、組ごとに双方向リンクを構築する。
 * @param i_tiles 行優先のタイルと、対応するポリゴン・領域。
 * @return 接続一覧と各ポリゴンからの参照。
 * @throws std::invalid_argument 入力に不整合がある場合。
 * @throws std::length_error 接続数が上限を超える場合。
 */
PolygonLinkGraph BuildPolygonLinks(const std::vector<PolygonLinkTile>& i_tiles);
}
#endif
