/**
 * @file polygon_triangles.h
 * @brief 凸多角形の三角形を描画用頂点に変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEBUG_POLYGON_TRIANGLES_H
#define NAVMESH_DEMO_DEBUG_POLYGON_TRIANGLES_H

#include "../navmesh.h"
#include "bounds_lines.h"
#include <array>
#include <vector>

namespace demo::navmesh::debug
{
/**
 * @brief 全タイルの三角形をワールド座標の頂点配列に展開する。
 * @param i_navmesh 構築済みの地形とタイル。
 * @return 三頂点ごとに上向きの三角形。
 * @throws std::invalid_argument タイルがない場合。
 * @throws std::length_error 描画頂点数が上限を超える場合。
 * @throws std::overflow_error ワールド座標を表現できない場合。
 * @details 歩行面より少し上に描画し、端の短いセルを地形範囲に合わせる。
 */
std::vector<std::array<float, 3>> BuildPolygonTriangles(const Navmesh& i_navmesh);

/**
 * @brief 全タイルの凸多角形の境界辺をワールド座標の線分に変換する。
 * @param i_navmesh 構築済みの地形とタイル。
 * @return terrain に各ポリゴンの境界辺。
 * @throws std::invalid_argument タイルがない場合。
 * @throws std::length_error 描画頂点数が上限を超える場合。
 * @throws std::overflow_error ワールド座標を表現できない場合。
 */
BoundsLines BuildPolygonLines(const Navmesh& i_navmesh);

/**
 * @brief 接続された凸ポリゴンの中心同士を１本の線で結ぶ。
 * @param i_navmesh リンクを持つ構築結果。
 * @return links にポリゴンの組ごとの線分。
 * @throws std::invalid_argument 接続の参照が不正な場合。
 * @throws std::length_error 頂点数が上限を超える場合。
 * @throws std::overflow_error 描画座標を表現できない場合。
 */
BoundsLines BuildPolygonLinkLines(const Navmesh& i_navmesh);
}
#endif
