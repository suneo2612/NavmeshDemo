/**
 * @file contour_lines.h
 * @brief 領域の輪郭を確認するワールド座標の線分を作成する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEBUG_CONTOUR_LINES_H
#define NAVMESH_DEMO_DEBUG_CONTOUR_LINES_H

#include "bounds_lines.h"
#include "../navmesh.h"

namespace demo::navmesh::debug
{
/**
 * @brief 全タイルの外周と穴を描画用の線分に変換する。
 * @param i_navmesh 構築済みのタイルと地形範囲。
 * @return terrain に外周、tiles に穴と退化輪郭の線分。
 * @throws std::invalid_argument タイルがない場合。
 * @throws std::overflow_error ワールド座標を表現できない場合。
 * @details 線は各辺の歩行面の高さから少し上に描画する。
 */
BoundsLines BuildContourLines(const Navmesh& i_navmesh);
}
#endif
