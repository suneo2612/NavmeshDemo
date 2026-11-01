/**
 * @file tile_regions.h
 * @brief 地形を分割した各タイルが担当するセル範囲を定める。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#pragma once
#ifndef NAVMESH_DEMO_VOXELIZATION_TILES_TILE_REGIONS_H
#define NAVMESH_DEMO_VOXELIZATION_TILES_TILE_REGIONS_H

#include "../heightfield.h"
#include <optional>

namespace demo::navmesh::voxelization
{
/**
 * @brief 地形を区切ったセルの XZ 範囲。開始位置は含み、終端位置は含まない。
 */
struct CellRect
{
    std::size_t beginX = 0; // 範囲に含める最初の X セル。
    std::size_t beginZ = 0; // 範囲に含める最初の Z セル。
    std::size_t endX = 0;   // 範囲に含めない X 側の終端。
    std::size_t endZ = 0;   // 範囲に含めない Z 側の終端。
};

/**
 * @brief 一つのタイルが出力する範囲と、計算に使う周辺範囲。
 */
struct TileRegion
{
    TileCoordinate coordinate; // 格子の中でのタイル位置。
    Bounds bounds;             // ワールド座標で表したタイルの範囲。
    CellRect core;             // セル中心の位置を基準に、このタイルが出力を担当する範囲。
    CellRect managed;          // タイルの範囲と周囲の余白を含め、距離を計算する範囲。
    CellRect work;             // 隣とのつながりを調べるため、さらに外側まで含めた範囲。
};

/**
 * @brief セル範囲を地形の端からはみ出さないように広げる。
 * @param i_rect 広げる範囲。
 * @param i_cells 各辺に追加するセル数。
 * @param i_width 地形全体の X 方向のセル数。
 * @param i_depth 地形全体の Z 方向のセル数。
 * @return 地形全体の範囲内に収まる拡張後の範囲。
 */
CellRect ExpandCellRect(const CellRect& i_rect, std::size_t i_cells,
                        std::size_t i_width, std::size_t i_depth);

/**
 * @brief キャラクターが境界で不自然に切れないための最小余白を求める。
 * @param i_cellSize 地面を区切るセルの一辺の長さ。
 * @param i_agentRadius キャラクターの半径。
 * @return 距離判定と隣接確認に必要な最小セル数。
 * @throws std::invalid_argument セル幅または半径が有限の正数でない場合。
 * @throws std::length_error 必要な余白が許容上限を超える場合。
 */
std::size_t MinimumTileBorder(float i_cellSize, float i_agentRadius);

/**
 * @brief 地形の各セルを一つのタイルに割り当て、計算に使う周辺範囲を求める。
 * @param i_field 地形をセルに分けたデータ。
 * @param i_grid 地形と同じ範囲を分割するタイル格子。
 * @param i_borderCells タイルの外側に含める余白のセル数。
 * @param i_workCells 余白の外側に追加する、隣との接続確認用セル数。
 * @return 行ごとに並べた各タイルの出力範囲と計算範囲。
 * @throws std::invalid_argument 地形とタイル格子の範囲が異なる場合、またはセルを担当しないタイルがある場合。
 * @details 各セルの中心が入るタイルを出力担当とし、タイル境界に触れるセルも計算範囲に含める。
 */
std::vector<TileRegion> BuildTileRegions(const Heightfield& i_field, const TileGrid& i_grid,
                                         std::size_t i_borderCells, std::size_t i_workCells = 1);
}
#endif
