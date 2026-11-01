/**
 * @file tile_field.h
 * @brief 共通格子の歩行面をタイルごとに参照する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_TILE_FIELD_H
#define NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_TILE_FIELD_H

#include "distance_field.h"

namespace demo::navmesh::voxelization
{
/**
 * @brief 一つのタイルが所有するセルと歩行面。
 * @details 添字はそれぞれ共通格子の行優先セルと TiledWalkableField::field.spans を参照する。
 */
struct TileField
{
    Bounds bounds;                        // 本来のタイル空間の範囲。
    std::vector<std::size_t> cells;       // このタイルに属する共通格子のセル。
    std::vector<std::size_t> spans;       // このタイルに属する歩行面。
};

/**
 * @brief 接続と距離場を共有し、歩行面をタイルごとに管理する。
 */
struct TiledWalkableField
{
    WalkableField field;            // タイル間を含む接続を保持する共通格子。
    DistanceField distance;         // 共通格子全体の距離場。
    std::vector<TileField> tiles;   // Z 行・X 列順のタイル。
};

/**
 * @brief 共通格子の接続を維持したまま歩行面をタイルに振り分ける。
 * @param i_heightfield 歩行面を作成した高さ場。
 * @param i_field 高さ場から作成した歩行面。結果にコピーされる。
 * @param i_grid 高さ場と同じ地形範囲のタイル格子。
 * @return 距離場とタイル別の参照を持つ共通格子。
 * @throws std::invalid_argument 範囲、格子、歩行面の構成が不正な場合。
 * @throws std::length_error 距離が表現できる範囲を超えた場合。
 * @details ボクセルとタイルの境界は一致しなくてもよい。セルの地形範囲内の中心が
 * 属するタイルを所有者とするため、セルがタイルの範囲をはみ出すことがある。
 * タイル間接続と距離は共通格子全体で計算される。タイル単独の再構築ではない。
 */
TiledWalkableField PartitionWalkableField(const Heightfield& i_heightfield,
                                          const WalkableField& i_field,
                                          const TileGrid& i_grid);
}
#endif
