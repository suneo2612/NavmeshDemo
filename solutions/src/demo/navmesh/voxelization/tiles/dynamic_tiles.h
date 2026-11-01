/**
 * @file dynamic_tiles.h
 * @brief 箱型障害物を追加・削除したとき、影響する地形タイルだけを更新する。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#pragma once
#ifndef NAVMESH_DEMO_VOXELIZATION_TILES_DYNAMIC_TILES_H
#define NAVMESH_DEMO_VOXELIZATION_TILES_DYNAMIC_TILES_H

#include "../distance/tile_distance.h"
#include "../obstacles/box_obstacle.h"
#include "../../region/region.h"
#include "../../contour/contour.h"
#include "../../polygon/polygon_mesh.h"
#include "../../polygon/polygon_links.h"
#include <memory>
#include <optional>

namespace demo::navmesh::voxelization
{
/**
 * @brief タイル作成に用いるキャラクター寸法と境界の余白。
 */
struct TileBuildSettings
{
    WalkableSettings walkable;               // 通行に必要な頭上の高さと、上り下りできる段差。
    float agentRadius = 0.5F;                // 通行するキャラクターの半径。
    std::optional<std::size_t> borderCells;   // タイル境界に加える余白。未指定なら必要数を自動計算。
    region::PartitionMethod partitionMethod = region::PartitionMethod::Monotone; // 歩行できる面をまとめる方法。
};

/**
 * @brief 一つのタイルの通行可能な面と、そこから作成した地形データ。
 * @details walkable の各面の XZ 位置は region.work の左上を基準とするセル位置で、
 * 面同士の接続先も walkable.spans 内の添字で示す。sourceIndex は
 * solidColumns の同じセル内にある高さ区間を示す。managed の範囲外の面は、
 * 範囲の端で隣の面とつながるかを確認するために保持する。
 * eroded の値は erosionArea 内だけで判定済みであり、1 は通行不可と判定された面を示す。
 * walkable はその判定前の面、retained は判定後に残った面を表す。
 */
struct VoxelTile
{
    TileRegion region;                     // 出力する範囲と、計算に使う周辺範囲。
    std::vector<std::vector<SolidSpan>> solidColumns; // 計算範囲内の各セルにある高さ区間。
    WalkableField walkable;                 // 計算範囲内の通行可能な面と面同士の接続。
    std::vector<std::uint8_t> managed;      // 各面が距離計算の対象範囲に含まれるか。
    std::vector<std::size_t> coreSpans;     // 出力を担当する範囲内にある面の添字。
    TileDistance distance;                  // 通行不可と判定した面を除いた後の距離情報。
    std::vector<std::uint8_t> eroded;       // 1 は通行不可の判定。範囲外の面は未判定。
    TileDistance terrainDistance;           // 判定前の距離情報。地形の確認表示に使用。
    WalkableField retained;                 // 通行不可と判定した面を除いた通行可能な面。
    region::Regions regions;                // 出力担当の面を分割した領域。
    CellRect erosionArea;                   // キャラクター寸法による通行可否を判定した範囲。
    std::vector<contour::Contour> contours; // 各領域の外周と穴の輪郭。
    polygon::PolygonMesh polygonMesh;       // 出力担当の面を表す三角形と多角形。
};

/**
 * @brief 障害物と、地形をセルに分けて作成したタイル結果を管理する。
 * @details 更新に失敗した場合は、障害物・タイル・タイル間の接続を変更しない。
 * タイル結果への参照は、そのタイルの再作成または本オブジェクトの破棄まで有効。
 */
class DynamicVoxelTiles final
{
public:
    /**
     * @brief 地形をセルに分け、指定された障害物を含む全タイルを作成する。
     * @param i_terrain 地形を構成する三角形と地形の範囲。
     * @param i_grid 地形全体を分割するタイル格子。
     * @param i_raster セルの幅など、地形をセルに分ける設定。
     * @param i_settings キャラクター寸法、境界の余白、面のまとめ方。
     * @param i_initialObstacles 最初から反映する障害物。識別子は配列順に割り当てる。
     * @param i_firstObstacleId 初期障害物と以後の追加に使い始める識別子。
     * @throws std::invalid_argument 入力、格子、余白または障害物が不正な場合。
     * @throws std::length_error セル数、計算範囲または識別子が上限を超える場合。
     */
    DynamicVoxelTiles(const Terrain& i_terrain, const TileGrid& i_grid,
                      const RasterSettings& i_raster, const TileBuildSettings& i_settings,
                      const std::vector<Bounds>& i_initialObstacles = {},
                      std::uint64_t i_firstObstacleId = 1);

    /**
     * @brief 現在の障害物を保ったまま、新しい地形と格子で全タイルを作り直す。
     * @param i_terrain 作り直し後の地形。
     * @param i_grid 新しい地形全体を分割するタイル格子。
     * @return 作り直した結果。次に使う障害物識別子も引き継ぐ。
     * @details 障害物の位置はワールド座標のまま保つ。失敗時は現在の結果を変更しない。
     */
    std::unique_ptr<DynamicVoxelTiles> Rebuild(const Terrain& i_terrain, const TileGrid& i_grid) const;

    /**
     * @brief 障害物の位置と識別子を保ち、新しい構築設定ですべてのタイルを作り直す。
     * @param i_terrain 再構築する地形。
     * @param i_grid 新しいタイル格子。
     * @param i_raster 新しいセル設定。
     * @param i_settings 新しい歩行条件と境界設定。
     * @return 再構築したタイル。失敗時は現在の結果を変更しない。
     */
    std::unique_ptr<DynamicVoxelTiles> Rebuild(const Terrain& i_terrain, const TileGrid& i_grid,
                                              const RasterSettings& i_raster,
                                              const TileBuildSettings& i_settings) const;

    /**
     * @brief 通行可能な面をまとめる方法を変え、全タイルの形状データを作り直す。
     * @param i_method 使用するまとめ方。
     * @throws std::invalid_argument 指定方法または距離情報が不正な場合。
     * @throws std::length_error 作成する領域数が上限を超える場合。
     * @details 失敗時は現在の設定と全タイルを保つ。障害物と識別子は変更しない。
     */
    void SetPartitionMethod(region::PartitionMethod i_method);

    /**
     * @brief 箱型障害物を追加し、その範囲にかかるタイルを更新する。
     * @param i_bounds 障害物のワールド座標での範囲。
     * @return この障害物を後で個別に削除するための識別子。
     * @throws std::invalid_argument 範囲に有限でない値がある、または幅がない場合。
     * @throws std::length_error 識別子が上限に達した場合。
     */
    std::uint64_t AddObstacle(const Bounds& i_bounds);

    /**
     * @brief 識別子で障害物を一つ削除し、その範囲にかかるタイルを更新する。
     * @param i_id AddObstacle が返した識別子。
     * @throws std::invalid_argument 指定された識別子の障害物がない場合。
     */
    void RemoveObstacle(std::uint64_t i_id);

    /**
     * @brief 全障害物を削除し、影響を受けていたタイルを再構築する。
     */
    void ClearObstacles();

    /**
     * @brief 行ごとに並べたタイル結果を参照する。
     * @return タイル結果の配列。内容は次のタイル更新まで有効。
     */
    const std::vector<VoxelTile>& Tiles() const noexcept { return m_instTiles; }

    /**
     * @brief 隣り合うタイルの多角形どうしの接続情報を参照する。
     * @return 接続の一覧と、各タイル・多角形から一覧を引くための添字。
     * @details タイルまたは面のまとめ方を更新した後は、以前の参照を使用できない。
     */
    const polygon::PolygonLinkGraph& Links() const noexcept { return m_stLinks; }

    /**
     * @brief 現在反映されている障害物を参照する。
     * @return 追加した順の障害物一覧。
     */
    const std::vector<BoxObstacle>& Obstacles() const noexcept { return m_instObstacles; }

    /**
     * @brief キャラクター寸法から求めた必要最小の境界余白を返す。
     * @return XZ 方向それぞれに必要なセル数。
     */
    std::size_t MinimumBorder() const noexcept { return m_instMinimumBorder; }

    /**
     * @brief タイル作成に実際に使っている境界余白を返す。
     * @return XZ 方向それぞれに使うセル数。
     */
    std::size_t Border() const noexcept { return m_instBorder; }

private:
    /**
     * @brief 更新対象をすべて作成してから、障害物・タイル・接続情報を切り替える。
     * @param i_affected 更新が必要か調べる障害物の範囲。
     * @param io_obstacles 更新後に保持する障害物一覧。成功時に以前の一覧と交換する。
     */
    void ReplaceObstacles(const std::vector<Bounds>& i_affected,
                          std::vector<BoxObstacle>& io_obstacles);

    Heightfield m_stBase;                    // 障害物を重ねる前の地形データ。
    TileBuildSettings m_stSettings;           // 現在適用している作成設定。
    std::size_t m_instMinimumBorder = 0;      // キャラクター寸法から求めた最小の余白。
    std::size_t m_instBorder = 0;             // 実際に適用している余白。
    std::vector<VoxelTile> m_instTiles;       // 行ごとに並べたタイル結果。
    polygon::PolygonLinkGraph m_stLinks;      // 現在のタイル間の接続情報。
    std::vector<BoxObstacle> m_instObstacles; // 現在反映している障害物。
    std::uint64_t m_instNextId = 1;           // 次に割り当てる障害物識別子。
};
}
#endif
