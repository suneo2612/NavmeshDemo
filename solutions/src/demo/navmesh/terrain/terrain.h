/**
 * @file terrain.h
 * @brief Navmesh の入力地形と XZ タイル空間を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_TERRAIN_TERRAIN_H
#define NAVMESH_DEMO_DEMO_NAVMESH_TERRAIN_TERRAIN_H

#include "../../../general/math/matrix4.h"
#include "../../../general/model/ModelData.h"
#include <array>
#include <cstddef>
#include <vector>

namespace demo::navmesh
{
/**
 * @brief ワールド空間の軸平行な範囲。
 */
struct Bounds
{
    general::vec::Vector3 minimum; // 範囲の最小座標。
    general::vec::Vector3 maximum; // 範囲の最大座標。
};

/**
 * @brief 変換済み三角形と、その全頂点を含む範囲。
 */
struct Terrain
{
    std::vector<std::array<general::vec::Vector3, 3>> triangles; // 描画と同じ変換を適用した三角形。
    Bounds bounds;                                               // 三角形全体の範囲。
};

/**
 * @brief タイル格子内の位置。
 */
struct TileCoordinate
{
    std::size_t column; // X 方向の添字。
    std::size_t row;    // Z 方向の添字。
};

/**
 * @brief 入力モデルを描画と同じノード・ワールド変換でワールド空間に移す。
 * @param i_model 入力モデル。
 * @param i_world フィールド Entity のワールド変換。
 * @return 三角形とその範囲。
 * @throws std::invalid_argument 三角形がない、インデックスが不正、または入力が非有限の場合。
 * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
 */
Terrain MakeTerrain(const loader::ModelData& i_model, const general::math::Matrix4& i_world);

/**
 * @brief 地形範囲を XZ 方向に等分し、Y 方向は地形全体を含むタイル格子。
 */
class TileGrid final
{
public:
    /**
     * @brief 格子を作り、範囲とタイル数を検証する。
     * @param i_bounds 地形全体の範囲。
     * @param i_columns X 方向のタイル数。
     * @param i_rows Z 方向のタイル数。
     * @throws std::invalid_argument 範囲またはタイル数が不正な場合。
     */
    TileGrid(const Bounds& i_bounds, std::size_t i_columns, std::size_t i_rows);

    /**
     * @brief 指定したタイルの範囲を返す。
     * @param i_column X 方向の添字。
     * @param i_row Z 方向の添字。
     * @return 地形の Y 範囲を含むタイルの範囲。
     * @throws std::out_of_range 添字が格子の範囲外の場合。
     */
    Bounds TileBounds(std::size_t i_column, std::size_t i_row) const;

    /**
     * @brief 指定範囲に重なるタイルを Z 行・X 列の順に返す。
     * @param i_bounds 重なりを調べるワールド空間の範囲。
     * @param i_margin 境界処理を考慮して XZ に広げる距離。
     * @return 範囲と境界を含めて接触するタイルの座標。
     * @throws std::invalid_argument 範囲または余白が不正な場合。
     */
    std::vector<TileCoordinate> OverlappingTiles(const Bounds& i_bounds, float i_margin) const;

    /**
     * @brief X 方向のタイル数を返す。
     * @return X 方向のタイル数。
     */
    std::size_t Columns() const noexcept { return m_instColumns; }

    /**
     * @brief Z 方向のタイル数を返す。
     * @return Z 方向のタイル数。
     */
    std::size_t Rows() const noexcept { return m_instRows; }

private:
    Bounds      m_stBounds;     // 地形の範囲。
    std::size_t m_instColumns;  // X 方向のタイル数。
    std::size_t m_instRows;     // Z 方向のタイル数。
};
}
#endif
