/**
 * @file terrain_input.h
 * @brief ステージメッシュを Navmesh 構築用の地形に接続する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_TERRAIN_TERRAIN_INPUT_H
#define NAVMESH_DEMO_DEMO_NAVMESH_TERRAIN_TERRAIN_INPUT_H

#include "terrain.h"
#include <optional>

namespace demo::navmesh::input
{
/**
 * @brief 地形の元メッシュと変換済み三角形を保持する入力窓口。
 * @details 設定や変換の更新が失敗した場合、直前の地形を保持する。
 */
class TerrainInput final
{
public:
    /**
     * @brief ステージの三角形を受け取り、描画と同じ変換を適用する。
     * @param i_model ステージの CPU モデル。
     * @param i_world フィールド Entity のワールド変換。
     * @throws std::invalid_argument 地形の入力が不正な場合。
     * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
     */
    void SetTerrain(const loader::ModelData& i_model, const general::math::Matrix4& i_world);

    /**
     * @brief 保持している元メッシュからワールド座標を再計算する。
     * @param i_world フィールド Entity の新しいワールド変換。
     * @throws std::logic_error 地形が設定されていない場合。
     * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
     */
    void SetWorldTransform(const general::math::Matrix4& i_world);

    /**
     * @brief 保持している地形を破棄する。
     */
    void Clear() noexcept;

    /**
     * @brief 現在の地形を参照する。
     * @return 地形が設定されていない場合は nullptr。
     * @details ポインターは地形の設定、変換更新または破棄まで有効。
     */
    const Terrain* GetTerrain() const noexcept;

private:
    loader::ModelData      m_stSource;  // 位置、インデックス、ノード変換のみを保持する元メッシュ。
    std::optional<Terrain> m_instTerrain; // ワールド空間に変換した地形。
};
}
#endif
