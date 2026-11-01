/**
 * @file terrain_input.cpp
 * @brief ステージメッシュから Navmesh 用地形を準備する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "terrain_input.h"
#include <stdexcept>
#include <utility>

namespace demo::navmesh::input
{
/**
 * @brief ステージの三角形を受け取り、描画と同じ変換を適用する。
 * @param i_model ステージの CPU モデル。
 * @param i_world フィールド Entity のワールド変換。
 * @throws std::invalid_argument 地形の入力が不正な場合。
 * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
 */
void TerrainInput::SetTerrain(const loader::ModelData& i_model, const general::math::Matrix4& i_world)
{
    // 描画用の画像や頂点属性は除き、変換更新と三角形生成に必要なデータだけを一時保存する。
    auto source = loader::ModelData{};
    source.meshes.reserve(i_model.meshes.size());
    for (const auto& mesh : i_model.meshes)
    {
        // ノード変換、位置、三角形インデックスを複製し、元モデルの他のデータには依存しない。
        auto geometry = loader::MeshData{};
        geometry.node_transform = mesh.node_transform;
        geometry.positions = mesh.positions;
        geometry.indices = mesh.indices;
        source.meshes.push_back(std::move(geometry));
    }

    // 入力を検証して地形を作り終えてから保持データを置き換え、失敗時は以前の地形を維持する。
    auto terrain    =   MakeTerrain(source, i_world);
    m_stSource      =   std::move(source);
    m_instTerrain   =   std::move(terrain);
}

/**
 * @brief 保持している元メッシュからワールド座標を再計算する。
 * @param i_world フィールド Entity の新しいワールド変換。
 * @throws std::logic_error 地形が設定されていない場合。
 * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
 */
void TerrainInput::SetWorldTransform(const general::math::Matrix4& i_world)
{
    // 元メッシュがなければ再計算できないため、未設定状態を呼び出し側に通知する。
    if (!m_instTerrain) { throw std::logic_error("地形が設定されていません。"); }

    // 新しい変換結果を先に作り、成功した場合にだけ保持中の地形を更新する。
    auto terrain    =   MakeTerrain(m_stSource, i_world);
    m_instTerrain   =   std::move(terrain);
}

/**
 * @brief 保持している地形を破棄する。
 */
void TerrainInput::Clear() noexcept
{
    // 元メッシュと変換済み地形の両方を破棄し、未設定状態に戻す。
    m_stSource  =   loader::ModelData{};
    m_instTerrain.reset();
}

/**
 * @brief 現在の地形を参照する。
 * @return 地形が設定されていない場合は nullptr。
 * @details ポインターは地形の設定、変換更新または破棄まで有効。
 */
const Terrain* TerrainInput::GetTerrain() const noexcept
{
    // 地形の有無をそのままポインターで表し、未設定なら nullptr を返す。
    return m_instTerrain ? &*m_instTerrain : nullptr;
}
}
