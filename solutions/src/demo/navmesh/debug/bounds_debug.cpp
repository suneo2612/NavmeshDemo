/**
 * @file bounds_debug.cpp
 * @brief 地形とタイル枠のデバッグ設定・描画を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "bounds_debug.h"
#include <imgui.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kInitialRightOffset = 360.0F;
constexpr auto kDetailsRightOffset = 700.0F;
constexpr auto kInitialTopOffset = 20.0F;
}

/**
 * @brief 表示切り替えと、有効時に独立した地形分割の設定ウィンドウを描画する。
 * @param i_bounds 現在の地形範囲。未選択の場合は nullptr。
 * @param i_extraToggle 同じデバッグ機能ウィンドウに追加する切り替え。
 * @details 有効な ImGui フレームが必要。設定ウィンドウを閉じると枠の表示も無効になる。
 */
void BoundsDebug::DrawControls(const Bounds* i_bounds, const std::function<void()>& i_extraToggle)
{
    // 表示切り替えは常に残し、詳細設定のウィンドウとは独立して操作できるようにする。
    ImGui::SetNextWindowPos(ImVec2(std::max(0.0F, ImGui::GetIO().DisplaySize.x - kInitialRightOffset),
                                   kInitialTopOffset), ImGuiCond_FirstUseEver);
    ImGui::Begin("デバッグ機能");
    ImGui::Checkbox("地形・タイル表示", &m_bVisible);
    if (i_extraToggle) { i_extraToggle(); }
    ImGui::End();

    // 詳細設定はメイン切り替えが有効な間だけ表示し、ウィンドウを閉じる操作も同じ状態へ反映する。
    if (m_bVisible)
    {
        ImGui::SetNextWindowPos(ImVec2(std::max(0.0F, ImGui::GetIO().DisplaySize.x - kDetailsRightOffset),
                                       kInitialTopOffset), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("地形分割デバッグ", &m_bVisible))
        {
            ImGui::Checkbox("地形 AABB", &m_bShowTerrain);
            ImGui::Checkbox("タイル枠", &m_bShowTiles);
            constexpr auto maxTiles = static_cast<int>(kMaxDebugTilesPerAxis);
            ImGui::SliderInt("表示用 X 分割数", &m_iColumns, 1, maxTiles);
            ImGui::SliderInt("表示用 Z 分割数", &m_iRows, 1, maxTiles);
            if (!i_bounds) { ImGui::TextUnformatted("ステージを選択してください。"); }
            if (!m_instError.empty()) { ImGui::TextWrapped("%s", m_instError.c_str()); }
        }
        ImGui::End();
    }
}

/**
 * @brief 設定が有効な場合に現在の地形範囲とタイル枠を描画する。
 * @param i_bounds 現在の地形範囲。未選択の場合は nullptr。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @throws std::runtime_error OpenGL の処理が失敗した場合。
 * @details 有効な OpenGL コンテキストが必要。
 */
void BoundsDebug::Draw(const Bounds* i_bounds, const render::Camera& i_camera, const float i_aspect)
{
    // 非表示または地形未選択のフレームでは、キャッシュも描画資源も変更しない。
    if (!m_bVisible || !i_bounds) { return; }
    const auto& bounds = *i_bounds;
    const auto key = CacheKey{bounds.minimum.x, bounds.minimum.y, bounds.minimum.z,
        bounds.maximum.x, bounds.maximum.y, bounds.maximum.z, m_iColumns, m_iRows, m_bShowTerrain, m_bShowTiles};
    if (!m_instKey || *m_instKey != key)
    {
        // キャッシュキーが変わったときだけ線分を再構築し、設定変更による不要な毎フレーム計算を避ける。
        try
        {
            m_stLines = BuildBoundsLines(bounds, static_cast<std::size_t>(m_iColumns),
                static_cast<std::size_t>(m_iRows), m_bShowTerrain, m_bShowTiles);
            m_instKey = key;
            m_instError.clear();
        }
        catch (const std::invalid_argument& i_error)
        {
            if (m_instError != i_error.what()) { std::cerr << i_error.what() << '\n'; }
            m_instError = i_error.what();
            m_instKey.reset();
        }
    }
    // 線分が現在のキーで正常に作られている場合に限り、遅延生成した描画器で表示する。
    if (m_instKey)
    {
        if (!m_instRenderer) { m_instRenderer = std::make_unique<BoundsRenderer>(); }
        m_instRenderer->Draw(m_stLines, i_camera, i_aspect);
    }
}
}
