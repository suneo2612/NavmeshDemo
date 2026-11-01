/**
 * @file generation_debug.cpp
 * @brief Navmesh の生成設定と更新処理の計測画面を実装する。
 * @date 2026-11-01
 * @par 最終更新日 2026-11-01
 */
#include "generation_debug.h"
#include <imgui.h>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
/**
 * @brief 計測済みの操作は時間と成否を表示し、未実行は区別する。
 * @param i_label 操作名。
 * @param i_timing 操作の直近の計測結果。
 */
void DrawTiming(const char* i_label, const std::optional<UpdateTiming>& i_timing)
{
    if (i_timing)
    {
        ImGui::Text("%s: %.3f ms (%s)", i_label, i_timing->milliseconds,
                    i_timing->succeeded ? "成功" : "失敗");
    }
    else { ImGui::Text("%s: 未計測", i_label); }
}
}

void GenerationDebug::DrawToggles()
{
    ImGui::Checkbox("Navmesh 生成設定", &m_bSettingsVisible);
    ImGui::Checkbox("Navmesh 処理時間", &m_bTimingsVisible);
}

void GenerationDebug::LoadSettings(const NavmeshSettings& i_settings)
{
    // 符号付き入力欄に表現できない値を黙って切り詰めず、設定の読込失敗を通知する。
    const auto limit = static_cast<std::size_t>(std::numeric_limits<int>::max());
    if (i_settings.tileColumns > limit || i_settings.tileRows > limit ||
        (i_settings.voxel.borderCells && *i_settings.voxel.borderCells > limit))
    {
        throw std::length_error("生成設定を入力欄に表示できません。");
    }
    m_stDraft = i_settings;
    m_iColumns = static_cast<int>(i_settings.tileColumns);
    m_iRows = static_cast<int>(i_settings.tileRows);
    m_bAutomaticBorder = !i_settings.voxel.borderCells;
    m_iBorder = i_settings.voxel.borderCells ? static_cast<int>(*i_settings.voxel.borderCells) : 0;
    m_iPartition = i_settings.voxel.partitionMethod == region::PartitionMethod::Watershed ? 1 : 0;
    m_instError.clear();
}

void GenerationDebug::DrawControls(Navmesh* io_navmesh, const std::function<void()>& i_onRebuilt)
{
    if (m_bSettingsVisible)
    {
        if (ImGui::Begin("Navmesh 生成設定", &m_bSettingsVisible))
        {
            // 編集中の値は構築結果に反映せず、再生成の指示まで独立して保持する。
            ImGui::InputInt("タイル数 X", &m_iColumns);
            ImGui::InputInt("タイル数 Z", &m_iRows);
            ImGui::InputFloat("セル幅", &m_stDraft.raster.cellSize);
            ImGui::InputFloat("セル高さ", &m_stDraft.raster.cellHeight);
            ImGui::InputFloat("最大傾斜角", &m_stDraft.raster.maxSlopeDegrees);
            ImGui::InputFloat("エージェント高さ", &m_stDraft.voxel.walkable.agentHeight);
            ImGui::InputFloat("最大段差", &m_stDraft.voxel.walkable.maxClimb);
            ImGui::InputFloat("エージェント半径", &m_stDraft.voxel.agentRadius);
            ImGui::Checkbox("境界余白を自動計算", &m_bAutomaticBorder);
            ImGui::BeginDisabled(m_bAutomaticBorder);
            ImGui::InputInt("境界余白セル数", &m_iBorder);
            ImGui::EndDisabled();
            ImGui::Combo("領域分割方式", &m_iPartition, "単調分割\0流域分割\0");

            ImGui::BeginDisabled(!io_navmesh);
            if (ImGui::Button("現在の設定を読み込む"))
            {
                try { LoadSettings(io_navmesh->Settings()); }
                catch (const std::length_error& i_error) { m_instError = i_error.what(); std::cerr << m_instError << '\n'; }
            }
            ImGui::SameLine();
            if (ImGui::Button("Navmesh を再生成"))
            {
                try
                {
                    if (m_iColumns <= 0 || m_iRows <= 0 || (!m_bAutomaticBorder && m_iBorder < 0))
                    {
                        throw std::invalid_argument("タイル数は正、境界余白はゼロ以上にしてください。");
                    }
                    auto settings = m_stDraft;
                    settings.tileColumns = static_cast<std::size_t>(m_iColumns);
                    settings.tileRows = static_cast<std::size_t>(m_iRows);
                    settings.voxel.borderCells = m_bAutomaticBorder ? std::nullopt :
                        std::optional<std::size_t>(static_cast<std::size_t>(m_iBorder));
                    settings.voxel.partitionMethod = m_iPartition == 0 ?
                        region::PartitionMethod::Monotone : region::PartitionMethod::Watershed;
                    io_navmesh->Rebuild(settings);
                    m_stDraft = settings;
                    m_instError.clear();
                    if (i_onRebuilt) { i_onRebuilt(); }
                }
                catch (const std::invalid_argument& i_error) { m_instError = i_error.what(); std::cerr << m_instError << '\n'; }
                catch (const std::length_error& i_error) { m_instError = i_error.what(); std::cerr << m_instError << '\n'; }
                catch (const std::overflow_error& i_error) { m_instError = i_error.what(); std::cerr << m_instError << '\n'; }
            }
            ImGui::EndDisabled();
            if (!io_navmesh) { ImGui::TextUnformatted("ステージを選択してください。"); }
            ImGui::TextUnformatted("再生成時は障害物を保持します。");
            if (!m_instError.empty()) { ImGui::TextWrapped("%s", m_instError.c_str()); }
        }
        ImGui::End();
    }

    if (m_bTimingsVisible)
    {
        if (ImGui::Begin("Navmesh 処理時間", &m_bTimingsVisible))
        {
            ImGui::TextUnformatted("各操作の直近の実時間。描画時間は含みません。");
            if (io_navmesh)
            {
                const auto& timings = io_navmesh->Timings();
                DrawTiming("全体再生成", timings.rebuild);
                DrawTiming("地形変換", timings.transform);
                DrawTiming("領域分割方式変更", timings.partition);
                DrawTiming("障害物追加", timings.obstacleAdd);
                DrawTiming("障害物個別削除", timings.obstacleRemove);
                DrawTiming("障害物全削除", timings.obstacleClear);
            }
            else { ImGui::TextUnformatted("ステージを選択してください。"); }
        }
        ImGui::End();
    }
}
}
