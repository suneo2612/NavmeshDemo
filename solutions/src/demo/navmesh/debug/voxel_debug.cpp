/**
 * @file voxel_debug.cpp
 * @brief 地形ボクセルと距離場のデバッグ設定・描画を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "voxel_debug.h"
#include "contour_lines.h"
#include "polygon_triangles.h"
#include <imgui.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kDetailsRightOffset = 1040.0F;
constexpr auto kInitialTopOffset = 20.0F;
}

/**
 * @brief 開いているデバッグ機能ウィンドウに表示切り替えを追加する。
 */
void VoxelDebug::DrawToggle()
{
    ImGui::Checkbox("地形ボクセル+安全距離確認", &m_bVisible);
}

/**
 * @brief 表示切り替えと独立した設定ウィンドウを描画する。
 * @param io_navmesh 現在の構築結果。未選択の場合は nullptr。
 * @details 設定ウィンドウを閉じるとボクセル表示も無効になる。
 */
void VoxelDebug::DrawControls(Navmesh* io_navmesh)
{
    if (m_bVisible)
    {
        ImGui::SetNextWindowPos(ImVec2(std::max(0.0F, ImGui::GetIO().DisplaySize.x - kDetailsRightOffset),
                                       kInitialTopOffset), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("地形ボクセル・距離場デバッグ", &m_bVisible))
        {
            ImGui::Checkbox("地形ボクセル", &m_bShowVoxels);
            ImGui::BeginDisabled(!m_bShowVoxels);
            ImGui::Checkbox("DistanceField", &m_bShowDistance);
            ImGui::BeginDisabled(!m_bShowDistance);
            if (ImGui::RadioButton("元の距離", !m_bSmoothed)) { m_bSmoothed = false; }
            ImGui::SameLine();
            if (ImGui::RadioButton("平滑化後", m_bSmoothed)) { m_bSmoothed = true; }
            ImGui::EndDisabled();
            ImGui::Checkbox("安全距離確認", &m_bShowEroded);
            ImGui::BeginDisabled(!m_bShowDistance);
            ImGui::Checkbox("侵食後の距離場", &m_bAfterErosion);
            ImGui::EndDisabled();
            ImGui::EndDisabled();
            ImGui::TextUnformatted("距離: 黒=近い / 白=遠い / 紫=未到達");
            ImGui::TextUnformatted("赤: キャラクター半径による侵食面");
            ImGui::TextUnformatted("青灰色: 歩行面以外。距離は安全を保証しません。");
            if (!io_navmesh || !io_navmesh->VoxelTiles()) { ImGui::TextUnformatted("表示できる地形ボクセルがありません。"); }
            ImGui::BeginDisabled(!m_bShowVoxels);
            ImGui::Checkbox("領域の色分け", &m_bShowRegions);
            ImGui::EndDisabled();
            if (m_bShowRegions) { ImGui::TextUnformatted("領域の色はタイル内の識別子ごとに表示します。"); }
            const auto available = io_navmesh && io_navmesh->VoxelTiles();
            // Navmesh の切り替えや設定変更を検出し、UI が保持する分割方式を実際の設定に同期する。
            if (io_navmesh && (m_pControlsNavmesh != io_navmesh ||
                m_instAppliedMethod != io_navmesh->Settings().voxel.partitionMethod))
            {
                m_pControlsNavmesh = io_navmesh;
                m_instAppliedMethod = io_navmesh->Settings().voxel.partitionMethod;
                m_iRequestedMethod = m_instAppliedMethod == region::PartitionMethod::Watershed ? 1 : 0;
            }
            ImGui::BeginDisabled(!available);
            ImGui::Combo("領域分割方式", &m_iRequestedMethod, "単調分割\0流域分割\0");
            if (ImGui::Button("分割方式を適用"))
            {
                // 選択された分割方法を適用し、構築失敗は各エラー表示に分けて通知する。
                try
                {
                    io_navmesh->SetPartitionMethod(m_iRequestedMethod == 0 ?
                        region::PartitionMethod::Monotone : region::PartitionMethod::Watershed);
                    m_instAppliedMethod = io_navmesh->Settings().voxel.partitionMethod;
                    m_instError.clear();
                }
                catch (const std::invalid_argument& i_error) { ReportError(i_error.what()); }
                catch (const std::length_error& i_error) { ReportError(i_error.what()); }
                catch (const std::overflow_error& i_error) { ReportError(i_error.what()); }
            }
            ImGui::EndDisabled();
            ImGui::Checkbox("領域の輪郭", &m_bShowContours);
            if (m_bShowContours) { ImGui::TextUnformatted("黄: 外周 / 水色: 穴・退化輪郭"); }
            ImGui::Checkbox("Navmesh ポリゴン", &m_bShowPolygons);
            if (m_bShowPolygons) { ImGui::TextUnformatted("緑: 歩行面 / 黄: 凸ポリゴンの辺"); }
            ImGui::Checkbox("Navmesh ポリゴン接続", &m_bShowLinks);
            if (m_bShowLinks) { ImGui::TextUnformatted("水色: ポリゴン中心と通行可能な共有区間"); }
            if (!m_instError.empty()) { ImGui::TextWrapped("%s", m_instError.c_str()); }
            if (!m_instContourError.empty()) { ImGui::TextWrapped("%s", m_instContourError.c_str()); }
            if (!m_instPolygonError.empty()) { ImGui::TextWrapped("%s", m_instPolygonError.c_str()); }
        }
        ImGui::End();
    }
}

/**
 * @brief 地形またはタイルが更新されたときに表示データを破棄する。
 */
void VoxelDebug::Invalidate() noexcept
{
    // 描画用キャッシュと各生成試行状態を無効化し、次の有効フレームで再作成できるようにする。
    m_bAttempted = false;
    m_bReady = false;
    m_pTiles = nullptr;
    m_pContourTiles = nullptr;
    m_bContourAttempted = false;
    m_bContourReady = false;
    m_instContourError.clear();
    m_pPolygonTiles = nullptr;
    m_bPolygonAttempted = false;
    m_bPolygonReady = false;
    m_instPolygonError.clear();
    m_instError.clear();
}

/**
 * @brief 地形の切り替え時に表示データと距離の色基準を破棄する。
 */
void VoxelDebug::Reset() noexcept
{
    // タイルに結び付くキャッシュを無効化した後、地形単位で保持する距離基準も破棄する。
    Invalidate();
    m_pControlsNavmesh = nullptr;
    m_pScaleTiles = nullptr;
    m_instRawMaximum.reset();
    m_instSmoothedMaximum.reset();
    m_instErodedRawMaximum.reset();
    m_instErodedSmoothedMaximum.reset();
}

/**
 * @brief エラーを画面と標準エラーに通知する。
 * @param i_message 失敗理由。
 */
void VoxelDebug::ReportError(const char* i_message)
{
    m_instError = i_message;
    std::cerr << "ボクセル表示: " << m_instError << '\n';
}

/**
 * @brief 輪郭のエラーを画面と標準エラーに通知する。
 * @param i_message 失敗理由。
 */
void VoxelDebug::ReportContourError(const char* i_message)
{
    m_instContourError = i_message;
    std::cerr << "輪郭表示: " << m_instContourError << '\n';
}

/**
 * @brief ポリゴンのエラーを画面と標準エラーに通知する。
 * @param i_message 失敗理由。
 */
void VoxelDebug::ReportPolygonError(const char* i_message)
{
    m_instPolygonError = i_message;
    std::cerr << "ポリゴン表示: " << m_instPolygonError << '\n';
}

/**
 * @brief 有効な場合に Navmesh の三角形を地形の上に描画する。
 * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @details 失敗時は画面と標準エラーに理由を通知する。
 */
void VoxelDebug::DrawPolygons(const Navmesh* i_navmesh, const render::Camera& i_camera, const float i_aspect)
{
    // 表示設定と現在の構築結果を照合し、対象が変わった場合はポリゴン描画キャッシュを破棄する。
    if (!m_bVisible || (!m_bShowPolygons && !m_bShowLinks) || !i_navmesh || !i_navmesh->VoxelTiles()) { return; }
    const auto* tiles = i_navmesh->VoxelTiles();
    if (tiles != m_pPolygonTiles || i_navmesh->Revision() != m_uPolygonRevision)
    {
        m_pPolygonTiles = tiles;
        m_uPolygonRevision = i_navmesh->Revision();
        m_bPolygonAttempted = false;
        m_bPolygonReady = false;
    }
    if (!m_bPolygonAttempted)
    {
        // 現在の世代につき一度だけ頂点・辺・リンクを生成し、失敗時はこの世代での再試行を避ける。
        m_bPolygonAttempted = true;
        try
        {
            m_instPolygonVertices = BuildPolygonTriangles(*i_navmesh);
            m_stPolygonLines = BuildPolygonLines(*i_navmesh);
            m_stLinkLines = BuildPolygonLinkLines(*i_navmesh);
            if (!m_instPolygonRenderer) { m_instPolygonRenderer = std::make_unique<BoundsRenderer>(); }
            m_bPolygonReady = true;
            m_instPolygonError.clear();
        }
        catch (const std::invalid_argument& i_error) { ReportPolygonError(i_error.what()); }
        catch (const std::length_error& i_error) { ReportPolygonError(i_error.what()); }
        catch (const std::overflow_error& i_error) { ReportPolygonError(i_error.what()); }
        catch (const std::runtime_error& i_error) { ReportPolygonError(i_error.what()); }
    }
    if (!m_bPolygonReady) { return; }
    // ポリゴン面・外周・接続線はそれぞれ独立した表示設定に従って描画する。
    try
    {
        if (m_bShowPolygons)
        {
            m_instPolygonRenderer->DrawTriangles(m_instPolygonVertices, i_camera, i_aspect);
            m_instPolygonRenderer->Draw(m_stPolygonLines, i_camera, i_aspect, true);
        }
        if (m_bShowLinks) { m_instPolygonRenderer->Draw(m_stLinkLines, i_camera, i_aspect, true); }
    }
    catch (const std::invalid_argument& i_error) { m_bPolygonReady = false; ReportPolygonError(i_error.what()); }
    catch (const std::runtime_error& i_error) { m_bPolygonReady = false; ReportPolygonError(i_error.what()); }
}

/**
 * @brief 有効な場合に領域の輪郭を地形の上に描画する。
 * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @details 失敗時は画面と標準エラーに理由を通知する。
 */
void VoxelDebug::DrawContours(const Navmesh* i_navmesh, const render::Camera& i_camera, const float i_aspect)
{
    // 表示条件を満たすときだけ処理し、タイルや世代の更新を検出して輪郭キャッシュを無効化する。
    if (!m_bVisible || !m_bShowContours || !i_navmesh || !i_navmesh->VoxelTiles()) { return; }
    const auto* tiles = i_navmesh->VoxelTiles();
    if (tiles != m_pContourTiles || i_navmesh->Revision() != m_uContourRevision)
    {
        m_pContourTiles = tiles;
        m_uContourRevision = i_navmesh->Revision();
        m_bContourAttempted = false;
        m_bContourReady = false;
    }
    if (!m_bContourAttempted)
    {
        // 輪郭頂点を一度だけ作成し、構築失敗を記録してフレームごとの再計算を防ぐ。
        m_bContourAttempted = true;
        try
        {
            m_stContourLines = BuildContourLines(*i_navmesh);
            if (!m_instContourRenderer) { m_instContourRenderer = std::make_unique<BoundsRenderer>(); }
            m_bContourReady = true;
            m_instContourError.clear();
        }
        catch (const std::invalid_argument& i_error) { ReportContourError(i_error.what()); }
        catch (const std::length_error& i_error) { ReportContourError(i_error.what()); }
        catch (const std::overflow_error& i_error) { ReportContourError(i_error.what()); }
        catch (const std::runtime_error& i_error) { ReportContourError(i_error.what()); }
    }
    if (!m_bContourReady) { return; }
    // 生成済みの輪郭を描画し、OpenGL 側の失敗は表示状態を無効化して再試行可能にする。
    try { m_instContourRenderer->Draw(m_stContourLines, i_camera, i_aspect); }
    catch (const std::invalid_argument& i_error) { m_bContourReady = false; ReportContourError(i_error.what()); }
    catch (const std::runtime_error& i_error) { m_bContourReady = false; ReportContourError(i_error.what()); }
}

/**
 * @brief 表示が有効ならボクセルを描画する。
 * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @return 描画に成功し、元の地形描画を省略できる場合は true。
 * @details 作成・描画の失敗は画面と標準エラーに通知し、元の地形を残す。
 */
bool VoxelDebug::Draw(const Navmesh* i_navmesh, const render::Camera& i_camera, const float i_aspect)
{
    // 表示対象がある場合、タイルの変更に合わせて距離色基準と頂点キャッシュを管理する。
    if (!m_bVisible || !m_bShowVoxels || !i_navmesh || !i_navmesh->VoxelTiles()) { return false; }
    const auto* tiles = i_navmesh->VoxelTiles();
    if (tiles != m_pScaleTiles)
    {
        m_pScaleTiles = tiles;
        m_instRawMaximum.reset();
        m_instSmoothedMaximum.reset();
        m_instErodedRawMaximum.reset();
        m_instErodedSmoothedMaximum.reset();
    }
    const auto mode = m_bShowRegions ? VoxelColorMode::Regions : !m_bShowDistance ? VoxelColorMode::Terrain :
        m_bSmoothed ? VoxelColorMode::Smoothed : VoxelColorMode::Raw;
    if (tiles != m_pTiles || mode != m_instMode || m_bShowEroded != m_bDisplayedEroded ||
        m_bAfterErosion != m_bDisplayedAfterErosion ||
        i_navmesh->Revision() != m_uRevision) { Invalidate(); }
    if (!m_bAttempted)
    {
        // 表示データが完成するまで元の地形を隠さず、失敗した条件では再試行を繰り返さない。
        m_bAttempted = true;
        m_pTiles = tiles;
        m_instMode = mode;
        m_bDisplayedEroded = m_bShowEroded;
        m_bDisplayedAfterErosion = m_bAfterErosion;
        m_uRevision = i_navmesh->Revision();
        try
        {
            // 全体最大距離を色基準として保持し、障害物の局所更新で未変更タイルの色が変わるのを防ぐ。
            if (!m_instRawMaximum) { m_instRawMaximum = MaximumVoxelDistance(*i_navmesh, VoxelColorMode::Raw); }
            if (!m_instSmoothedMaximum) { m_instSmoothedMaximum = MaximumVoxelDistance(*i_navmesh, VoxelColorMode::Smoothed); }
            if (!m_instErodedRawMaximum) { m_instErodedRawMaximum = MaximumVoxelDistance(*i_navmesh, VoxelColorMode::Raw, true); }
            if (!m_instErodedSmoothedMaximum) { m_instErodedSmoothedMaximum = MaximumVoxelDistance(*i_navmesh, VoxelColorMode::Smoothed, true); }
            const auto rawMaximum = m_bAfterErosion ? m_instErodedRawMaximum : m_instRawMaximum;
            const auto smoothMaximum = m_bAfterErosion ? m_instErodedSmoothedMaximum : m_instSmoothedMaximum;
            const auto maximum = mode == VoxelColorMode::Raw ? rawMaximum :
                mode == VoxelColorMode::Smoothed ? smoothMaximum : std::nullopt;
            auto instances = BuildVoxelInstances(*i_navmesh, mode, maximum, m_bShowEroded, m_bAfterErosion);
            if (!m_instRenderer) { m_instRenderer = std::make_unique<VoxelRenderer>(); }
            m_instRenderer->Upload(instances);
            m_bReady = true;
            m_instError.clear();
        }
        catch (const std::invalid_argument& i_error) { ReportError(i_error.what()); }
        catch (const std::length_error& i_error) { ReportError(i_error.what()); }
        catch (const std::runtime_error& i_error) { ReportError(i_error.what()); }
    }
    if (!m_bReady) { return false; }
    // 描画成功時だけ元地形の代わりにボクセル表示を使い、失敗時は呼び出し側へ false を返す。
    try { m_instRenderer->Draw(i_camera, i_aspect); }
    catch (const std::invalid_argument& i_error) { m_bReady = false; ReportError(i_error.what()); return false; }
    catch (const std::runtime_error& i_error) { m_bReady = false; ReportError(i_error.what()); return false; }
    return true;
}
}
