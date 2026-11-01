/**
 * @file obstacle_debug.cpp
 * @brief 障害物の配置・削除と箱の描画を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "obstacle_debug.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kWidth = 1.0F;
constexpr auto kHeight = 2.0F;
constexpr auto kDepth = 1.0F;
constexpr auto kObstacleColor = std::array<float, 3>{1.0F, 0.35F, 0.05F};
constexpr auto kMaxDisplayedObstacles = std::size_t{100000};
constexpr auto kDetailsRightOffset = 700.0F;
constexpr auto kDetailsTopOffset = 260.0F;

/**
 * @brief 地形の交点を底面中心にした固定寸法の箱を作る。
 * @param i_position 地形との交点。
 * @return ワールド空間の箱。
 * @throws std::overflow_error 座標を表現できない場合。
 */
Bounds MakeObstacleBounds(const general::vec::Vector3& i_position)
{
    // 地形交点を底面中心として固定サイズの箱を計算し、float 範囲を超えないことを確認する。
    const auto minX = static_cast<double>(i_position.x) - kWidth * 0.5;
    const auto maxX = static_cast<double>(i_position.x) + kWidth * 0.5;
    const auto minZ = static_cast<double>(i_position.z) - kDepth * 0.5;
    const auto maxZ = static_cast<double>(i_position.z) + kDepth * 0.5;
    const auto maxY = static_cast<double>(i_position.y) + kHeight;
    const auto limit = static_cast<double>(std::numeric_limits<float>::max());
    if (minX < -limit || maxX > limit || minZ < -limit || maxZ > limit || maxY > limit)
    {
        throw std::overflow_error("障害物の座標を表現できません。");
    }
    return {{static_cast<float>(minX), i_position.y, static_cast<float>(minZ)},
            {static_cast<float>(maxX), static_cast<float>(maxY), static_cast<float>(maxZ)}};
}
}

/**
 * @brief 開いているデバッグ機能ウィンドウに表示切り替えを追加する。
 */
void ObstacleDebug::DrawToggle()
{
    ImGui::Checkbox("デバッグ障害物配置", &m_bVisible);
}

/**
 * @brief 配置設定と全削除ボタンを描画する。
 * @param io_navmesh 更新する構築結果。未選択の場合は nullptr。
 */
void ObstacleDebug::DrawControls(Navmesh* io_navmesh)
{
    if (!m_bVisible) { return; }
    ImGui::SetNextWindowPos(ImVec2(std::max(0.0F, ImGui::GetIO().DisplaySize.x - kDetailsRightOffset),
                                   kDetailsTopOffset), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("デバッグ障害物", &m_bVisible))
    {
        // ボクセル結果がない間は障害物操作を無効にし、操作失敗は画面と標準エラーへ通知する。
        const auto available = io_navmesh && io_navmesh->VoxelTiles();
        ImGui::BeginDisabled(!available);
        ImGui::Checkbox("クリックで配置する", &m_bPlace);
        if (ImGui::Button("全削除"))
        {
            try { io_navmesh->ClearObstacles(); m_instError.clear(); }
            catch (const std::invalid_argument& i_error) { ReportError(i_error.what()); }
            catch (const std::length_error& i_error) { ReportError(i_error.what()); }
            catch (const std::overflow_error& i_error) { ReportError(i_error.what()); }
        }
        ImGui::EndDisabled();
        ImGui::TextUnformatted("Shift + 左クリック: 個別削除");
        ImGui::TextUnformatted("障害物のサイズ: 幅 1 × 高さ 2 × 奥行き 1");
        if (!available) { ImGui::TextUnformatted("ボクセル構築済みのステージを選択してください。"); }
        if (!m_instError.empty()) { ImGui::TextWrapped("%s", m_instError.c_str()); }
    }
    ImGui::End();
}

/**
 * @brief UI に捕捉されていないクリックを配置または削除として処理する。
 * @param io_navmesh 更新する構築結果。未選択の場合は nullptr。
 * @param i_camera カメラ。
 * @param i_aspect フレームバッファーの横縦比。
 * @details Shift を押しながら左クリックすると最も手前の障害物を削除する。
 */
void ObstacleDebug::HandleInput(Navmesh* io_navmesh, const render::Camera& i_camera, const float i_aspect)
{
    const auto& io = ImGui::GetIO();
    // UI 操作中や対象地形がない場合は入力を消費せず、条件を満たすクリックだけを選択処理へ進める。
    if (!m_bVisible || !io_navmesh || !io_navmesh->VoxelTiles() || io.WantCaptureMouse ||
        !ImGui::IsMouseClicked(ImGuiMouseButton_Left) || (!io.KeyShift && !m_bPlace))
    {
        return;
    }
    try
    {
        const auto ray = ScreenPickRay(i_camera, io.MousePos.x, io.MousePos.y,
                                       io.DisplaySize.x, io.DisplaySize.y, i_aspect);
        if (!ray) { return; }
        const auto terrain = PickTerrain(io_navmesh->TerrainData(), *ray);
        if (io.KeyShift)
        {
            // Shift 操作では地形交点より手前にある箱だけを調べて、背後の障害物を誤削除しない。
            const auto maxDistance = terrain ? terrain->distance : std::numeric_limits<double>::infinity();
            const auto id = PickObstacle(io_navmesh->VoxelTiles()->Obstacles(), *ray, maxDistance);
            if (id) { io_navmesh->RemoveObstacle(*id); m_instError.clear(); }
        }
        else if (terrain)
        {
            // 地形に当たった位置を新しい障害物の底面中心として追加する。
            static_cast<void>(io_navmesh->AddObstacle(MakeObstacleBounds(terrain->position)));
            m_instError.clear();
        }
    }
    catch (const std::invalid_argument& i_error) { ReportError(i_error.what()); }
    catch (const std::length_error& i_error) { ReportError(i_error.what()); }
    catch (const std::overflow_error& i_error) { ReportError(i_error.what()); }
}

/**
 * @brief エラーを画面と標準エラーに通知する。
 * @param i_message 失敗理由。
 */
void ObstacleDebug::ReportError(const char* i_message)
{
    m_instError = i_message;
    std::cerr << "障害物デバッグ: " << m_instError << '\n';
}

/**
 * @brief 配置済み障害物を描画する。設定ウィンドウを閉じても表示を続ける。
 * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
 * @param i_camera カメラ。
 * @param i_aspect フレームバッファーの横縦比。
 */
void ObstacleDebug::Draw(const Navmesh* i_navmesh, const render::Camera& i_camera, const float i_aspect)
{
    // 障害物がある場合にのみ表示し、タイルポインターと世代が変われば GPU 表示データを作り直す。
    if (!i_navmesh || !i_navmesh->VoxelTiles() || i_navmesh->VoxelTiles()->Obstacles().empty()) { return; }
    const auto* tiles = i_navmesh->VoxelTiles();
    if (m_pTiles != tiles || m_uRevision != i_navmesh->Revision()) { m_bAttempted = false; m_bReady = false; }
    if (!m_bAttempted)
    {
        m_bAttempted = true;
        m_pTiles = tiles;
        m_uRevision = i_navmesh->Revision();
        try
        {
            const auto& obstacles = tiles->Obstacles();
            if (obstacles.size() > kMaxDisplayedObstacles)
            {
                throw std::length_error("表示する障害物数が上限を超えました。");
            }
            auto instances = std::vector<VoxelInstance>{};
            auto outlines = BoundsLines{};
            instances.reserve(obstacles.size());
            // 各箱をインスタンス描画データと輪郭線へ変換し、現在の障害物状態を一括登録する。
            for (const auto& obstacle : obstacles)
            {
                const auto& bounds = obstacle.bounds;
                auto size = std::array<float, 3>{};
                for (auto axis = std::size_t{}; axis < 3; ++axis)
                {
                    const auto width = static_cast<double>(bounds.maximum[axis]) - bounds.minimum[axis];
                    if (width > std::numeric_limits<float>::max())
                    {
                        throw std::overflow_error("障害物の描画寸法が上限を超えました。");
                    }
                    size[axis] = static_cast<float>(width);
                }
                instances.push_back({{bounds.minimum.x, bounds.minimum.y, bounds.minimum.z}, size, kObstacleColor});
                const auto box = BuildBoundsLines(bounds, 1, 1, true, false);
                outlines.terrain.insert(outlines.terrain.end(), box.terrain.begin(), box.terrain.end());
            }
            if (!m_instRenderer) { m_instRenderer = std::make_unique<VoxelRenderer>(); }
            if (!m_instOutlineRenderer) { m_instOutlineRenderer = std::make_unique<BoundsRenderer>(); }
            m_instRenderer->Upload(instances);
            m_stOutlines = std::move(outlines);
            m_bReady = true;
        }
        catch (const std::invalid_argument& i_error) { ReportError(i_error.what()); }
        catch (const std::length_error& i_error) { ReportError(i_error.what()); }
        catch (const std::overflow_error& i_error) { ReportError(i_error.what()); }
        catch (const std::runtime_error& i_error) { ReportError(i_error.what()); }
    }
    if (!m_bReady) { return; }
    try
    {
        m_instRenderer->Draw(i_camera, i_aspect);
        m_instOutlineRenderer->Draw(m_stOutlines, i_camera, i_aspect);
    }
    catch (const std::invalid_argument& i_error) { m_bReady = false; ReportError(i_error.what()); }
    catch (const std::runtime_error& i_error) { m_bReady = false; ReportError(i_error.what()); }
}
}
