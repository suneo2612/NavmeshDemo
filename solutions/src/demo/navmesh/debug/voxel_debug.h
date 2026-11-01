/**
 * @file voxel_debug.h
 * @brief 地形ボクセルと距離場のデバッグ設定・描画を管理する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_VOXEL_DEBUG_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_VOXEL_DEBUG_H

#include "voxel_renderer.h"
#include "bounds_renderer.h"
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace demo::navmesh::debug
{
/**
 * @brief ImGui の設定とボクセルの表示を管理する。
 * @details 描画資源は OpenGL コンテキストのあるスレッドで使用・破棄する。
 */
class VoxelDebug final
{
public:
    /**
     * @brief 開いているデバッグ機能ウィンドウに表示切り替えを追加する。
     */
    void DrawToggle();

    /**
     * @brief 表示切り替えと独立した設定ウィンドウを描画する。
     * @param io_navmesh 現在の構築結果。未選択の場合は nullptr。
     * @details 設定ウィンドウを閉じるとボクセル表示も無効になる。
     */
    void DrawControls(Navmesh* io_navmesh);

    /**
     * @brief 表示が有効ならボクセルを描画する。
     * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @return 描画に成功し、元の地形描画を省略できる場合は true。
     * @details 作成・描画の失敗は画面と標準エラーに通知し、元の地形を残す。
     */
    bool Draw(const Navmesh* i_navmesh, const render::Camera& i_camera, float i_aspect);

    /**
     * @brief 有効な場合に領域の輪郭を地形の上に描画する。
     * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @details 失敗時は画面と標準エラーに理由を通知する。
     */
    void DrawContours(const Navmesh* i_navmesh, const render::Camera& i_camera, float i_aspect);

    /**
     * @brief 有効な場合に Navmesh の三角形を地形の上に描画する。
     * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @details 失敗時は画面と標準エラーに理由を通知する。
     */
    void DrawPolygons(const Navmesh* i_navmesh, const render::Camera& i_camera, float i_aspect);

    /**
     * @brief 地形またはタイルが更新されたときに表示データを破棄する。
     */
    void Invalidate() noexcept;

    /**
     * @brief 地形の切り替え時に表示データと距離の色基準を破棄する。
     */
    void Reset() noexcept;

    /**
     * @brief 設定ウィンドウの表示状態を返す。
     * @return 有効な場合は true。
     */
    bool IsVisible() const noexcept { return m_bVisible; }

private:
    /**
     * @brief エラーを画面と標準エラーに通知する。
     * @param i_message 失敗理由。
     */
    void ReportError(const char* i_message);

    /**
     * @brief 輪郭のエラーを画面と標準エラーに通知する。
     * @param i_message 失敗理由。
     */
    void ReportContourError(const char* i_message);

    /**
     * @brief ポリゴンのエラーを画面と標準エラーに通知する。
     * @param i_message 失敗理由。
     */
    void ReportPolygonError(const char* i_message);

    bool m_bVisible = false;           // 設定ウィンドウとボクセル表示の切り替え。
    bool m_bShowVoxels = true;         // 地形をボクセルに置き換えるか。
    bool m_bShowDistance = false;      // 歩行面を距離で着色するか。
    bool m_bShowRegions = false;       // 距離に代えて領域ごとの色を表示するか。
    bool m_bShowContours = false;      // 領域の外周と穴の輪郭を表示するか。
    bool m_bShowPolygons = false;      // Navmesh の三角形を表示するか。
    bool m_bShowLinks = false;         // ポリゴン間の接続線を表示するか。
    bool m_bPolygonAttempted = false;  // 現在のポリゴン描画データの生成を試みたか。
    bool m_bPolygonReady = false;      // 現在のポリゴン描画データを使用できるか。
    bool m_bContourAttempted = false; // 現在の輪郭データの生成を試みたか。
    bool m_bContourReady = false;     // 現在の輪郭データを描画できるか。
    bool m_bSmoothed = false;          // 平滑化後の距離を選ぶか。
    bool m_bShowEroded = false;        // 侵食面を赤色で表示するか。
    bool m_bDisplayedEroded = false;   // 表示データに反映した侵食面の表示状態。
    bool m_bAfterErosion = false;     // 侵食後の距離場を表示するか。
    bool m_bDisplayedAfterErosion = false; // 表示データに反映した距離場。
    bool m_bAttempted = false;         // 現在の条件で表示データの作成を試みたか。
    bool m_bReady = false;             // 現在の条件の表示データが登録済みか。
    const voxelization::DynamicVoxelTiles* m_pTiles = nullptr; // 表示に反映したタイル。
    const voxelization::DynamicVoxelTiles* m_pScaleTiles = nullptr; // 距離の色基準を求めたタイル。
    std::uint64_t m_uRevision = 0;                            // 表示に反映した構築結果の世代。
    std::optional<std::uint32_t> m_instRawMaximum;            // 平滑化前の固定された色基準。
    std::optional<std::uint32_t> m_instSmoothedMaximum;       // 平滑化後の固定された色基準。
    std::optional<std::uint32_t> m_instErodedRawMaximum;      // 侵食後の元距離の固定基準。
    std::optional<std::uint32_t> m_instErodedSmoothedMaximum; // 侵食後の平滑化距離の固定基準。
    VoxelColorMode m_instMode = VoxelColorMode::Terrain;       // 表示に反映した距離の選択。
    const Navmesh* m_pControlsNavmesh = nullptr;               // 分割方式の選択元。
    region::PartitionMethod m_instAppliedMethod = region::PartitionMethod::Monotone; // 表示中の適用方式。
    int m_iRequestedMethod = 0;                               // 次に適用する方式。
    std::string m_instError;                                  // 表示の失敗理由。
    std::string m_instContourError;                           // 輪郭表示の失敗理由。
    std::string m_instPolygonError;                           // ポリゴン表示の失敗理由。
    std::unique_ptr<VoxelRenderer> m_instRenderer;             // ボクセルの OpenGL 描画資源。
    const voxelization::DynamicVoxelTiles* m_pContourTiles = nullptr; // 輪郭を生成したタイル。
    std::uint64_t m_uContourRevision = 0;                    // 輪郭に反映した構築結果の世代。
    BoundsLines m_stContourLines;                             // 輪郭のワールド座標。
    std::unique_ptr<BoundsRenderer> m_instContourRenderer;    // 輪郭の OpenGL 描画資源。
    const voxelization::DynamicVoxelTiles* m_pPolygonTiles = nullptr; // ポリゴンを生成したタイル。
    std::uint64_t m_uPolygonRevision = 0;                    // ポリゴンに反映した構築結果の世代。
    std::vector<std::array<float, 3>> m_instPolygonVertices; // 表示用の三角形頂点。
    BoundsLines m_stPolygonLines;                            // 表示用の凸多角形の境界辺。
    BoundsLines m_stLinkLines;                               // ポリゴン中心と共有区間を結ぶ線。
    std::unique_ptr<BoundsRenderer> m_instPolygonRenderer;    // ポリゴンの OpenGL 描画資源。
};
}
#endif
