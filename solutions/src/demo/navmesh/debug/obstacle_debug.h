/**
 * @file obstacle_debug.h
 * @brief 箱型障害物のデバッグ操作と表示を管理する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_OBSTACLE_DEBUG_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_OBSTACLE_DEBUG_H

#include "bounds_renderer.h"
#include "obstacle_picking.h"
#include "voxel_renderer.h"
#include <memory>
#include <string>

namespace demo::navmesh::debug
{
/**
 * @brief ImGui と画面クリックで箱型障害物を配置・削除する。
 * @details OpenGL 資源はコンテキストのあるスレッドで使用・破棄する。
 */
class ObstacleDebug final
{
public:
    /**
     * @brief 開いているデバッグ機能ウィンドウに表示切り替えを追加する。
     */
    void DrawToggle();

    /**
     * @brief 配置設定と全削除ボタンを描画する。
     * @param io_navmesh 更新する構築結果。未選択の場合は nullptr。
     */
    void DrawControls(Navmesh* io_navmesh);

    /**
     * @brief UI に捕捉されていないクリックを配置または削除として処理する。
     * @param io_navmesh 更新する構築結果。未選択の場合は nullptr。
     * @param i_camera カメラ。
     * @param i_aspect フレームバッファーの横縦比。
     * @details Shift を押しながら左クリックすると最も手前の障害物を削除する。
     */
    void HandleInput(Navmesh* io_navmesh, const render::Camera& i_camera, float i_aspect);

    /**
     * @brief 配置済み障害物を描画する。設定ウィンドウを閉じても表示を続ける。
     * @param i_navmesh 現在の構築結果。未選択の場合は nullptr。
     * @param i_camera カメラ。
     * @param i_aspect フレームバッファーの横縦比。
     */
    void Draw(const Navmesh* i_navmesh, const render::Camera& i_camera, float i_aspect);

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

    bool m_bVisible = false;    // 障害物設定ウィンドウの表示。
    bool m_bPlace = false;      // 左クリックによる配置の有効状態。
    bool m_bAttempted = false;  // 現在の結果で表示データを作成済みか。
    bool m_bReady = false;      // 表示データが登録済みか。
    const voxelization::DynamicVoxelTiles* m_pTiles = nullptr; // 表示に反映したタイル。
    std::uint64_t m_uRevision = 0;                            // 表示に反映した世代。
    std::string m_instError;                                   // 操作または描画の失敗理由。
    BoundsLines m_stOutlines;                                  // 障害物を示す枠。
    std::unique_ptr<VoxelRenderer> m_instRenderer;             // 箱の描画資源。
    std::unique_ptr<BoundsRenderer> m_instOutlineRenderer;     // 枠の描画資源。
};
}
#endif
