/**
 * @file bounds_debug.h
 * @brief 地形とタイル枠のデバッグ設定・描画を管理する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_BOUNDS_DEBUG_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_BOUNDS_DEBUG_H

#include "bounds_renderer.h"
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

namespace demo::navmesh::debug
{
/**
 * @brief ImGui の設定と地形・タイル枠の表示を管理する。
 * @details 描画資源は OpenGL コンテキストのあるスレッドで使用・破棄する。
 */
class BoundsDebug final
{
public:
    /**
     * @brief 表示切り替えと、有効時に独立した地形分割の設定ウィンドウを描画する。
     * @param i_bounds 現在の地形範囲。未選択の場合は nullptr。
     * @param i_extraToggle 同じデバッグ機能ウィンドウに追加する切り替え。
     * @details 有効な ImGui フレームが必要。設定ウィンドウを閉じると枠の表示も無効になる。
     */
    void DrawControls(const Bounds* i_bounds, const std::function<void()>& i_extraToggle = {});

    /**
     * @brief 設定が有効な場合に現在の地形範囲とタイル枠を描画する。
     * @param i_bounds 現在の地形範囲。未選択の場合は nullptr。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @throws std::runtime_error OpenGL の処理が失敗した場合。
     * @details 有効な OpenGL コンテキストが必要。
     */
    void Draw(const Bounds* i_bounds, const render::Camera& i_camera, float i_aspect);

    /**
     * @brief 枠の表示が有効かを返す。
     * @return 表示が有効な場合は true。
     */
    bool IsVisible() const noexcept { return m_bVisible; }

private:
    using CacheKey = std::tuple<float, float, float, float, float, float, int, int, bool, bool>;

    bool m_bVisible = false;                                 // 設定ウィンドウと枠の表示切り替え。
    bool m_bShowTerrain = true;                              // 地形全体の枠を表示するか。
    bool m_bShowTiles = true;                                // タイルごとの枠を表示するか。
    int m_iColumns = 4;                                     // 表示用の X 分割数。
    int m_iRows = 4;                                        // 表示用の Z 分割数。
    std::string m_instError;                                // 地形範囲または分割数のエラー。
    std::unique_ptr<BoundsRenderer> m_instRenderer;         // 枠の OpenGL 描画資源。
    BoundsLines m_stLines;                                  // 再利用する線分。
    std::optional<CacheKey> m_instKey;                      // 線分に反映済みの範囲と設定。
};
}
#endif
