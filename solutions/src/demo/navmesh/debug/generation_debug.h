/**
 * @file generation_debug.h
 * @brief Navmesh の生成設定と更新処理の計測結果を表示する。
 * @date 2026-11-01
 * @par 最終更新日 2026-11-01
 */
#pragma once
#ifndef NAVMESH_DEMO_NAVMESH_DEBUG_GENERATION_DEBUG_H
#define NAVMESH_DEMO_NAVMESH_DEBUG_GENERATION_DEBUG_H

#include "../navmesh.h"
#include <functional>
#include <string>

namespace demo::navmesh::debug
{
/**
 * @brief 未適用の生成設定と二つのデバッグ画面の表示状態を保持する。
 */
class GenerationDebug final
{
public:
    /**
     * @brief デバッグ機能ウィンドウに表示切り替えを追加する。
     */
    void DrawToggles();

    /**
     * @brief 適用済み設定を編集欄に取り込む。
     * @param i_settings 取り込む設定。
     */
    void LoadSettings(const NavmeshSettings& i_settings);

    /**
     * @brief 設定と計測結果のウィンドウを描画する。
     * @param io_navmesh 再生成する Navmesh。未選択の場合は nullptr。
     * @param i_onRebuilt 再生成成功後に表示データを破棄する処理。
     */
    void DrawControls(Navmesh* io_navmesh, const std::function<void()>& i_onRebuilt);

private:
    NavmeshSettings m_stDraft;       // 再生成時に適用する編集途中の設定。
    int m_iColumns = 4;              // 編集中の X 方向タイル数。
    int m_iRows = 4;                 // 編集中の Z 方向タイル数。
    int m_iBorder = 0;               // 手動指定する境界余白。
    int m_iPartition = 0;            // 編集中の領域分割方式。
    bool m_bAutomaticBorder = true;   // 境界余白を自動計算するか。
    bool m_bSettingsVisible = false; // 生成設定ウィンドウを表示するか。
    bool m_bTimingsVisible = false;  // 処理時間ウィンドウを表示するか。
    std::string m_instError;         // 設定または再生成の失敗理由。
};
}
#endif
