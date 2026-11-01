/**
 * @file stage_controls.h
 * @brief カメラ入力とステージ設定 UI を扱う。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_APPLICATION_STAGE_CONTROLS_H
#define NAVMESH_DEMO_DEMO_APPLICATION_STAGE_CONTROLS_H

#include <array>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

struct GLFWwindow;
namespace render { class Camera; struct Lighting; }

namespace demo::application
{
/**
 * @brief ImGui が使用していないマウス・キー入力をカメラへ反映する。
 * @param i_window 入力を読み取る GLFW ウィンドウ。
 * @param io_camera 更新するカメラ。
 * @param i_elapsed 前フレームからの経過時間（秒）。
 */
void UpdateCameraInput(GLFWwindow* i_window, render::Camera& io_camera, float i_elapsed);

/**
 * @brief ステージ一覧と入力途中の照明設定を保持し、設定 UI を描画する。
 */
class StageControls final
{
public:
    /**
     * @brief 検索先と照明の初期入力値を設定する。
     * @param i_directory ステージの検索先。
     * @param i_lighting 照明の初期値。
     */
    StageControls(const std::filesystem::path& i_directory, const render::Lighting& i_lighting);

    /**
     * @brief ステージ一覧を再検索し、失敗時は画面と標準エラーに理由を表示する。
     */
    void Refresh();

    /**
     * @brief ステージに関連するエラーを画面と標準エラーに表示する。
     * @param i_message 失敗理由。
     */
    void ReportError(const std::string& i_message);

    /**
     * @brief ステージと照明の設定 UI を描画する。
     * @param io_lighting 有効な入力を反映する照明設定。
      * @param i_setStage 選択したファイルをフィールドに設定する処理。失敗時は例外を送出する。
     * @details 有効な OpenGL コンテキストと ImGui フレームが必要。
     */
    void Draw(render::Lighting& io_lighting,
              const std::function<void(const std::filesystem::path&)>& i_setStage);

private:
    std::filesystem::path m_instDirectory;              // ステージの検索先。
    std::vector<std::filesystem::path> m_instStages;    // 検索済みステージの一覧。
    std::string m_instSelected;                         // 選択中の相対パス。
    std::string m_instError;                            // 検索または読込のエラー。
    std::string m_instLightError;                       // 照明方向の入力エラー。
    std::array<float, 3> m_instDirection;               // 編集途中の照明方向。
};
}
#endif
