/**
 * @file stage_controls.cpp
 * @brief カメラ入力とステージ設定 UI を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "stage_controls.h"
#include "demo/stage/stage_files.h"
#include "general/util/path_util.h"
#include "render/camera.h"
#include "render/renderer.h"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace demo::application
{
void UpdateCameraInput(GLFWwindow* i_window, render::Camera& io_camera, const float i_elapsed)
{
    // UI に捕捉された入力をカメラへ重複して反映せず、右ドラッグだけで視点を回転する。
    const auto& io = ImGui::GetIO();
    if (!io.WantCaptureMouse && ImGui::IsMouseDragging(ImGuiMouseButton_Right))
    {
        io_camera.Rotate(io.MouseDelta.x, io.MouseDelta.y);
    }
    // WASD の反対方向の入力を相殺し、経過時間に応じた移動量を適用する。
    if (!io.WantCaptureKeyboard)
    {
        const auto forward = (glfwGetKey(i_window, GLFW_KEY_W) == GLFW_PRESS ? 1.0F : 0.0F) -
            (glfwGetKey(i_window, GLFW_KEY_S) == GLFW_PRESS ? 1.0F : 0.0F);
        const auto right = (glfwGetKey(i_window, GLFW_KEY_D) == GLFW_PRESS ? 1.0F : 0.0F) -
            (glfwGetKey(i_window, GLFW_KEY_A) == GLFW_PRESS ? 1.0F : 0.0F);
        io_camera.Move(forward * i_elapsed * 5.0F, right * i_elapsed * 5.0F);
    }
}

StageControls::StageControls(const std::filesystem::path& i_directory, const render::Lighting& i_lighting)
    : m_instDirectory(i_directory),
      m_instDirection{i_lighting.direction.x, i_lighting.direction.y, i_lighting.direction.z}
{
}

void StageControls::Refresh()
{
    // 検索失敗時は前回の一覧を維持し、エラーを画面にも表示する。
    try { m_instStages = demo::stage::FindStages(m_instDirectory); m_instError.clear(); }
    catch (const std::filesystem::filesystem_error& i_error)
    {
        m_instError = i_error.what();
        std::cerr << m_instError << '\n';
    }
}

void StageControls::ReportError(const std::string& i_message)
{
    m_instError = i_message;
    std::cerr << m_instError << '\n';
}

void StageControls::Draw(render::Lighting& io_lighting,
                         const std::function<void(const std::filesystem::path&)>& i_setStage)
{
    // 再検索と選択を同じ画面に置き、空の一覧でも検索先の状態を知らせる。
    ImGui::Begin("Stage");
    if (ImGui::Button("Refresh")) { Refresh(); }
    if (m_instStages.empty()) { ImGui::TextUnformatted("No FBX files in the stage directory."); }
    if (ImGui::BeginCombo("Model", m_instSelected.empty() ? "(none)" : m_instSelected.c_str()))
    {
        // モデルを読み込めた場合だけ選択状態とフィールドの描画を切り替える。
        for (const auto& path : m_instStages)
        {
            const auto name = general::util::PathToUtf8(path.lexically_relative(m_instDirectory));
            ImGui::PushID(&path);
            if (ImGui::Selectable(name.c_str(), name == m_instSelected))
            {
                try
                {
                    i_setStage(path);
                    m_instSelected = name;
                    m_instError.clear();
                }
                catch (const std::runtime_error& i_error)
                {
                    ReportError(i_error.what());
                }
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    // 照明の有効状態と方向を編集し、不正な方向は入力欄に残して修正できるようにする。
    ImGui::Checkbox("Lighting", &io_lighting.enabled);
    if (ImGui::InputFloat3("Light direction", m_instDirection.data()))
    {
        // 入力途中の不正な方向を描画設定に反映しない。
        const auto candidate = general::vec::Vector3(m_instDirection[0], m_instDirection[1], m_instDirection[2]);
        const auto length = candidate.Length();
        if (std::isfinite(length) && length > 1e-8F)
        {
            io_lighting.direction = candidate;
            m_instLightError.clear();
        }
        else { m_instLightError = "Light direction must be finite and nonzero."; }
    }
    // 検索・読み込み・照明のエラーを画面で確認できるようにする。
    if (!m_instError.empty()) { ImGui::TextWrapped("%s", m_instError.c_str()); }
    if (!m_instLightError.empty()) { ImGui::TextWrapped("%s", m_instLightError.c_str()); }
    ImGui::TextUnformatted("Right drag: look  |  WASD: move");
    ImGui::End();
}
}
