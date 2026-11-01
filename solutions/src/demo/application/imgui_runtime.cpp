/**
 * @file imgui_runtime.cpp
 * @brief ImGui のフォント、コンテキスト、描画バックエンドを初期化する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "imgui_runtime.h"
#include "general/util/path_util.h"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace demo::application
{
ImGuiRuntime::ImGuiRuntime(GLFWwindow* i_window)
{
    IMGUI_CHECKVERSION();
    // 設定ファイルを生成せず、起動時の UI 状態を一定にする。
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;

    // Windows のフォントディレクトリが指定されていれば、日本語フォントを探す。
    // 見つからない場合や読み込めない場合は警告し、標準フォントで続行する。
    if (const auto* root = std::getenv("WINDIR"))
    {
        const auto fontPath = std::filesystem::path(root) / "Fonts" / "meiryo.ttc";
        if (std::filesystem::exists(fontPath))
        {
            const auto font = general::util::PathToUtf8(fontPath);
            if (!ImGui::GetIO().Fonts->AddFontFromFileTTF(
                    font.c_str(), 16.0F, nullptr, ImGui::GetIO().Fonts->GetGlyphRangesJapanese()))
            {
                std::cerr << "日本語フォントを読み込めませんでした。\n";
            }
        }
        else { std::cerr << "日本語フォントが見つかりません。標準フォントを使用します。\n"; }
    }
    ImGui::StyleColorsDark();

    // GLFW を先に初期化し、後続の OpenGL 初期化に失敗した場合は取得済みの資源を逆順に解放する。
    if (!ImGui_ImplGlfw_InitForOpenGL(i_window, true))
    {
        ImGui::DestroyContext();
        throw std::runtime_error("ImGui の GLFW 初期化に失敗しました。");
    }
    if (!ImGui_ImplOpenGL3_Init("#version 430 core"))
    {
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        throw std::runtime_error("ImGui の OpenGL 初期化に失敗しました。");
    }
}

ImGuiRuntime::~ImGuiRuntime()
{
    // 描画コンテキストが有効なうちに両バックエンドを停止し、最後に ImGui を破棄する。
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}
}
