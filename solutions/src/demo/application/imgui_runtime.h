/**
 * @file imgui_runtime.h
 * @brief ImGui と描画バックエンドの寿命を管理する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_APPLICATION_IMGUI_RUNTIME_H
#define NAVMESH_DEMO_DEMO_APPLICATION_IMGUI_RUNTIME_H

struct GLFWwindow;

namespace demo::application
{
/**
 * @brief ImGui コンテキストと GLFW/OpenGL バックエンドを管理する。
 */
class ImGuiRuntime final
{
public:
    /**
     * @brief 描画コンテキストが有効なウィンドウにバックエンドを接続する。
     * @param i_window バックエンドに接続する GLFW ウィンドウ。
     * @throws std::runtime_error バックエンドの初期化に失敗した場合。
     */
    explicit ImGuiRuntime(GLFWwindow* i_window);

    /**
     * @brief 初期化と逆の順序でバックエンドとコンテキストを解放する。
     */
    ~ImGuiRuntime();
    ImGuiRuntime(const ImGuiRuntime&) = delete;
    ImGuiRuntime& operator=(const ImGuiRuntime&) = delete;
};
}
#endif
