/**
 * @file application.cpp
 * @brief アプリケーションの初期化と描画ループを実行する。
 * @date 2026-10-28
 * @par 最終更新日 2026-11-01
 */
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "demo/application/application.h"
#include "demo/application/imgui_runtime.h"
#include "demo/application/stage_assets.h"
#include "demo/application/stage_controls.h"
#include "demo/component/model/model_component.h"
#include "demo/component/model/model_traits.h"
#include "demo/ecs/component_world.h"
#include "demo/navmesh/debug/bounds_debug.h"
#include "demo/navmesh/debug/generation_debug.h"
#include "demo/navmesh/debug/obstacle_debug.h"
#include "demo/navmesh/debug/voxel_debug.h"
#include "demo/navmesh/navmesh.h"
#include "general/util/path_util.h"
#include "render/camera.h"
#include "render/renderer.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{
using ModelComponent = demo::component::model::ModelComponent;
using Transform = demo::component::transform::LocalTransform;
using World = demo::ecs::ComponentWorld<Transform, ModelComponent>;
using Clock = std::chrono::steady_clock;
using demo::application::ImGuiRuntime;

/**
 * @brief GLFW の初期化と終了を管理する。
 */
struct GlfwRuntime final
{
    GlfwRuntime()
    {
        // 初期化中のエラーも確認できるよう、GLFW を開始する前に通知先を登録する。
        glfwSetErrorCallback([](int i_code, const char* i_message)
        {
            std::cerr << "GLFW [" << i_code << "]: " << i_message << '\n';
        });
        if (!glfwInit()) { throw std::runtime_error("GLFW の初期化に失敗しました。"); }
    }
    ~GlfwRuntime() { glfwTerminate(); }
    GlfwRuntime(const GlfwRuntime&) = delete;
    GlfwRuntime& operator=(const GlfwRuntime&) = delete;
};

/**
 * @brief 初期化後の GLFW ウィンドウで描画ループを実行する。
 * @return 終了コード。
 */
int RunInternal()
{
    // GLFW を初期化し、描画資源を作る前に OpenGL コンテキストを持つウィンドウを用意する。
    auto glfw = GlfwRuntime{};
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // ウィンドウを自動解放し、描画資源より後にコンテキストを破棄する。
    auto window = std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)>(
        glfwCreateWindow(1280, 720, "navmesh_demo - Model Viewer", nullptr, nullptr), glfwDestroyWindow);
    if (!window) { throw std::runtime_error("OpenGL 4.3 ウィンドウを作成できません。"); }

    // このウィンドウを現在の描画先とし、GLEW にコアプロファイルの OpenGL 関数を読み込ませる。
    glfwMakeContextCurrent(window.get());
    glewExperimental = GL_TRUE;
    const auto glewResult = glewInit();
    if (glewResult != GLEW_OK)
    {
        throw std::runtime_error("GLEW の初期化に失敗しました: " +
            std::string(reinterpret_cast<const char*>(glewGetErrorString(glewResult))));
    }
    // コアプロファイルの GLEW 初期化で残る GL_INVALID_ENUM は許容し、他のエラーは検出する。
    const auto probeError = glGetError();
    if (probeError != GL_NO_ERROR && probeError != GL_INVALID_ENUM)
    {
        throw std::runtime_error("GLEW の初期化中に OpenGL エラーが発生しました。");
    }
    if (!GLEW_VERSION_4_3) { throw std::runtime_error("OpenGL 4.3 以降が必要です。"); }
    glfwSwapInterval(1);

    // OpenGL コンテキストを使う描画機能と UI を初期化する。
    auto renderer = render::Renderer{};
    auto imgui = ImGuiRuntime(window.get());

    // フィールドの描画状態を保持するワールドと Entity を用意する。
    // Entity の作成要求は更新時に処理されるため、結果を確認してから識別子を使う。
    auto world = World{};
    const auto request = world.RequestCreate(Transform{}, ModelComponent{});
    world.Update(0.0F);
    const auto result = world.TakeResult(request);
    if (result.status != demo::ecs::RequestStatus::Succeeded || !result.entity)
    {
        throw std::runtime_error("フィールド Entity の作成に失敗しました: " + result.message);
    }
    const auto field = *result.entity;

    // カメラと照明を初期化し、ステージ一覧を検索する。
    auto camera = render::Camera{};
    auto lighting = render::Lighting{};
    const auto stageDirectory = general::util::PathFromUtf8(DYNAMIC_WIND_STAGE_DIRECTORY);
    auto controls = demo::application::StageControls(stageDirectory, lighting);
    auto navmesh = std::optional<demo::navmesh::Navmesh>{};
    auto navmeshSettings = demo::navmesh::NavmeshSettings{};
    auto terrainTransform = std::array<float, 16>{};
    auto terrainLocalTransform = world.Transforms().GetLocal(field);
    auto boundsDebug = demo::navmesh::debug::BoundsDebug{};
    auto generationDebug = demo::navmesh::debug::GenerationDebug{};
    auto voxelDebug = demo::navmesh::debug::VoxelDebug{};
    auto obstacleDebug = demo::navmesh::debug::ObstacleDebug{};
    controls.Refresh();
    auto previous = Clock::now();
    while (!glfwWindowShouldClose(window.get()))
    {
        // 停止後の復帰時にカメラが大きく移動しないよう、更新に渡す時間を 0.1 秒までに抑える。
        const auto now = Clock::now();
        const auto elapsed = std::min(std::chrono::duration<float>(now - previous).count(), 0.1F);
        previous = now;
        glfwPollEvents();
        if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) { glfwSetWindowShouldClose(window.get(), GLFW_TRUE); }

        // 最小化などで描画領域がない場合、イベントを待ってから再確認する。
        auto width = 0;
        auto height = 0;
        glfwGetFramebufferSize(window.get(), &width, &height);
        if (width <= 0 || height <= 0)
        {
            glfwWaitEventsTimeout(0.05);
            continue;
        }

        // GLFW と OpenGL のバックエンドを更新し、新しい UI フレームを開始する。
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ステージ読込時に現在の姿勢を参照できるよう、UI の選択処理より先に同期する。
        world.SynchronizeTransforms();
        demo::application::UpdateCameraInput(window.get(), camera, elapsed);
        controls.Draw(lighting, [&](const auto& i_path)
        {
            // 地形と描画モデルを同じ入力から作り、両方の準備ができた場合だけ切り替える。
            const auto transform = world.Transforms().GetWorldMatrix(field);
            auto model = demo::application::LoadStageWithNavmesh(i_path, transform, navmeshSettings, navmesh);
            world.ForEach<ModelComponent>([&](const auto& i_entity, auto& io_component)
            {
                if (i_entity == field) { io_component.model = model; }
            });
            terrainTransform = transform.Data();
            terrainLocalTransform = world.Transforms().GetLocal(field);
            voxelDebug.Reset();
            generationDebug.LoadSettings(navmesh->Settings());
        });
        boundsDebug.DrawControls(navmesh ? &navmesh->TerrainBounds() : nullptr,
                                 [&] { voxelDebug.DrawToggle(); obstacleDebug.DrawToggle(); generationDebug.DrawToggles(); });
        voxelDebug.DrawControls(navmesh ? &*navmesh : nullptr);
        generationDebug.DrawControls(navmesh ? &*navmesh : nullptr, [&] { voxelDebug.Reset(); });
        if (navmesh) { navmeshSettings = navmesh->Settings(); }
        obstacleDebug.DrawControls(navmesh ? &*navmesh : nullptr);
        // ワールドの変更と姿勢を同期し、描画先をクリアして各モデルを描画する。
        world.Update(elapsed);
        world.SynchronizeTransforms();
        if (navmesh)
        {
            // フィールドが移動した場合も、描画と地形の座標を一致させる。
            const auto transform = world.Transforms().GetWorldMatrix(field);
            if (transform.Data() != terrainTransform)
            {
                // フィールドは親のない Entity のため、失敗時は直前のローカル姿勢に戻す。
                const auto reportFailure = [&](const std::exception& i_error)
                {
                    world.SetTransform(field, terrainLocalTransform);
                    world.SynchronizeTransforms();
                    controls.ReportError(std::string("Navmesh の変換更新に失敗しました: ") + i_error.what());
                };
                try
                {
                    navmesh->SetWorldTransform(transform);
                    terrainTransform = transform.Data();
                    terrainLocalTransform = world.Transforms().GetLocal(field);
                    voxelDebug.Reset();
                }
                catch (const std::invalid_argument& i_error) { reportFailure(i_error); }
                catch (const std::length_error& i_error) { reportFailure(i_error); }
                catch (const std::overflow_error& i_error) { reportFailure(i_error); }
            }
        }
        obstacleDebug.HandleInput(navmesh ? &*navmesh : nullptr, camera,
                                  static_cast<float>(width) / static_cast<float>(height));
        glViewport(0, 0, width, height);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        glClearColor(0.05F, 0.05F, 0.08F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const auto voxelDrawn = voxelDebug.Draw(navmesh ? &*navmesh : nullptr, camera,
                                                static_cast<float>(width) / static_cast<float>(height));
        world.ForEachRead<ModelComponent>([&](const auto& i_entity, const auto& i_component)
        {
            if (i_component.model && !(voxelDrawn && i_entity == field))
            {
                renderer.Draw(*i_component.model, world.Transforms().GetWorldMatrix(i_entity), camera,
                              static_cast<float>(width) / static_cast<float>(height), lighting);
            }
        });
        voxelDebug.DrawPolygons(navmesh ? &*navmesh : nullptr, camera,
                                static_cast<float>(width) / static_cast<float>(height));
        voxelDebug.DrawContours(navmesh ? &*navmesh : nullptr, camera,
                                static_cast<float>(width) / static_cast<float>(height));
        obstacleDebug.Draw(navmesh ? &*navmesh : nullptr, camera,
                           static_cast<float>(width) / static_cast<float>(height));
        boundsDebug.Draw(navmesh ? &navmesh->TerrainBounds() : nullptr, camera,
                         static_cast<float>(width) / static_cast<float>(height));
        // モデルの上に UI を描画してからバッファを交換し、完成したフレームを表示する。
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window.get());
    }
    return 0;
}
}

int demo::application::Run()
{
    return RunInternal();
}
