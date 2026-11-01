/**
 * @file voxel_renderer.cpp
 * @brief 共有立方体と位置・色のインスタンス配列でボクセルを描画する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "voxel_renderer.h"
#include "gl_debug.h"
#include "../../../render/camera.h"
#include "shader_sources.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace demo::navmesh::debug
{
namespace
{
// シェーダーは６頂点ごとに面の法線を割り当てるため、この面の順序を維持する。
constexpr auto kCube = std::array<float, 108>{
    0,0,0, 0,1,0, 1,1,0, 0,0,0, 1,1,0, 1,0,0,
    0,0,1, 1,1,1, 0,1,1, 0,0,1, 1,0,1, 1,1,1,
    0,0,0, 0,0,1, 0,1,1, 0,0,0, 0,1,1, 0,1,0,
    1,0,0, 1,1,1, 1,0,1, 1,0,0, 1,1,0, 1,1,1,
    0,0,0, 1,0,1, 0,0,1, 0,0,0, 1,0,0, 1,0,1,
    0,1,0, 0,1,1, 1,1,1, 0,1,0, 1,1,1, 1,1,0};
constexpr auto kVerticesPerCube = GLsizei{36};
}

struct VoxelRenderer::Impl
{
    GLuint m_uiProgram = 0;     // 描画プログラム。
    GLuint m_uiVertexArray = 0; // 共有立方体とインスタンスの頂点配列。
    GLuint m_uiCube = 0;        // 共有立方体の頂点。
    GLuint m_uiInstances = 0;   // ボクセルごとの位置・寸法・色。
    GLint m_iView = -1;         // ビュー行列の位置。
    GLint m_iProjection = -1;   // 投影行列の位置。
    GLsizei m_iCount = 0;       // 登録済みボクセル数。

    /**
     * @brief 描画資源を解放する。
     */
    ~Impl()
    {
        glDeleteBuffers(1, &m_uiInstances);
        glDeleteBuffers(1, &m_uiCube);
        glDeleteVertexArrays(1, &m_uiVertexArray);
        if (m_uiProgram) { glDeleteProgram(m_uiProgram); }
    }
};

/**
 * @brief 描画に必要な OpenGL 資源を作る。
 * @throws std::runtime_error シェーダーまたは資源の作成に失敗した場合。
 */
VoxelRenderer::VoxelRenderer() : m_instImpl(std::make_unique<Impl>())
{
    // 描画プログラムをリンクし、カメラ行列を渡す uniform が利用可能か確認する。
    const auto vertex = gl::Shader(GL_VERTEX_SHADER, shaders::navmeshVoxelVertex);
    const auto fragment = gl::Shader(GL_FRAGMENT_SHADER, shaders::navmeshVoxelFragment);
    m_instImpl->m_uiProgram = glCreateProgram();
    if (!m_instImpl->m_uiProgram) { throw std::runtime_error("ボクセル描画プログラムを作成できません。"); }
    glAttachShader(m_instImpl->m_uiProgram, vertex.Id());
    glAttachShader(m_instImpl->m_uiProgram, fragment.Id());
    glLinkProgram(m_instImpl->m_uiProgram);
    auto success = GLint{};
    glGetProgramiv(m_instImpl->m_uiProgram, GL_LINK_STATUS, &success);
    if (success != GL_TRUE)
    {
        auto length = GLint{};
        glGetProgramiv(m_instImpl->m_uiProgram, GL_INFO_LOG_LENGTH, &length);
        auto log = std::string(static_cast<std::size_t>(std::max(length, 1)), '\0');
        glGetProgramInfoLog(m_instImpl->m_uiProgram, length, nullptr, log.data());
        throw std::runtime_error("ボクセル描画プログラムのリンクに失敗しました: " + log);
    }
    m_instImpl->m_iView = glGetUniformLocation(m_instImpl->m_uiProgram, "view");
    m_instImpl->m_iProjection = glGetUniformLocation(m_instImpl->m_uiProgram, "projection");
    if (m_instImpl->m_iView < 0 || m_instImpl->m_iProjection < 0)
    {
        throw std::runtime_error("ボクセル描画シェーダーの設定が見つかりません。");
    }

    // 共有立方体とインスタンス属性を専用 VAO/VBO に登録し、呼び出し側の OpenGL 状態は RAII で復元する。
    const auto state = gl::DrawState{};
    glGenVertexArrays(1, &m_instImpl->m_uiVertexArray);
    glGenBuffers(1, &m_instImpl->m_uiCube);
    glGenBuffers(1, &m_instImpl->m_uiInstances);
    if (!m_instImpl->m_uiVertexArray || !m_instImpl->m_uiCube || !m_instImpl->m_uiInstances)
    {
        throw std::runtime_error("ボクセル描画用の頂点資源を作成できません。");
    }
    glBindVertexArray(m_instImpl->m_uiVertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_instImpl->m_uiCube);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kCube), kCube.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, m_instImpl->m_uiInstances);
    const auto offsets = std::array<std::size_t, 3>{
        offsetof(VoxelInstance, minimum), offsetof(VoxelInstance, size), offsetof(VoxelInstance, color)};
    for (auto index = GLuint{}; index < offsets.size(); ++index)
    {
        const auto location = index + 1;
        glEnableVertexAttribArray(location);
        glVertexAttribPointer(location, 3, GL_FLOAT, GL_FALSE, sizeof(VoxelInstance),
                              reinterpret_cast<const void*>(offsets[index]));
        glVertexAttribDivisor(location, 1);
    }
    gl::Check("ボクセル描画資源の作成");
}

/**
 * @brief 所有する OpenGL 描画資源を解放する。
 */
VoxelRenderer::~VoxelRenderer() = default;

/**
 * @brief 描画する位置・寸法・色を GPU に登録する。
 * @param i_instances ボクセルの配列。
 * @throws std::length_error 配列が OpenGL の上限を超える場合。
 * @throws std::runtime_error OpenGL の処理が失敗した場合。
 */
void VoxelRenderer::Upload(const std::vector<VoxelInstance>& i_instances)
{
    // インスタンス数とバイト数を検査し、転送に失敗した場合は描画件数をゼロのままにする。
    if (i_instances.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) ||
        i_instances.size() > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(VoxelInstance))
    {
        throw std::length_error("表示するボクセル数が OpenGL の上限を超えました。");
    }
    const auto state = gl::DrawState{};
    m_instImpl->m_iCount = 0;
    glBindBuffer(GL_ARRAY_BUFFER, m_instImpl->m_uiInstances);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(i_instances.size() * sizeof(VoxelInstance)),
                 i_instances.data(), GL_STATIC_DRAW);
    gl::Check("ボクセル描画データの登録");
    m_instImpl->m_iCount = static_cast<GLsizei>(i_instances.size());
}

/**
 * @brief 登録済みボクセルを深度判定付きで描画する。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @throws std::runtime_error OpenGL の処理が失敗した場合。
 */
void VoxelRenderer::Draw(const render::Camera& i_camera, const float i_aspect) const
{
    // カメラ行列と一時的な描画状態を準備し、共有立方体を登録済み件数だけインスタンス描画する。
    const auto view = i_camera.View();
    const auto projection = render::Camera::Projection(i_aspect);
    const auto state = gl::DrawState{};
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glUseProgram(m_instImpl->m_uiProgram);
    glUniformMatrix4fv(m_instImpl->m_iView, 1, GL_FALSE, view.data());
    glUniformMatrix4fv(m_instImpl->m_iProjection, 1, GL_FALSE, projection.data());
    glBindVertexArray(m_instImpl->m_uiVertexArray);
    glDrawArraysInstanced(GL_TRIANGLES, 0, kVerticesPerCube, m_instImpl->m_iCount);
    gl::Check("ボクセルの描画");
}
}
