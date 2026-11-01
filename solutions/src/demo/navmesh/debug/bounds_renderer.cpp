/**
 * @file bounds_renderer.cpp
 * @brief 地形とタイルのワイヤーフレーム描画を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "bounds_renderer.h"
#include "gl_debug.h"
#include "../../../render/camera.h"
#include "shader_sources.hpp"
#include <GL/glew.h>
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <string>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kTerrainColor = std::array<float, 3>{1.0F, 0.8F, 0.1F};
constexpr auto kTileColor = std::array<float, 3>{0.1F, 0.9F, 1.0F};
constexpr auto kPolygonColor = std::array<float, 3>{0.2F, 0.9F, 0.35F};
constexpr auto kLinkColor = std::array<float, 3>{1.0F, 0.1F, 0.1F};

/**
 * @brief 線分頂点の大きさが OpenGL に渡せることを確認する。
 * @param i_vertices 検証する頂点。
 */
void ValidateVertices(const std::vector<std::array<float, 3>>& i_vertices)
{
    // GL_LINES は頂点を2個単位で、描画数とバッファーサイズはいずれも OpenGL 型の範囲で受け取る。
    if (i_vertices.size() % 2 != 0 ||
        i_vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) ||
        i_vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) /
            sizeof(i_vertices.front()))
    {
        throw std::invalid_argument("デバッグ線分の頂点数が不正です。");
    }
}
}

struct BoundsRenderer::Impl
{
    GLuint m_uiProgram = 0;       // 描画プログラム。
    GLuint m_uiVertexArray = 0;   // 線分の頂点配列。
    GLuint m_uiBuffer = 0;        // 線分の頂点バッファー。
    GLint m_iView = -1;           // ビュー行列の位置。
    GLint m_iProjection = -1;     // 投影行列の位置。
    GLint m_iColor = -1;          // 線の色の位置。

    /**
     * @brief 描画資源を解放する。
     */
    ~Impl()
    {
        glDeleteBuffers(1, &m_uiBuffer);
        glDeleteVertexArrays(1, &m_uiVertexArray);
        if (m_uiProgram) { glDeleteProgram(m_uiProgram); }
    }
};

/**
 * @brief 描画に必要な OpenGL 資源を作る。
 * @throws std::runtime_error シェーダーまたは資源の作成に失敗した場合。
 */
BoundsRenderer::BoundsRenderer() : m_instImpl(std::make_unique<Impl>())
{
    // 描画専用シェーダーをリンクし、必要な設定位置を確認する。
    const auto vertex = gl::Shader(GL_VERTEX_SHADER, shaders::navmeshBoundsVertex);
    const auto fragment = gl::Shader(GL_FRAGMENT_SHADER, shaders::navmeshBoundsFragment);
    m_instImpl->m_uiProgram = glCreateProgram();
    if (!m_instImpl->m_uiProgram) { throw std::runtime_error("デバッグ描画プログラムを作成できません。"); }
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
        throw std::runtime_error("デバッグ描画プログラムのリンクに失敗しました: " + log);
    }
    m_instImpl->m_iView = glGetUniformLocation(m_instImpl->m_uiProgram, "view");
    m_instImpl->m_iProjection = glGetUniformLocation(m_instImpl->m_uiProgram, "projection");
    m_instImpl->m_iColor = glGetUniformLocation(m_instImpl->m_uiProgram, "lineColor");
    if (m_instImpl->m_iView < 0 || m_instImpl->m_iProjection < 0 || m_instImpl->m_iColor < 0)
    {
        throw std::runtime_error("デバッグ描画シェーダーの設定が見つかりません。");
    }
    glGenVertexArrays(1, &m_instImpl->m_uiVertexArray);
    glGenBuffers(1, &m_instImpl->m_uiBuffer);
    if (!m_instImpl->m_uiVertexArray || !m_instImpl->m_uiBuffer)
    {
        throw std::runtime_error("デバッグ描画用の頂点資源を作成できません。");
    }
    gl::Check("デバッグ描画資源の作成");
}

/**
 * @brief 所有する OpenGL 描画資源を解放する。
 */
BoundsRenderer::~BoundsRenderer() = default;

/**
 * @brief 線分を地形の上に重ねて描画する。
 * @param i_lines 地形、タイルとポリゴン接続の線分。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @param i_depthTest 地形や他の面に隠す場合は true。
 * @throws std::runtime_error OpenGL の処理が失敗した場合。
 * @throws std::invalid_argument 頂点数または画面の比率が不正な場合。
 */
void BoundsRenderer::Draw(const BoundsLines& i_lines, const render::Camera& i_camera, const float i_aspect,
                          const bool i_depthTest) const
{
    // 全カテゴリの頂点を先に検証し、描画状態をスコープ終了時に復元できるよう保存する。
    ValidateVertices(i_lines.terrain);
    ValidateVertices(i_lines.tiles);
    ValidateVertices(i_lines.links);
    const auto view = i_camera.View();
    const auto projection = render::Camera::Projection(i_aspect);
    const auto state = gl::DrawState{};

    // 指定された深度条件で共通の描画設定を行い、色別の配列を順に GPU へ転送する。
    if (i_depthTest) { glEnable(GL_DEPTH_TEST); } else { glDisable(GL_DEPTH_TEST); }
    glDisable(GL_BLEND);
    glUseProgram(m_instImpl->m_uiProgram);
    glUniformMatrix4fv(m_instImpl->m_iView, 1, GL_FALSE, view.data());
    glUniformMatrix4fv(m_instImpl->m_iProjection, 1, GL_FALSE, projection.data());
    glBindVertexArray(m_instImpl->m_uiVertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_instImpl->m_uiBuffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(std::array<float, 3>)), nullptr);

    // 空のカテゴリは送信を省き、データがあるカテゴリだけを色付き線分として描く。
    const auto draw = [&](const auto& i_vertices, const auto& i_color)
    {
        if (i_vertices.empty()) { return; }
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(i_vertices.size() * sizeof(i_vertices.front())),
                     i_vertices.data(), GL_STREAM_DRAW);
        glUniform3fv(m_instImpl->m_iColor, 1, i_color.data());
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(i_vertices.size()));
    };
    draw(i_lines.tiles, kTileColor);
    draw(i_lines.terrain, kTerrainColor);
    draw(i_lines.links, kLinkColor);
    gl::Check("デバッグ枠の描画");
}

/**
 * @brief 三角形を深度判定付きで地形に重ねる。
 * @param i_vertices 三頂点ごとに一つの三角形。
 * @param i_camera カメラ。
 * @param i_aspect 描画領域の横幅と高さの比率。
 * @throws std::runtime_error OpenGL の処理が失敗した場合。
 * @throws std::invalid_argument 頂点数または画面の比率が不正な場合。
 */
void BoundsRenderer::DrawTriangles(const std::vector<std::array<float, 3>>& i_vertices,
                                    const render::Camera& i_camera, const float i_aspect) const
{
    // 三角形単位の配列と OpenGL のサイズ上限を確認し、空配列なら GPU 操作を省く。
    if (i_vertices.size() % 3 != 0 ||
        i_vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) ||
        i_vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) /
            sizeof(i_vertices.front()))
    {
        throw std::invalid_argument("デバッグ三角形の頂点数が不正です。");
    }
    const auto view = i_camera.View();
    const auto projection = render::Camera::Projection(i_aspect);
    const auto state = gl::DrawState{};
    if (i_vertices.empty()) { return; }

    // 深度判定を有効にして前後関係を保ち、三角形の頂点を一括転送して描画する。
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glUseProgram(m_instImpl->m_uiProgram);
    glUniformMatrix4fv(m_instImpl->m_iView, 1, GL_FALSE, view.data());
    glUniformMatrix4fv(m_instImpl->m_iProjection, 1, GL_FALSE, projection.data());
    glUniform3fv(m_instImpl->m_iColor, 1, kPolygonColor.data());
    glBindVertexArray(m_instImpl->m_uiVertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_instImpl->m_uiBuffer);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(std::array<float, 3>)), nullptr);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(i_vertices.size() * sizeof(i_vertices.front())),
                 i_vertices.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(i_vertices.size()));
    gl::Check("デバッグポリゴンの描画");
}
}
