/**
 * @file gl_debug.cpp
 * @brief デバッグ描画用の OpenGL 資源と状態を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "gl_debug.h"
#include <algorithm>
#include <stdexcept>
#include <string>

namespace demo::navmesh::debug::gl
{
/**
 * @brief OpenGL エラーを読み取り、失敗を通知する。
 * @param i_stage エラーが発生した処理の名前。
 * @throws std::runtime_error OpenGL エラーが一つ以上報告された場合。
 */
void Check(const char* i_stage)
{
    // 呼び出し時点でキューにあるエラーを全て読み取り、複数の原因を一度に報告する。
    auto errors = std::string{};
    for (auto error = glGetError(); error != GL_NO_ERROR; error = glGetError())
    {
        if (!errors.empty()) { errors += ", "; }
        errors += std::to_string(error);
    }
    if (!errors.empty()) { throw std::runtime_error(std::string(i_stage) + ": OpenGL エラー " + errors); }
}

/**
 * @brief シェーダーをコンパイルする。
 * @param i_type シェーダーの種類。
 * @param i_source シェーダーのソース。
 * @throws std::runtime_error シェーダーの作成またはコンパイルに失敗した場合。
 */
Shader::Shader(const GLenum i_type, const char* i_source)
{
    // OpenGL オブジェクトを生成してコンパイルし、診断ログを含めて失敗を通知する。
    m_uiId = glCreateShader(i_type);
    if (!m_uiId) { throw std::runtime_error("デバッグ用シェーダーを作成できません。"); }
    glShaderSource(m_uiId, 1, &i_source, nullptr);
    glCompileShader(m_uiId);
    auto success = GLint{};
    glGetShaderiv(m_uiId, GL_COMPILE_STATUS, &success);
    if (success != GL_TRUE)
    {
        auto length = GLint{};
        glGetShaderiv(m_uiId, GL_INFO_LOG_LENGTH, &length);
        auto log = std::string(static_cast<std::size_t>(std::max(length, 1)), '\0');
        glGetShaderInfoLog(m_uiId, length, nullptr, log.data());
        glDeleteShader(m_uiId);
        m_uiId = 0;
        throw std::runtime_error("デバッグ用シェーダーのコンパイルに失敗しました: " + log);
    }
}

/**
 * @brief 所有する OpenGL シェーダーを解放する。
 */
Shader::~Shader() { glDeleteShader(m_uiId); }

/**
 * @brief 現在の描画状態を保存する。
 */
DrawState::DrawState()
{
    // デバッグ描画が変更するバインド状態と有効状態だけを記録する。
    glGetIntegerv(GL_CURRENT_PROGRAM, &m_iProgram);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &m_iVertexArray);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &m_iBuffer);
    m_bDepth = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
    m_bBlend = glIsEnabled(GL_BLEND) == GL_TRUE;
    m_bCull = glIsEnabled(GL_CULL_FACE) == GL_TRUE;
}

/**
 * @brief 保存した OpenGL 描画状態を復元する。
 */
DrawState::~DrawState()
{
    // RAII により正常終了・例外終了のどちらでも呼び出し前の状態へ戻す。
    glUseProgram(static_cast<GLuint>(m_iProgram));
    glBindVertexArray(static_cast<GLuint>(m_iVertexArray));
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(m_iBuffer));
    if (m_bDepth) { glEnable(GL_DEPTH_TEST); } else { glDisable(GL_DEPTH_TEST); }
    if (m_bBlend) { glEnable(GL_BLEND); } else { glDisable(GL_BLEND); }
    if (m_bCull) { glEnable(GL_CULL_FACE); } else { glDisable(GL_CULL_FACE); }
}
}
