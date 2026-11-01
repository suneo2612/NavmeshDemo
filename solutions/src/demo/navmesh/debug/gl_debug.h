/**
 * @file gl_debug.h
 * @brief デバッグ描画で共通に使う OpenGL 資源と状態を管理する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_GL_DEBUG_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_GL_DEBUG_H

#include <GL/glew.h>

namespace demo::navmesh::debug::gl
{
/**
 * @brief OpenGL エラーを読み取り、失敗を通知する。
 * @param i_stage エラーが発生した処理の名前。
 */
void Check(const char* i_stage);

/**
 * @brief コンパイルしたシェーダーを所有する。
 */
class Shader final
{
public:
    /**
     * @brief シェーダーをコンパイルする。
     * @param i_type シェーダーの種類。
     * @param i_source シェーダーのソース。
     */
    Shader(GLenum i_type, const char* i_source);
    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    /**
     * @brief OpenGL の識別子を返す。
     * @return シェーダー識別子。
     */
    GLuint Id() const noexcept { return m_uiId; }

private:
    GLuint m_uiId = 0; // シェーダーの識別子。
};

/**
 * @brief デバッグ描画で変更する OpenGL の状態を復元する。
 */
class DrawState final
{
public:
    /**
     * @brief 現在の描画状態を保存する。
     */
    DrawState();
    ~DrawState();
    DrawState(const DrawState&) = delete;
    DrawState& operator=(const DrawState&) = delete;

private:
    GLint m_iProgram = 0;     // 元の描画プログラム。
    GLint m_iVertexArray = 0; // 元の頂点配列。
    GLint m_iBuffer = 0;      // 元の頂点バッファー。
    bool m_bDepth = false;    // 元の深度判定。
    bool m_bBlend = false;    // 元の色の合成。
    bool m_bCull = false;     // 元の背面除去。
};
}
#endif
