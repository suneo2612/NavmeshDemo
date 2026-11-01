/**
 * @file bounds_renderer.h
 * @brief 地形とタイルの枠を OpenGL で描画する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_BOUNDS_RENDERER_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_BOUNDS_RENDERER_H

#include "bounds_lines.h"
#include <memory>

namespace render { class Camera; }

namespace demo::navmesh::debug
{
/**
 * @brief 地形とタイルの線分を画面に重ねる描画器。
 * @details OpenGL コンテキストのあるスレッドで作成・使用・破棄する。
 */
class BoundsRenderer final
{
public:
    /**
     * @brief 描画に必要な OpenGL 資源を作る。
     * @throws std::runtime_error シェーダーまたは資源の作成に失敗した場合。
     */
    BoundsRenderer();
    ~BoundsRenderer();
    BoundsRenderer(const BoundsRenderer&) = delete;
    BoundsRenderer& operator=(const BoundsRenderer&) = delete;

    /**
     * @brief 線分を地形の上に重ねて描画する。
     * @param i_lines 地形、タイルとポリゴン接続の線分。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @param i_depthTest 地形や他の面に隠す場合は true。
     * @throws std::runtime_error OpenGL の処理が失敗した場合。
     * @throws std::invalid_argument 頂点数または画面の比率が不正な場合。
     */
    void Draw(const BoundsLines& i_lines, const render::Camera& i_camera, float i_aspect,
              bool i_depthTest = false) const;

    /**
     * @brief 三角形を深度判定付きで地形に重ねる。
     * @param i_vertices 三頂点ごとに一つの三角形。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @throws std::runtime_error OpenGL の処理が失敗した場合。
     * @throws std::invalid_argument 頂点数または画面の比率が不正な場合。
     */
    void DrawTriangles(const std::vector<std::array<float, 3>>& i_vertices,
                       const render::Camera& i_camera, float i_aspect) const;

private:
    struct Impl;                      // OpenGL 資源の実装。
    std::unique_ptr<Impl> m_instImpl;  // 描画資源。
};
}
#endif
