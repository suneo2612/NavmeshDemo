/**
 * @file voxel_renderer.h
 * @brief 占有ボクセルを OpenGL インスタンシングで描画する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_VOXEL_RENDERER_H
#define NAVMESH_DEMO_DEMO_NAVMESH_DEBUG_VOXEL_RENDERER_H

#include "voxel_instances.h"
#include <memory>

namespace render { class Camera; }
namespace demo::navmesh::debug
{
/**
 * @brief 単位立方体を共有し、ワールド空間のボクセルを描画する。
 * @details OpenGL コンテキストのあるスレッドで作成・使用・破棄する。
 */
class VoxelRenderer final
{
public:
    /**
     * @brief 描画に必要な OpenGL 資源を作る。
     * @throws std::runtime_error シェーダーまたは資源の作成に失敗した場合。
     */
    VoxelRenderer();
    ~VoxelRenderer();
    VoxelRenderer(const VoxelRenderer&) = delete;
    VoxelRenderer& operator=(const VoxelRenderer&) = delete;

    /**
     * @brief 描画する位置・寸法・色を GPU に登録する。
     * @param i_instances ボクセルの配列。
     * @throws std::length_error 配列が OpenGL の上限を超える場合。
     * @throws std::runtime_error OpenGL の処理が失敗した場合。
     */
    void Upload(const std::vector<VoxelInstance>& i_instances);

    /**
     * @brief 登録済みボクセルを深度判定付きで描画する。
     * @param i_camera カメラ。
     * @param i_aspect 描画領域の横幅と高さの比率。
     * @throws std::runtime_error OpenGL の処理が失敗した場合。
     */
    void Draw(const render::Camera& i_camera, float i_aspect) const;

private:
    struct Impl;                      // OpenGL 資源の実装。
    std::unique_ptr<Impl> m_instImpl;  // 描画資源。
};
}
#endif
