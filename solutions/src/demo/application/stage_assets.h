/**
 * @file stage_assets.h
 * @brief ステージモデルの読み込みを扱う。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_APPLICATION_STAGE_ASSETS_H
#define NAVMESH_DEMO_DEMO_APPLICATION_STAGE_ASSETS_H

#include <filesystem>
#include <memory>
#include <optional>
#include "demo/navmesh/navmesh.h"

namespace render { class Model; }
namespace general::math { class Matrix4; }

namespace demo::application
{
/**
 * @brief FBX を読み込み、GPU 資源まで作成して返す。
 * @param i_path FBX のパス。
 * @return 他の Entity と共有できるモデル。
 * @throws std::runtime_error ローダーの作成、読み込みまたは GPU 登録に失敗した場合。
 * @details 有効な OpenGL コンテキストが必要。
 */
std::shared_ptr<const render::Model> LoadStage(const std::filesystem::path& i_path);

/**
 * @brief 描画モデルと Navmesh の現在の構築結果を同じ FBX から準備する。
 * @param i_path FBX のパス。
 * @param i_world フィールド Entity のワールド変換。
 * @param i_settings Navmesh の構築設定。
 * @param io_navmesh 成功時に更新する Navmesh。
 * @return 他の Entity と共有できる描画モデル。
 * @throws std::runtime_error 読み込み、地形の変換または GPU 登録に失敗した場合。
 * @details 有効な OpenGL コンテキストが必要。失敗時は既存の Navmesh を保持する。
 */
std::shared_ptr<const render::Model> LoadStageWithNavmesh(const std::filesystem::path& i_path,
    const general::math::Matrix4& i_world, const navmesh::NavmeshSettings& i_settings,
    std::optional<navmesh::Navmesh>& io_navmesh);
}
#endif
