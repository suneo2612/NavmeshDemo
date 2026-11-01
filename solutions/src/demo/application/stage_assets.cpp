/**
 * @file stage_assets.cpp
 * @brief ステージモデルの読み込みと GPU 登録を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "stage_assets.h"
#include "general/util/path_util.h"
#include "loader/loader_factory.h"
#include "render/renderer.h"
#include <stdexcept>
#include <utility>

namespace demo::application
{
namespace
{
/**
 * @brief FBX を読み込み、データを保持するローダーを返す。
 * @param i_path ステージファイルのパス。
 * @return 読み込み済みローダー。
 * @throws std::runtime_error 読み込みに失敗した場合。
 */
std::unique_ptr<loader::LoaderBase> ReadStage(const std::filesystem::path& i_path)
{
    // ローダーが要求する UTF-8 パスに変換し、対応する形式のローダーを作成する。
    const auto path = general::util::PathToUtf8(i_path);
    auto loader = loader::LoaderFactory::CreateLoader(path);
    if (!loader) { throw std::runtime_error("対応するモデルローダーがありません: " + path); }

    // 読み込みに失敗した場合は理由を伝え、成功したデータだけを返す。
    if (!loader->Load(path)) { throw std::runtime_error(loader->GetLastError()); }
    return loader;
}
}

std::shared_ptr<const render::Model> LoadStage(const std::filesystem::path& i_path)
{
    auto loader = ReadStage(i_path);
    return std::make_shared<render::Model>(loader->GetModelData());
}

std::shared_ptr<const render::Model> LoadStageWithNavmesh(const std::filesystem::path& i_path,
    const general::math::Matrix4& i_world, const navmesh::NavmeshSettings& i_settings,
    std::optional<navmesh::Navmesh>& io_navmesh)
{
    auto loader = ReadStage(i_path);
    // 描画と構築結果の両方を準備し、片方の失敗で前のステージが失われないようにする。
    auto candidate = [&]
    {
        try { return navmesh::CreateNavmesh(loader->GetModelData(), i_world, i_settings); }
        catch (const std::invalid_argument& i_error) { throw std::runtime_error(i_error.what()); }
        catch (const std::length_error& i_error) { throw std::runtime_error(i_error.what()); }
    }();
    auto model = std::make_shared<render::Model>(loader->GetModelData());
    io_navmesh = std::move(candidate);
    return model;
}
}
