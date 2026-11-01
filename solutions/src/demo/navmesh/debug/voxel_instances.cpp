/**
 * @file voxel_instances.cpp
 * @brief タイルの占有区間を描画用ボクセルと距離色に変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "voxel_instances.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kMaxVoxels = std::size_t{2000000};
constexpr auto kUnreached = std::numeric_limits<std::uint32_t>::max();
constexpr auto kTerrainColor = std::array<float, 3>{0.55F, 0.65F, 0.75F};
constexpr auto kUnwalkableColor = std::array<float, 3>{0.35F, 0.45F, 0.55F};
constexpr auto kUnreachedColor = std::array<float, 3>{0.9F, 0.15F, 0.9F};
constexpr auto kErodedColor = std::array<float, 3>{1.0F, 0.0F, 0.0F};
constexpr auto kRegionColors = std::array<std::array<float, 3>, 12>{{
    {0.25F, 0.75F, 0.95F}, {0.95F, 0.65F, 0.20F}, {0.40F, 0.85F, 0.35F},
    {0.80F, 0.45F, 0.85F}, {0.20F, 0.80F, 0.70F}, {0.95F, 0.50F, 0.50F},
    {0.65F, 0.75F, 0.25F}, {0.45F, 0.55F, 0.95F}, {0.85F, 0.75F, 0.50F},
    {0.60F, 0.40F, 0.80F}, {0.30F, 0.70F, 0.50F}, {0.90F, 0.45F, 0.65F}
}};

/**
 * @brief 指定した表示方法の距離配列を返す。
 * @param i_tile 対象のタイル。
 * @param i_mode 距離の表示方法。
 * @param i_afterErosion 侵食後の距離場を使用する場合は true。
 * @return 歩行面に対応する距離。
 */
const std::vector<std::uint32_t>& Distances(const voxelization::VoxelTile& i_tile, const VoxelColorMode i_mode,
                                             const bool i_afterErosion)
{
    // 侵食の前後と平滑化の有無を選び、同じ walkable 面添字で参照する距離配列を返す。
    const auto& distance = i_afterErosion ? i_tile.distance : i_tile.terrainDistance;
    return i_mode == VoxelColorMode::Smoothed ? distance.smoothed.distances : distance.raw.distances;
}

/**
 * @brief ワールド座標でボクセルの両端が表現できるか確認する。
 * @param i_min 最小座標。
 * @param i_size 軸方向の幅。
 */
void ValidateAxis(const double i_min, const double i_size)
{
    // double の加算結果と float への変換後も、正の幅を区別できる座標範囲にあるか検証する。
    const auto maximum = i_min + i_size;
    const auto limit = static_cast<double>(std::numeric_limits<float>::max());
    if (!std::isfinite(i_min) || !std::isfinite(maximum) || i_size <= 0 ||
        i_min < -limit || maximum > limit ||
        static_cast<float>(i_min) >= static_cast<float>(maximum))
    {
        throw std::overflow_error("ボクセルの描画範囲をワールド座標で表現できません。");
    }
}
}

/**
 * @brief 全タイルの所有面から表示用の有限最大距離を求める。
 * @param i_navmesh 構築済みの地形とタイル。
 * @param i_mode 元または平滑化後の距離。
 * @param i_afterErosion 侵食後の距離場を使用する場合は true。
 * @return 有限距離がない場合または全てゼロの場合は 0。
 * @throws std::invalid_argument 距離の種類またはボクセル結果が不正な場合。
 */
std::uint32_t MaximumVoxelDistance(const Navmesh& i_navmesh, const VoxelColorMode i_mode, const bool i_afterErosion)
{
    // 距離表示に使えるモードとタイル結果を検査する。
    if (i_mode != VoxelColorMode::Raw && i_mode != VoxelColorMode::Smoothed)
    {
        throw std::invalid_argument("距離の表示方法が不正です。");
    }
    const auto* voxels = i_navmesh.VoxelTiles();
    if (!voxels) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    auto maximum = std::uint32_t{};
    // 周辺計算面ではなく各タイルが出力を担当する面だけを調べ、色基準を決める。
    for (const auto& tile : voxels->Tiles())
    {
        const auto& distances = Distances(tile, i_mode, i_afterErosion);
        for (const auto span : tile.coreSpans)
        {
            const auto distance = distances.at(span);
            if (distance != kUnreached) { maximum = std::max(maximum, distance); }
        }
    }
    return maximum;
}

/**
 * @brief 所有セルの占有区間だけをボクセルに展開する。
 * @param i_navmesh 構築済みの地形とタイル。
 * @param i_mode 距離の表示方法。
 * @param i_fixedMaximum 指定時はこの値を白の基準に固定する。
 * @param i_showEroded 侵食面を赤色で表示する場合は true。
 * @param i_afterErosion 侵食後の距離場を使用する場合は true。
 * @return 距離に応じて着色したボクセル。
 * @throws std::invalid_argument ボクセル結果が存在しない場合。
 * @throws std::length_error ボクセル数が表示の上限を超える場合。
 * @throws std::overflow_error 描画する座標を表現できない場合。
 * @details 距離表示で基準未指定時は全タイルの所有面における有限最大値で正規化する。
 * 歩行面を持つ占有区間の最上段のみ距離色にし、指定時は侵食面を赤色にする。
 * 侵食後の距離表示または領域表示で赤色を無効にした場合、侵食面は歩行面以外の色で表示する。
 */
std::vector<VoxelInstance> BuildVoxelInstances(const Navmesh& i_navmesh, const VoxelColorMode i_mode,
                                                const std::optional<std::uint32_t> i_fixedMaximum,
                                                const bool i_showEroded, const bool i_afterErosion)
{
    // 色モードとタイル結果を検証し、固定基準がなければ対象距離場の全体最大値を求める。
    if (i_mode != VoxelColorMode::Terrain && i_mode != VoxelColorMode::Raw &&
        i_mode != VoxelColorMode::Smoothed && i_mode != VoxelColorMode::Regions)
    {
        throw std::invalid_argument("ボクセル色の表示方法が不正です。");
    }
    const auto* voxels = i_navmesh.VoxelTiles();
    if (!voxels) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    const auto& tiles = voxels->Tiles();
    const auto& settings = i_navmesh.Settings().raster;
    const auto& bounds = i_navmesh.TerrainBounds();
    const auto maximum = (i_mode == VoxelColorMode::Terrain || i_mode == VoxelColorMode::Regions) ? std::uint32_t{} :
        i_fixedMaximum ? *i_fixedMaximum : MaximumVoxelDistance(i_navmesh, i_mode, i_afterErosion);

    auto result = std::vector<VoxelInstance>{};
    // タイルごとの core セルだけを出力し、重複する計算余白を描画対象にしない。
    for (const auto& tile : tiles)
    {
        const auto& work = tile.region.work;
        const auto& core = tile.region.core;
        const auto width = tile.walkable.width;
        const auto& distances = Distances(tile, i_mode, i_afterErosion);
        for (auto z = core.beginZ; z < core.endZ; ++z)
        {
            for (auto x = core.beginX; x < core.endX; ++x)
            {
                const auto cell = (z - work.beginZ) * width + (x - work.beginX);
                const auto& solids = tile.solidColumns.at(cell);
                auto walk = tile.walkable.offsets.at(cell);
                const auto walkEnd = tile.walkable.offsets.at(cell + 1);
                const auto minX = static_cast<double>(bounds.minimum.x) + x * static_cast<double>(settings.cellSize);
                const auto minZ = static_cast<double>(bounds.minimum.z) + z * static_cast<double>(settings.cellSize);
                const auto sizeX = std::min(static_cast<double>(settings.cellSize), bounds.maximum.x - minX);
                const auto sizeZ = std::min(static_cast<double>(settings.cellSize), bounds.maximum.z - minZ);
                if (!solids.empty())
                {
                    ValidateAxis(minX, sizeX);
                    ValidateAxis(minZ, sizeZ);
                }
                for (auto solidIndex = std::size_t{}; solidIndex < solids.size(); ++solidIndex)
                {
                    const auto& solid = solids[solidIndex];
                    // 初期色は地形または歩行不可色とし、対応する歩行面がある区間だけ表示モードで上書きする。
                    auto color = i_mode == VoxelColorMode::Terrain ? kTerrainColor : kUnwalkableColor;
                    if (walk < walkEnd && tile.walkable.spans.at(walk).sourceIndex == solidIndex)
                    {
                        if (i_showEroded && tile.eroded.at(walk))
                        {
                            color = kErodedColor;
                        }
                        else if ((i_afterErosion || i_mode == VoxelColorMode::Regions) &&
                                 tile.eroded.at(walk) && i_mode != VoxelColorMode::Terrain)
                        {
                            color = kUnwalkableColor;
                        }
                        else if (i_mode == VoxelColorMode::Regions)
                        {
                            const auto id = tile.regions.ids.at(walk);
                            if (id == 0) { throw std::invalid_argument("歩行面に領域が割り当てられていません。"); }
                            const auto& coordinate = tile.region.coordinate;
                            const auto paletteIndex = (id - 1 + coordinate.column * 3 + coordinate.row * 5) %
                                kRegionColors.size();
                            color = kRegionColors[paletteIndex];
                        }
                        else if (i_mode != VoxelColorMode::Terrain)
                        {
                            const auto distance = distances.at(walk);
                            if (distance == kUnreached) { color = kUnreachedColor; }
                            else
                            {
                                const auto value = maximum == 0 ? (distance == 0 ? 0.0F : 1.0F) :
                                    std::min(1.0F, static_cast<float>(distance) / static_cast<float>(maximum));
                                color = {value, value, value};
                            }
                        }
                        ++walk;
                    }
                    const auto count = static_cast<std::size_t>(solid.maximum - solid.minimum);
                    // 表示上限を超える前に区間全体のボクセル数を確認し、途中までの結果を返さない。
                    if (count > kMaxVoxels - result.size())
                    {
                        throw std::length_error("表示するボクセル数が上限を超えました。");
                    }
                    for (auto y = solid.minimum; y < solid.maximum; ++y)
                    {
                        // 最上段だけを床面色にし、区間内部は地形色または歩行不可色で積み上げる。
                        const auto minY = static_cast<double>(bounds.minimum.y) +
                            static_cast<double>(y) * settings.cellHeight;
                        ValidateAxis(minY, settings.cellHeight);
                        const auto voxelColor = y + 1 == solid.maximum ? color :
                            (i_mode == VoxelColorMode::Terrain ? kTerrainColor : kUnwalkableColor);
                        result.push_back({{static_cast<float>(minX), static_cast<float>(minY), static_cast<float>(minZ)},
                                          {static_cast<float>(sizeX), settings.cellHeight, static_cast<float>(sizeZ)},
                                          voxelColor});
                    }
                }
            }
        }
    }
    if (result.empty()) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    return result;
}
}
