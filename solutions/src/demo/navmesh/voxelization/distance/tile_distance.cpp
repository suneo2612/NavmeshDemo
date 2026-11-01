/**
 * @file tile_distance.cpp
 * @brief タイルの距離を前後の走査と九点平均で求める。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "tile_distance.h"
#include "../walkable_validation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::voxelization
{
namespace
{
constexpr auto kInfinity = std::numeric_limits<std::uint32_t>::max();
constexpr auto kStraightCost = std::uint32_t{2};
constexpr auto kDiagonalCost = std::uint32_t{3};
constexpr auto kCardinalCount = std::size_t{4};
constexpr auto kDirections = std::size_t{8};
constexpr auto kAverageCount = std::uint64_t{9};
constexpr auto kRounding = std::uint64_t{4};
constexpr auto kForward = std::array<std::size_t, 4>{0, 2, 4, 5};
constexpr auto kBackward = std::array<std::size_t, 4>{1, 3, 6, 7};

/**
 * @brief 指定面の接続先から小さい距離を取り込む。
 * @param i_field 歩行面と接続。
 * @param i_managed 管理範囲を示す面ごとの値。
 * @param i_index 更新する面。
 * @param i_directions 走査方向で参照する方向。
 * @param io_distances 更新する距離配列。
 */
void Relax(const WalkableField& i_field, const std::vector<std::uint8_t>& i_managed,
           const std::size_t i_index, const std::array<std::size_t, 4>& i_directions,
           std::vector<std::uint32_t>& io_distances)
{
    // 指定された走査方向の隣接面を調べ、管理対象かつ到達済みの距離だけを候補にする。
    for (const auto direction : i_directions)
    {
        // 直交移動と斜め移動のコストを分け、短い経路が現在値より小さい場合だけ更新する。
        const auto cost = direction < kCardinalCount ? kStraightCost : kDiagonalCost;
        for (const auto target : i_field.spans[i_index].neighbors[direction])
        {
            if (!i_managed[target] || io_distances[target] == kInfinity) { continue; }
            if (io_distances[target] >= kInfinity - cost)
            {
                throw std::length_error("距離場の値が表現可能な上限を超えました。");
            }
            io_distances[i_index] = std::min(io_distances[i_index], io_distances[target] + cost);
        }
    }
}
}

/**
 * @brief 管理範囲の歩行面を前後に走査して距離を求め、平滑化する。
 * @param i_field 管理範囲の外側にも接続判定用の面を持つ歩行面。
 * @param i_managed 各面が管理範囲内なら 1、外側なら 0。
 * @param i_cellSize XZ セルの幅。
 * @param i_agentRadius キャラクターの半径。0 以上。
 * @return 入力面に対応する距離場。
 * @throws std::invalid_argument 入力が不正、または距離伝播に矛盾がある場合。
 * @throws std::length_error 距離が表現できる範囲を超えた場合。
 * @details 管理範囲外の接続先は距離伝播に使わないが、境界の誤判定を防ぐため
 * 直交方向の接続の有無には使用する。
 */
TileDistance BuildTileDistance(const WalkableField& i_field,
                           const std::vector<std::uint8_t>& i_managed,
                           const float i_cellSize, const float i_agentRadius)
{
// 配列間の添字対応、寸法、管理マスクを検証してから距離配列を作る。
ValidateWalkableField(i_field);
if (i_managed.size() != i_field.spans.size() ||
    !std::isfinite(i_cellSize) || i_cellSize <= 0 ||
        !std::isfinite(i_agentRadius) || i_agentRadius < 0 ||
        std::any_of(i_managed.begin(), i_managed.end(), [](const auto value) { return value > 1; }))
    {
        throw std::invalid_argument("タイルの距離場の範囲または寸法が不正です。");
    }

    // 未到達を無限大で初期化し、管理範囲内で直交接続が欠けている面だけを境界距離 0 にする。
    // 管理範囲外の余白面も接続判定に使うため、タイル端そのものを境界とは誤認しない。
    auto result = TileDistance{};
    result.raw.distances.assign(i_field.spans.size(), kInfinity);
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        if (!i_managed[index]) { continue; }
        for (auto direction = std::size_t{}; direction < kCardinalCount; ++direction)
        {
            if (i_field.spans[index].neighbors[direction].empty())
            {
                result.raw.distances[index] = 0;
                break;
            }
        }
    }

    // 面の並びに沿って前方、後方の二方向から距離を緩和し、境界からの距離を伝播する。
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        if (i_managed[index]) { Relax(i_field, i_managed, index, kForward, result.raw.distances); }
    }
    for (auto index = i_field.spans.size(); index > 0; --index)
    {
        if (i_managed[index - 1]) { Relax(i_field, i_managed, index - 1, kBackward, result.raw.distances); }
    }

    // 到達済み最大距離を記録し、隣接面間に到達状態の食い違いがあれば走査不足として失敗させる。
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        if (!i_managed[index]) { continue; }
        const auto distance = result.raw.distances[index];
        if (distance != kInfinity && (!result.maximum || distance > *result.maximum))
        {
            result.maximum = distance;
        }
        for (const auto& neighbors : i_field.spans[index].neighbors)
        {
            for (const auto target : neighbors)
            {
                if (i_managed[target] && ((distance == kInfinity) != (result.raw.distances[target] == kInfinity)))
                {
                    throw std::invalid_argument("距離場の走査で接続先への距離が伝播しませんでした。");
                }
            }
        }
    }

    // 平滑化は元の距離値を参照して別配列へ計算する。
    // 境界そのものと、キャラクター半径に近い面は値を変えず安全余白を保つ。
    result.smoothed = result.raw;
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        const auto current = result.raw.distances[index];
        if (!i_managed[index] || current == kInfinity || current == 0 ||
            static_cast<double>(current) * i_cellSize < static_cast<double>(i_agentRadius) * kStraightCost)
        {
            continue;
        }
        auto sum = static_cast<std::uint64_t>(current);
        for (auto direction = std::size_t{}; direction < kDirections; ++direction)
        {
            auto neighborDistance = kInfinity;
            for (const auto target : i_field.spans[index].neighbors[direction])
            {
                if (i_managed[target])
                {
                    neighborDistance = std::min(neighborDistance, result.raw.distances[target]);
                }
            }
            sum += neighborDistance == kInfinity ? current : neighborDistance;
        }
        // 8 近傍の距離と自身の距離を等しく平均し、整数除算は定数分を加えて丸める。
        result.smoothed.distances[index] = static_cast<std::uint32_t>((sum + kRounding) / kAverageCount);
    }
    return result;
}
}
