/**
 * @file filtered_walkable.cpp
 * @brief 侵食面への接続と、角を抜ける斜め接続を取り除く。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "filtered_walkable.h"
#include "../walkable_validation.h"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace demo::navmesh::voxelization
{
namespace
{
constexpr auto kCardinalCount = std::size_t{4};
constexpr auto kDiagonalRoutes = std::array<std::array<std::size_t, 2>, 4>{
    std::array<std::size_t, 2>{0, 2}, {1, 2}, {0, 3}, {1, 3}};

/**
 * @brief 二つの直交方向で対象面に到達できるか確認する。
 * @param i_field 直交接続を更新した歩行面。
 * @param i_from 開始面。
 * @param i_to 対象面。
 * @param i_first 最初の方向。
 * @param i_second 次の方向。
 * @return 接続が残る場合は true。
 */
bool HasPath(const WalkableField& i_field, const std::size_t i_from, const std::size_t i_to,
             const std::size_t i_first, const std::size_t i_second)
{
    // 最初の方向へ進んだ各中間面から次の方向へ進み、目的面への接続があるか調べる。
    for (const auto middle : i_field.spans[i_from].neighbors[i_first])
    {
        const auto& targets = i_field.spans[middle].neighbors[i_second];
        if (std::find(targets.begin(), targets.end(), i_to) != targets.end()) { return true; }
    }
    return false;
}
}

/**
 * @brief 侵食面の接続を除き、残った面の斜め接続を再確認する。
 * @param i_field 侵食前の歩行面と接続。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @return 添字と面を維持し、侵食面の接続を除いた歩行面。侵食面は配列に残る。
 * @throws std::invalid_argument 入力が不正な場合。
 * @details 侵食面を除いた後に直交二方向の経路が両方とも成立しない斜め接続を削除し、
 * 障害物の角を斜めに通り抜ける接続が残らないようにする。
 */
WalkableField FilterErodedWalkable(const WalkableField& i_field, const std::vector<std::uint8_t>& i_eroded)
{
    // 入力面と侵食マスクの添字対応を確認してから、元データを複製して編集する。
    ValidateWalkableField(i_field);
    if (i_eroded.size() != i_field.spans.size() ||
        std::any_of(i_eroded.begin(), i_eroded.end(), [](const auto i_value) { return i_value > 1; }))
    {
        throw std::invalid_argument("歩行面と侵食結果の対応が不正です。");
    }
    auto result = i_field;
    // 侵食された面の接続を全方向から消し、未侵食面から侵食面への接続も取り除く。
    for (auto index = std::size_t{}; index < result.spans.size(); ++index)
    {
        for (auto& neighbors : result.spans[index].neighbors)
        {
            if (i_eroded[index]) { neighbors.clear(); }
            else
            {
                neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
                    [&](const auto i_target) { return i_eroded[i_target] != 0; }), neighbors.end());
            }
        }
    }
    // 侵食後の面だけで直交二方向の経路を両順序から確認し、
    // どちらかの経路が欠ける斜め接続を除いて障害物の角抜けを防ぐ。
    for (auto index = std::size_t{}; index < result.spans.size(); ++index)
    {
        for (auto direction = kCardinalCount; direction < result.spans[index].neighbors.size(); ++direction)
        {
            const auto route = kDiagonalRoutes[direction - kCardinalCount];
            auto& neighbors = result.spans[index].neighbors[direction];
            neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(), [&](const auto i_target)
            {
                return !HasPath(result, index, i_target, route[0], route[1]) ||
                       !HasPath(result, index, i_target, route[1], route[0]);
            }), neighbors.end());
        }
    }
    return result;
}
}
