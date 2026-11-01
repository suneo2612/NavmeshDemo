/**
 * @file distance_field.cpp
 * @brief 歩行可能範囲の境界からの距離を計算する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "distance_field.h"
#include "walkable_validation.h"
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace demo::navmesh::voxelization
{
namespace
{
// 距離場の近似尺度。直交方向への移動を 2 とする。
constexpr auto kStraightCost = std::uint32_t{2};
// 距離場の近似尺度。斜め方向への移動を 3 とする。
constexpr auto kDiagonalCost = std::uint32_t{3};
// Direction の先頭に並ぶ直交方向（西・東・北・南）の数。
constexpr auto kCardinalDirections = std::size_t{4};
// 未到達を表す番兵値。距離の加算前にオーバーフローも確認する。
constexpr auto kInfinity = std::numeric_limits<std::uint32_t>::max();
}

/**
 * @brief 歩行できない方向を持つ面を境界として距離場を構築する。
 * @param i_field 八方向の双方向接続を持つ歩行面。
 * @return 対応する面ごとの距離場。境界の距離は 0。
 * @throws std::invalid_argument 歩行面や隣接接続が不正な場合。
 * @throws std::length_error 距離が表現できる範囲を超えた場合。
 * @details 直交四方向のいずれかに移動できない面を境界とし、
 * 接続されている面だけを通って距離を伝播する。複数階層の接続も考慮する。
 */
DistanceField BuildDistanceField(const WalkableField& i_field)
{
    // セル配置と双方向接続を検証し、隣接添字を安全にたどれるようにする。
    ValidateWalkableField(i_field);
    
    // 全面を未到達として初期化する。境界から探索できた面だけが後で有限値になる。
    auto result = DistanceField{};
    result.distances.assign(i_field.spans.size(), kInfinity);

    // 距離と対応するボクセルの組を保持する優先度付きキューを用意する。
    using QueueEntry = std::pair<std::uint32_t, std::size_t>;
    auto queue = std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>{};

    // 直交方向のいずれかに歩行可能な隣接面がない面を境界とし、
    // 距離 0 の探索起点として優先順位付きキューへ入れる。
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        // 四方向のいずれかが途切れているか確認する。
        for (auto direction = std::size_t{}; direction < kCardinalDirections; ++direction)
        {
            // 移動可能な隣接面が存在しない場合は境界とみなす。
            if (i_field.spans[index].neighbors[direction].empty())
            {
                result.distances[index] = 0;   // 距離 0 の境界として登録
                queue.emplace(0, index);
                break;
            }
        }
    }

    // 距離の小さい面から探索し、明示された接続だけを使って直交・斜めのコストを伝播する。
    // 古いキュー項目は現在値と一致しないため捨て、より短い経路が見つかった場合だけ更新する。
    while (!queue.empty())
    {
        const auto [distance, index] = queue.top();     // 最も距離が小さいエントリを取得
        queue.pop();

        // 同じ面がより短い距離で再登録されている場合、以前の長い項目は処理しない。
        if (distance != result.distances[index]) { continue; }
        for (auto direction = std::size_t{}; direction < i_field.spans[index].neighbors.size(); ++direction)
        {
            // Direction の並びに従い、直交移動と斜め移動で近似距離の増分を使い分ける。
            const auto cost = direction < kCardinalDirections ? kStraightCost : kDiagonalCost;

            // 接続先ごとに加算上限を確かめ、新しい経路の距離が短い場合だけキューへ追加する。
            for (const auto target : i_field.spans[index].neighbors[direction])
            {
                if (distance >= kInfinity - cost)
                {
                    throw std::length_error("距離場の値が表現可能な上限を超えました。");
                }

                const auto candidate = distance + cost;
                if (candidate < result.distances[target])
                {
                    result.distances[target] = candidate;
                    queue.emplace(candidate, target);
                }
            }
        }
    }
    for (const auto distance : result.distances)
    {
        // 境界に到達できない成分を最大値のまま返さず、入力の接続不整合として通知する。
        if (distance == kInfinity) { throw std::invalid_argument("境界に到達できない歩行面があります。"); }
    }
    return result;
}
}
