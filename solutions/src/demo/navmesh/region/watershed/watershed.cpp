/**
 * @file watershed.cpp
 * @brief 距離の極大部から歩行面へ領域を広げる。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "watershed.h"
#include "../../voxelization/walkable_validation.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>

namespace demo::navmesh::region
{
namespace
{
constexpr auto kCardinalDirections = std::size_t{4};                    // 平坦部と領域の拡張で調べる東西南北の方向数。
constexpr auto kUnreached = std::numeric_limits<std::uint32_t>::max();  // 距離場で境界から到達できない面を示す値。

/**
 * @brief 領域を広げる候補。
 */
struct Candidate
{
    std::uint32_t distance; // 対象面の距離。最大値は未到達。
    std::size_t span;       // 対象面の添字。
    std::uint32_t region;   // 広げ元の領域 ID。
};

/**
 * @brief 有限距離の高い候補を優先し、同値は添字と ID で安定させる。
 */
struct CandidateOrder
{
    /**
     * @brief 優先度付きキューで距離の高い候補を先に取り出す。
     * @param i_a 比較する候補。
     * @param i_b 比較する候補。
     * @return i_a の優先度が i_b より低い場合は true。
     */
    bool operator()(const Candidate& i_a, const Candidate& i_b) const noexcept
    {
        // 有限距離を未到達より先にし、距離が同じ場合も添字と領域 ID で順序を固定する。
        const auto aFinite = i_a.distance != kUnreached;
        const auto bFinite = i_b.distance != kUnreached;
        if (aFinite != bFinite) { return !aFinite; }
        if (aFinite && i_a.distance != i_b.distance) { return i_a.distance < i_b.distance; }
        if (i_a.span != i_b.span) { return i_a.span > i_b.span; }
        return i_a.region > i_b.region;
    }
};
}

/**
 * @brief 侵食後の距離場の極大部から所有面を流域に分割する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_distance 面ごとの平滑化後の距離。最大整数値は未到達。
 * @return 面と同じ添字を持つタイル内の領域 ID。0 は非対象。
 * @throws std::invalid_argument 入力や未到達面の接続が不正な場合。
 * @throws std::length_error 領域数が上限を超えた場合。
 * @details
 * 四方向接続で同じ距離の平坦部をまとめ、周囲に高い距離がない平坦部を
 * 始点として高い距離から順に広げる。競合は添字と領域 ID の順で決める。
 * 未到達の接続成分は有限距離の高さと比較せず、平坦な独立領域として扱う。
 * 一つの領域に同じ XZ セルの複数階層を含めない。
 */
Regions BuildWatershedRegions(const voxelization::WalkableField& i_field,
                               const std::vector<std::size_t>& i_coreSpans,
                               const std::vector<std::uint8_t>& i_eroded,
                               const voxelization::DistanceField& i_distance)
{
    // 距離場と侵食マスクを面配列に対応させ、以後の隣接探索で参照する添字を安全にする。
    voxelization::ValidateWalkableField(i_field);

    // 侵食状態と距離値が歩行面ごとに一つずつ存在し、侵食状態が 0 / 1 であることを確認する。
    // 配列長や値域が合わないまま探索すると、面との対応を誤って領域を割り当てるため拒否する。
    if (i_eroded.size() != i_field.spans.size() ||
        i_distance.distances.size() != i_field.spans.size() ||
        std::any_of(
            i_eroded.begin(),
            i_eroded.end(),
            [](const auto i_value) { return i_value > 1; }
        )
    )
    {
        throw std::invalid_argument("流域分割の距離場または侵食結果が不正です。");
    }

    // 所有面だけを活動対象として印を付ける。seen は所有添字の重複検出にも使い、
    // 侵食面は後続の平坦部探索と領域拡張から外す。
    auto active = std::vector<std::uint8_t>(i_field.spans.size());
    auto seen = std::vector<std::uint8_t>(i_field.spans.size());
    for (const auto index : i_coreSpans)
    {
        if (index >= i_field.spans.size() || seen[index])
        {
            throw std::invalid_argument("流域分割の所有面が不正です。");
        }
        seen[index] = 1;
        if (!i_eroded[index]) { active[index] = 1; }
    }
    auto result = Regions{};
    result.ids.resize(i_field.spans.size());
    // 各 XZ 列で領域 ID が使用済みか記録し、同じ領域が同一列の複数階層を占有するのを防ぐ。
    auto owners = std::vector<std::vector<std::uint32_t>>(i_field.width * i_field.depth);
    // 領域の境界から未割り当て面へ広がる順序を距離と添字で決定する。
    auto candidates = std::priority_queue<Candidate, std::vector<Candidate>, CandidateOrder>{};

    // 面を領域に割り当てるとき、同じ XZ 列に同じ領域が既にあれば割り当てを拒否する。
    // 別の領域 ID なら同じ列の別階層を占有できるため、階層ごとの通路は維持される。
    const auto claim = [&](const std::size_t i_index, const std::uint32_t i_id)
    {
        const auto& span = i_field.spans[i_index];
        auto& column = owners[span.z * i_field.width + span.x];
        if (std::find(column.begin(), column.end(), i_id) != column.end()) { return false; }
        column.push_back(i_id);
        result.ids[i_index] = i_id;
        return true;
    };
    // 領域 ID の上限を確認し、次の未使用 ID を発行する。
    const auto newId = [&]()
    {
        if (result.count == std::numeric_limits<std::uint32_t>::max())
        {
            throw std::length_error("流域の数が上限を超えました。");
        }
        return ++result.count;
    };
    // 現在の領域に属する面の四方向を調べ、活動中かつ未割り当ての隣接面を拡張候補にする。
    // 同じ面が複数回候補に入ることは許し、取り出し時に割り当て済みかを再確認する。
    const auto enqueue = [&](const std::size_t i_index, const std::uint32_t i_id)
    {
        for (auto direction = std::size_t{}; direction < kCardinalDirections; ++direction)
        {
            for (const auto target : i_field.spans[i_index].neighbors[direction])
            {
                if (active[target] && !result.ids[target])
                {
                    candidates.push({i_distance.distances[target], target, i_id});
                }
            }
        }
    };

    // 四方向接続で同じ距離の面を一つの平坦部として集め、隣接するより高い面の有無を調べる。
    // 未到達値同士は平坦部として扱うが、未到達面と有限距離面がつながる入力は不整合として拒否する。
    auto visited = std::vector<std::uint8_t>(i_field.spans.size());     // 同じ距離の平坦部を探索済みとして記録し、重複探索を防ぐ。
    auto plateau = std::vector<std::size_t>{};                          // 四方向に連結した同距離面を集め、平坦部の極大判定に使う。
    // 未訪問の活動面を起点に、同じ距離でつながる成分を一度ずつ調べる。
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        if (!active[index] || visited[index]) { continue; }
        // 起点の距離をこの平坦部の基準値とし、探索中に高い隣接面が見つかったか記録する。
        plateau.clear();
        plateau.push_back(index);
        visited[index] = 1;
        const auto level = i_distance.distances[index];
        auto hasHigher = false;
        // plateau は探索中に同距離の隣接面が追加されるため、末尾まで処理して成分全体を走査する。
        for (auto position = std::size_t{}; position < plateau.size(); ++position)
        {
            for (auto direction = std::size_t{}; direction < kCardinalDirections; ++direction)
            {
                for (const auto target : i_field.spans[plateau[position]].neighbors[direction])
                {
                    if (!active[target]) { continue; }
                    const auto other = i_distance.distances[target];
                    // 未到達値を最大の有限距離として比較してしまわないよう、種類の違いを先に検出する。
                    if ((level == kUnreached) != (other == kUnreached))
                    {
                        throw std::invalid_argument("未到達の面が有限距離の面と接続しています。");
                    }
                    // 有限距離の成分に、より高い面が一つでも接していれば、この成分は種にしない。
                    if (level != kUnreached && other > level) { hasHigher = true; }
                    if (other == level && !visited[target])
                    {
                        // 追加時に訪問済みにし、別の隣接面から重ねてキューへ入らないようにする。
                        visited[target] = 1;
                        plateau.push_back(target);
                    }
                }
            }
        }
        // より高い隣接面を持つ成分は、後でその高い側から領域が広がるため種の作成を省く。
        if (hasHigher) { continue; }
        // 同じ極大平坦部に複数の種を作る場合でも、面添字順で領域 ID が決まるように整列する。
        std::sort(plateau.begin(), plateau.end());
        auto queue = std::vector<std::size_t>{};
        // 極大平坦部を添字順に走査し、同じ高さで接続する未割り当て面を一つの領域にする。
        for (const auto seed : plateau)
        {
            // 先行する種の探索が割り当てた面は再利用せず、未割り当ての成分だけ新しい ID を発行する。
            if (result.ids[seed]) { continue; }
            const auto id = newId();
            static_cast<void>(claim(seed, id));
            queue.clear();
            queue.push_back(seed);
            // 種から同じ距離で接続する面を幅優先で集め、平坦部内の各連結成分を一つの領域にする。
            for (auto position = std::size_t{}; position < queue.size(); ++position)
            {
                const auto current = queue[position];
                // 同じ高さの隣接面だけを種の領域に含める。claim は同一 XZ 列の重複階層を拒否する。
                for (auto direction = std::size_t{}; direction < kCardinalDirections; ++direction)
                {
                    for (const auto target : i_field.spans[current].neighbors[direction])
                    {
                        if (active[target] && !result.ids[target] && i_distance.distances[target] == level &&
                            claim(target, id))
                        {
                            queue.push_back(target);
                        }
                    }
                }
            }
            // 極大部に割り当てた全ての面から隣接する低い面を候補化し、外側への拡張を始める。
            for (const auto span : queue) { enqueue(span, id); }
        }
    }

    // 極大部を起点に優先キューを処理し、距離の高い面から各領域へ割り当てる。
    // 候補が別階層による列の占有で拒否されても、その面が未割り当てなら独立した種として扱う。
    for (auto seed = std::size_t{}; seed < i_field.spans.size(); ++seed)
    {
        if (!active[seed] || result.ids[seed]) { continue; }
        while (!candidates.empty())
        {
            const auto candidate = candidates.top();
            candidates.pop();
            // 他の領域が先に割り当てた候補、または同じ領域の別階層となる候補はここで除外する。
            if (result.ids[candidate.span] || !claim(candidate.span, candidate.region)) { continue; }
            enqueue(candidate.span, candidate.region);
        }
        if (!result.ids[seed])
        {
            const auto id = newId();
            static_cast<void>(claim(seed, id));
            enqueue(seed, id);
        }
    }
    // 最後に作られた種から追加された候補も同じ規則で処理し、活動中の面を割り当て切る。
    while (!candidates.empty())
    {
        const auto candidate = candidates.top();
        candidates.pop();
        if (result.ids[candidate.span] || !claim(candidate.span, candidate.region)) { continue; }
        enqueue(candidate.span, candidate.region);
    }
    return result;
}

}   // namespace demo::navmesh::region::watershed
