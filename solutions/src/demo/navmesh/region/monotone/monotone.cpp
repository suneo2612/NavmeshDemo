/**
 * @file monotone.cpp
 * @brief 歩行面の行を走査して単調領域を構築する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "monotone.h"
#include "../../voxelization/walkable_validation.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::region
{
namespace
{
constexpr auto kWest = static_cast<std::size_t>(voxelization::Direction::West);   // X 方向の連続列を遡る西側の接続方向。
constexpr auto kEast = static_cast<std::size_t>(voxelization::Direction::East);   // 西側接続の対称性を確認する東側の接続方向。
constexpr auto kNorth = static_cast<std::size_t>(voxelization::Direction::North); // 隣接する Z 行の run を結ぶ接続方向。
constexpr auto kNoRun = std::numeric_limits<std::size_t>::max();                  // 歩行面がまだどの run にも属さないことを示す値。

/**
 * @brief X 方向に連続する歩行面の列。
 */
struct Run
{
    std::vector<std::size_t> spans;    // この列を構成する歩行面。
    std::vector<std::size_t> parents;  // 前の Z 行に接続する列。
    std::vector<std::size_t> children; // 次の Z 行に接続する列。
    std::uint32_t id = 0;              // 確定した領域の識別子。
};

/**
 * @brief 配列にない値だけを追加する。
 * @param io_values 重複のない配列。
 * @param i_value 追加する値。
 */
void AddUnique(std::vector<std::size_t>& io_values, const std::size_t i_value)
{
    // 小さな隣接一覧の重複を避け、分岐数を正しく数えられるようにする。
    if (std::find(io_values.begin(), io_values.end(), i_value) == io_values.end())
    {
        io_values.push_back(i_value);
    }
}
}

/**
 * @brief 所有範囲の侵食されていない歩行面を単調領域に分割する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @return 面と同じ添字を持つタイル内の領域 ID。0 は非対象。
 * @throws std::invalid_argument 面、接続または侵食結果が不正な場合。
 * @throws std::length_error 領域数が上限を超える場合。
 * @details X 方向の連続列を Z 行ごとに作り、前の行と一対一のときだけ継続する。
 * 分岐や合流では新しい領域とするため、各行は一つの連続列に収まる。
 */
MonotoneRegions BuildMonotoneRegions(const voxelization::WalkableField& i_field,
                                     const std::vector<std::size_t>& i_coreSpans,
                                     const std::vector<std::uint8_t>& i_eroded)
{
    // 面の接続構造を検証し、侵食マスクが全ての面に対応することを確かめる。
    voxelization::ValidateWalkableField(i_field);
    if (i_eroded.size() != i_field.spans.size() ||
        std::any_of(i_eroded.begin(), i_eroded.end(), [](const auto i_value) { return i_value > 1; }))
    {
        throw std::invalid_argument("単調分割の侵食結果が不正です。");
    }

    // 所有面だけを対象にし、侵食された面は分割対象から外す。
    auto active = std::vector<std::uint8_t>(i_field.spans.size());
    auto seen = std::vector<std::uint8_t>(i_field.spans.size());
    for (const auto index : i_coreSpans)
    {
        if (index >= i_field.spans.size() || seen[index])
        {
            throw std::invalid_argument("単調分割の所有面が不正です。");
        }
        seen[index] = 1;
        if (!i_eroded[index]) { active[index] = 1; }
    }

    // X 方向の接続をたどって行ごとの連続列を作る。
    // 前の面に西隣が一つあり、その面から見た東隣も一つである場合だけ同じ列を継続する。
    // 分岐や複数候補がある箇所では列を分け、後の領域割り当てで曖昧さを持ち越さない。
    auto runs = std::vector<Run>{};
    auto spanRun = std::vector<std::size_t>(i_field.spans.size(), kNoRun);
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        if (!active[index]) { continue; }
        auto previous = kNoRun;
        auto westCount = std::size_t{};
        // 西側の活動面が一つだけか確認する。複数あれば一意な継続元を選べない。
        for (const auto target : i_field.spans[index].neighbors[kWest])
        {
            if (active[target]) { previous = target; ++westCount; }
        }
        auto eastCount = std::size_t{};
        if (westCount == 1)
        {
            // 逆方向の接続も一つであることを要求し、片方向だけの接続や合流を列の継続と誤認しない。
            for (const auto target : i_field.spans[previous].neighbors[kEast])
            {
                if (active[target]) { ++eastCount; }
            }
        }
        // 直前の面がすでに属する列を持つときだけ、その列 ID を引き継いで一本の run にまとめる。
        if (westCount == 1 && eastCount == 1 && spanRun[previous] != kNoRun)
        {
            spanRun[index] = spanRun[previous];
        }
        else
        {
            spanRun[index] = runs.size();
            runs.emplace_back();
        }
        runs[spanRun[index]].spans.push_back(index);
    }

    // Z 方向の接続から隣接行の run 間の親子関係を記録し、次段階で分岐・合流の有無を判定する。
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        if (!active[index]) { continue; }
        for (const auto target : i_field.spans[index].neighbors[kNorth])
        {
            if (!active[target]) { continue; }
            // North 接続先を前行、現在面を次行として記録し、行をまたぐ run の対応を明示する。
            const auto child = spanRun[index];
            const auto parent = spanRun[target];
            AddUnique(runs[child].parents, parent);
            AddUnique(runs[parent].children, child);
        }
    }
    auto result = MonotoneRegions{};
    result.ids.resize(i_field.spans.size());
    // 前行と一対一で続く run だけが領域 ID を継承する。分岐・合流・新規開始は別領域とする。
    for (auto index = std::size_t{}; index < runs.size(); ++index)
    {
        auto& run = runs[index];
        if (run.parents.size() == 1 && runs[run.parents.front()].children.size() == 1)
        {
            run.id = runs[run.parents.front()].id;
        }
        else
        {
            if (result.count == std::numeric_limits<std::uint32_t>::max())
            {
                throw std::length_error("単調領域の数が上限を超えました。");
            }
            run.id = ++result.count;
        }
        for (const auto span : run.spans) { result.ids[span] = run.id; }
    }
    return result;
}
}
