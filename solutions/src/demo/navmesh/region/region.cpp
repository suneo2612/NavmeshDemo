/**
 * @file region.cpp
 * @brief 領域分割の共通設定を検証する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "region.h"
#include "../voxelization/walkable_validation.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::region
{
/**
 * @brief 分割方式が対応しているか検証する。
 * @param i_method 確認する分割方式。
 * @throws std::invalid_argument 対応していない場合。
 */
void ValidatePartitionMethod(const PartitionMethod i_method)
{
    // 実装済みの方式だけを受け付け、後続処理で未対応方式が暗黙に扱われるのを防ぐ。
    if (i_method != PartitionMethod::Monotone && i_method != PartitionMethod::Watershed)
    {
        throw std::invalid_argument("領域の分割方式が不正です。");
    }
}

/**
 * @brief 所有面と領域の被覆、階層の重複および四方向の接続を検証する。
 * @param i_field 侵食後の歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 面ごとの領域。
 * @return 領域 ID を添字とする面数。添字 0 は未使用。
 * @throws std::invalid_argument 入力や領域が不正な場合。
 */
std::vector<std::size_t> ValidateRegions(const voxelization::WalkableField& i_field,
                                         const std::vector<std::size_t>& i_coreSpans,
                                         const std::vector<std::uint8_t>& i_eroded,
                                         const Regions& i_regions)
{
    // 面データ・領域 ID・侵食マスクの長さと値域を検査する。
    voxelization::ValidateWalkableField(i_field);
    if (i_regions.ids.size() != i_field.spans.size() ||
        i_eroded.size() != i_field.spans.size() ||
        i_regions.count > i_field.spans.size() ||
        std::any_of(i_eroded.begin(), i_eroded.end(), [](const auto i_value) { return i_value > 1; }))
    {
        throw std::invalid_argument("領域または侵食結果が不正です。");
    }

    // 所有面の一覧をマスクに変換し、範囲外や重複した添字を拒否する。
    auto core = std::vector<std::uint8_t>(i_field.spans.size());
    for (const auto index : i_coreSpans)
    {
        if (index >= core.size() || core[index]) { throw std::invalid_argument("領域の所有面が不正です。"); }
        core[index] = 1;
    }

    // セルごとに面を走査し、対象の所有面だけに有効な領域 ID が付いていることを確認する。
    // lastColumn は同じ領域が同一セルの複数階層を占有していないかを検出する。
    auto counts = std::vector<std::size_t>(static_cast<std::size_t>(i_regions.count) + 1);
    auto lastColumn = std::vector<std::size_t>(counts.size(), std::numeric_limits<std::size_t>::max());
    for (auto cell = std::size_t{}; cell < i_field.width * i_field.depth; ++cell)
    {
        for (auto index = i_field.offsets[cell]; index < i_field.offsets[cell + 1]; ++index)
        {
            const auto id = i_regions.ids[index];
            if (id > i_regions.count || (id != 0) != static_cast<bool>(core[index] && !i_eroded[index]))
            {
                throw std::invalid_argument("所有面と領域が一致しません。");
            }
            if (!id) { continue; }
            if (lastColumn[id] == cell) { throw std::invalid_argument("同じ領域の階層がセル内で重なっています。"); }
            lastColumn[id] = cell;
            ++counts[id];
        }
    }
    if (std::any_of(counts.begin() + 1, counts.end(), [](const auto i_count) { return i_count == 0; }))
    {
        throw std::invalid_argument("領域番号が連続していません。");
    }

    // 各領域の未訪問面から四方向探索を開始する。領域 ID が複数の連結成分に分かれていれば、
    // 最初の成分で到達した面数が先に数えた領域全体の面数に届かないため、ここで不整合を検出できる。
    auto seen = std::vector<std::uint8_t>(i_field.spans.size());
    auto queue = std::vector<std::size_t>{};
    constexpr auto cardinalCount = std::size_t{4};
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        const auto id = i_regions.ids[index];
        if (!id || seen[index]) { continue; }

        // 未訪問の領域面を探索の始点にし、面添字を幅優先キューへ登録する。
        queue.clear();
        queue.push_back(index);
        seen[index] = 1;
        
        // キューへ追加した面を順に取り出し、同じ領域 ID の四方向隣接面へ探索範囲を広げる。
        for (auto position = std::size_t{}; position < queue.size(); ++position)
        {
            // 隣接方向ごとに接続先を確認し、別領域の面はこの連結成分へ含めない。
            for (auto direction = std::size_t{}; direction < cardinalCount; ++direction)
            {
                for (const auto target : i_field.spans[queue[position]].neighbors[direction])
                {
                    if (i_regions.ids[target] == id && !seen[target])
                    {
                        // 登録時点で訪問済みにすることで、複数経路から同じ面が重複登録されるのを防ぐ。
                        seen[target] = 1;
                        queue.push_back(target);
                    }
                }
            }
        }
        // 同じ ID の全ての面へ到達できた場合だけ、一つの連結領域として受け入れる。
        if (queue.size() != counts[id]) { throw std::invalid_argument("領域の歩行面が接続されていません。"); }
    }
    return counts;
}
}
