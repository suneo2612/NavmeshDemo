/**
 * @file core_erosion.cpp
 * @brief 境界線とセル全体の距離から歩行面の侵食を求める。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#include "core_erosion.h"
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
constexpr auto kCardinalDirections = std::size_t{4};
constexpr auto kMaxChecks = std::size_t{100000000};

/**
 * @brief セル単位の XZ 矩形を表す。
 */
struct Rect
{
    double minX; // X の最小値。
    double minZ; // Z の最小値。
    double maxX; // X の最大値。
    double maxZ; // Z の最大値。
};

/**
 * @brief 矩形同士の最短距離の二乗を返す。
 * @param i_a 一方の矩形。線分も扱う。
 * @param i_b 他方の矩形。
 * @return セル幅を単位とする距離の二乗。
 */
double DistanceSquared(const Rect& i_a, const Rect& i_b)
{
    const auto x = std::max({i_a.minX - i_b.maxX, i_b.minX - i_a.maxX, 0.0});
    const auto z = std::max({i_a.minZ - i_b.maxZ, i_b.minZ - i_a.maxZ, 0.0});
    return x * x + z * z;
}

/**
 * @brief 座標の検査範囲を作業範囲内へ制限する。
 * @param i_value セル座標。
 * @param i_count 作業範囲のセル数。
 * @return 範囲内のセル添字。
 */
std::size_t ClampCell(const double i_value, const std::size_t i_count)
{
    // 小数座標をセル添字へ変換する前に両端へ制限し、範囲外アクセスを防ぐ。
    if (i_value <= 0) { return 0; }
    if (i_value >= static_cast<double>(i_count - 1)) { return i_count - 1; }
    return static_cast<std::size_t>(i_value);
}

/**
 * @brief 開始位置を含み、空でないセル範囲か確認する。
 * @param i_rect 確認する範囲。
 * @return X、Z の両方向に一つ以上のセルがある場合は true。
 */
bool IsNonEmpty(const CellRect& i_rect)
{
    return i_rect.beginX < i_rect.endX && i_rect.beginZ < i_rect.endZ;
}

/**
 * @brief 一つのセル範囲が別の範囲内に収まるか確認する。
 * @param i_outer 内側の範囲を含む側。
 * @param i_inner 含まれているか確認する側。
 * @return X、Z の両方向で内側の範囲が収まる場合は true。
 */
bool ContainsRect(const CellRect& i_outer, const CellRect& i_inner)
{
    return i_inner.beginX >= i_outer.beginX && i_inner.beginZ >= i_outer.beginZ &&
           i_inner.endX <= i_outer.endX && i_inner.endZ <= i_outer.endZ;
}

/**
 * @brief タイルの作業範囲と高さ場・歩行面の寸法が一致するか確認する。
 * @param i_heightfield 対象タイルの高さ場。
 * @param i_walkable 対象タイルの歩行面。
 * @param i_work タイルの作業範囲。
 * @return 範囲と配列の幅・奥行きが一致する場合は true。
 */
bool HasMatchingWorkDimensions(const Heightfield& i_heightfield,
                               const WalkableField& i_walkable, const CellRect& i_work)
{
    return IsNonEmpty(i_work) &&
           i_work.endX - i_work.beginX == i_walkable.width &&
           i_work.endZ - i_work.beginZ == i_walkable.depth &&
           i_heightfield.width == i_walkable.width &&
           i_heightfield.depth == i_walkable.depth;
}

/**
 * @brief 出力範囲と距離計算範囲が作業範囲内に正しく収まるか確認する。
 * @param i_work タイルの作業範囲。
 * @param i_core このタイルが出力を担当する範囲。
 * @param i_managed 距離を計算する範囲。
 * @return どちらの範囲も作業範囲内にあり、出力範囲が距離計算範囲内にある場合は true。
 */
bool HasValidRegionNesting(const CellRect& i_work, const CellRect& i_core,
                           const CellRect& i_managed)
{
    return IsNonEmpty(i_core) && ContainsRect(i_work, i_core) &&
           IsNonEmpty(i_managed) && ContainsRect(i_work, i_managed) &&
           ContainsRect(i_managed, i_core);
}

/**
 * @brief 距離計算範囲を示す配列の長さと値が有効か確認する。
 * @param i_managed 各歩行面が距離計算範囲内にあるかを示す配列。
 * @param i_spanCount 歩行面の数。
 * @return 面ごとに値があり、各値が 0 または 1 の場合は true。
 */
bool HasValidManagedMask(const std::vector<std::uint8_t>& i_managed,
                         const std::size_t i_spanCount)
{
    return i_managed.size() == i_spanCount &&
           std::none_of(i_managed.begin(), i_managed.end(), [](const auto i_value) { return i_value > 1; });
}
}

/**
 * @brief 所有セル全体から歩行不能境界までの距離を調べる。
 * @param i_heightfield タイルの作業範囲の高さ場。範囲は共通地形の座標。
 * @param i_region タイルの所有・管理・作業範囲。
 * @param i_walkable 作業範囲の歩行面と接続。
 * @param i_managed 各面が管理範囲内なら 1。
 * @param i_coreSpans 所有範囲の歩行面の添字。
 * @param i_agentRadius キャラクター半径。
 * @return 歩行面と同じ添字の配列。所有面が侵食された場合は 1、その他は 0。
 * @throws std::invalid_argument 入力が不正な場合。
 * @throws std::length_error 判定回数の上限を超えた場合。
 * @details 高さ方向のキャラクター範囲が接する四方向の歩行不能境界に対し、
 * セル矩形と境界線の最短距離が半径以下なら侵食する。十分離れた階層は区別する。
 * 近接している別階層の境界は保守的に侵食する場合がある。
 * 所有範囲外は判定対象ではなく、戻り値の 0 は歩行可能を意味しない。
 */
std::vector<std::uint8_t> BuildCoreErosion(const Heightfield& i_heightfield,
                                           const TileRegion& i_region,
                                           const WalkableField& i_walkable,
                                           const std::vector<std::uint8_t>& i_managed,
                                           const std::vector<std::size_t>& i_coreSpans,
                                           const float i_agentRadius)
{
    // 面配列と高さ場、タイルの作業・管理・所有範囲が整合しているか先に検証する。
    ValidateWalkableField(i_walkable);
    const auto cell = static_cast<double>(i_heightfield.settings.cellSize);
    const auto& work = i_region.work;
    const auto& core = i_region.core;
    const auto& managed = i_region.managed;
    if (!std::isfinite(cell) || cell <= 0 || !std::isfinite(i_agentRadius) || i_agentRadius <= 0 ||
        !HasMatchingWorkDimensions(i_heightfield, i_walkable, work) ||
        !HasValidRegionNesting(work, core, managed) ||
        !HasValidManagedMask(i_managed, i_walkable.spans.size()))
    {
        throw std::invalid_argument("侵食する歩行面の範囲または寸法が不正です。");
    }
    const auto limitX = (static_cast<double>(i_heightfield.bounds.maximum.x) -
                         i_heightfield.bounds.minimum.x) / cell - work.beginX;
    const auto limitZ = (static_cast<double>(i_heightfield.bounds.maximum.z) -
                         i_heightfield.bounds.minimum.z) / cell - work.beginZ;
    const auto radius = static_cast<double>(i_agentRadius) / cell;
    if (!std::isfinite(limitX) || !std::isfinite(limitZ) || !std::isfinite(radius) ||
        limitX <= static_cast<double>(i_walkable.width - 1) ||
        limitZ <= static_cast<double>(i_walkable.depth - 1))
    {
        throw std::invalid_argument("侵食するセルの地形範囲が不正です。");
    }

    // 所有面の一覧からマスクを作り、重複・範囲外の添字や管理範囲との不一致を検出する。
    auto isCore = std::vector<std::uint8_t>(i_walkable.spans.size());
    for (const auto index : i_coreSpans)
    {
        if (index >= i_walkable.spans.size() || isCore[index] || !i_managed[index])
        {
            throw std::invalid_argument("侵食する所有面の添字が不正です。");
        }
        const auto& span = i_walkable.spans[index];
        if (span.x + work.beginX < core.beginX || span.x + work.beginX >= core.endX ||
            span.z + work.beginZ < core.beginZ || span.z + work.beginZ >= core.endZ)
        {
            throw std::invalid_argument("侵食する所有面が所有範囲外です。");
        }
        isCore[index] = 1;
    }
    for (auto index = std::size_t{}; index < i_walkable.spans.size(); ++index)
    {
        const auto& span = i_walkable.spans[index];
        const auto x = span.x + work.beginX;
        const auto z = span.z + work.beginZ;
        const auto inManaged = x >= managed.beginX && x < managed.endX &&
                               z >= managed.beginZ && z < managed.endZ;
        const auto inCore = x >= core.beginX && x < core.endX && z >= core.beginZ && z < core.endZ;
        if (static_cast<bool>(i_managed[index]) != inManaged || static_cast<bool>(isCore[index]) != inCore)
        {
            throw std::invalid_argument("侵食する面とタイル範囲の対応が不正です。");
        }
    }

    // 各管理面について、接続のない四方向を歩行不能境界として調べる。
    // 距離比較はセル矩形の実形状で行い、丸め誤差はわずかな許容幅で安全側に扱う。
    auto result = std::vector<std::uint8_t>(i_walkable.spans.size());
    auto checks = std::size_t{};
    const auto tolerance = 16 * std::numeric_limits<double>::epsilon() *
        std::max({1.0, radius, static_cast<double>(i_walkable.width), static_cast<double>(i_walkable.depth)});
    const auto conservativeRadius = radius + tolerance;
    const auto radiusSquared = conservativeRadius * conservativeRadius;
    for (auto index = std::size_t{}; index < i_walkable.spans.size(); ++index)
    {
        if (!i_managed[index]) { continue; }
        const auto& span = i_walkable.spans[index];
        const auto x = static_cast<double>(span.x);
        const auto z = static_cast<double>(span.z);
        const auto xEnd = std::min(x + 1, limitX);
        const auto zEnd = std::min(z + 1, limitZ);
        const auto edges = std::array<Rect, kCardinalDirections>{
            Rect{x, z, x, zEnd}, Rect{xEnd, z, xEnd, zEnd},
            Rect{x, z, xEnd, z}, Rect{x, zEnd, xEnd, zEnd}};
        for (auto direction = std::size_t{}; direction < kCardinalDirections; ++direction)
        {
            // 接続が残る方向は境界ではないため飛ばし、影響し得る所有セルだけに候補を絞る。
            if (!span.neighbors[direction].empty()) { continue; }
            const auto& edge = edges[direction];
            const auto beginX = std::max(core.beginX - work.beginX,
                ClampCell(std::ceil(edge.minX - conservativeRadius - 1), i_walkable.width));
            const auto endX = std::min(core.endX - work.beginX - 1,
                ClampCell(std::floor(edge.maxX + conservativeRadius), i_walkable.width));
            const auto beginZ = std::max(core.beginZ - work.beginZ,
                ClampCell(std::ceil(edge.minZ - conservativeRadius - 1), i_walkable.depth));
            const auto endZ = std::min(core.endZ - work.beginZ - 1,
                ClampCell(std::floor(edge.maxZ + conservativeRadius), i_walkable.depth));
            if (beginX > endX || beginZ > endZ) { continue; }
            for (auto targetZ = beginZ; targetZ <= endZ; ++targetZ)
            {
                for (auto targetX = beginX; targetX <= endX; ++targetX)
                {
                    if (++checks > kMaxChecks)
                    {
                        throw std::length_error("歩行範囲の侵食判定が上限を超えました。");
                    }
                    const auto column = targetZ * i_walkable.width + targetX;
                    const auto tx = static_cast<double>(targetX);
                    const auto tz = static_cast<double>(targetZ);
                    const auto targetRect = Rect{tx, tz, std::min(tx + 1, limitX), std::min(tz + 1, limitZ)};
                    if (DistanceSquared(edge, targetRect) > radiusSquared) { continue; }
                    // 候補列の全高さ面を調べ、キャラクターの高さに重なる所有面だけを侵食する。
                    for (auto target = i_walkable.offsets[column]; target < i_walkable.offsets[column + 1]; ++target)
                    {
                        const auto floor = i_walkable.spans[target].floor;
                        const auto heightDifference = std::max(floor, span.floor) - std::min(floor, span.floor);
                        if (isCore[target] && heightDifference <= i_walkable.requiredHeight) { result[target] = 1; }
                    }
                }
            }
        }
    }
    return result;
}
}
