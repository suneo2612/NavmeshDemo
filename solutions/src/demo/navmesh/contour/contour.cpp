/**
 * @file contour.cpp
 * @brief 所有領域の外周と穴を境界辺から抽出する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "contour.h"
#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace demo::navmesh::contour
{
namespace
{
constexpr auto kDirections = std::array<voxelization::Direction, 4>{
    voxelization::Direction::North, voxelization::Direction::East,
    voxelization::Direction::South, voxelization::Direction::West};
constexpr auto kMissing = std::numeric_limits<std::size_t>::max();

/**
 * @brief 辺の面積への寄与を加算し、符号付き整数の上限を検査する。
 * @param io_area 現在の符号付き面積。
 * @param i_span 辺を持つ歩行面。
 * @param i_side 北・東・南・西を時計回りに表す添字。
 */
void AddArea(std::int64_t& io_area, const voxelization::WalkableSpan& i_span, const std::size_t i_side)
{
    constexpr auto limit = std::numeric_limits<std::int64_t>::max();
    if (i_span.x >= static_cast<std::size_t>(limit) || i_span.z >= static_cast<std::size_t>(limit))
    {
        throw std::length_error("輪郭の座標が表現可能な上限を超えました。");
    }
    const auto x = static_cast<std::int64_t>(i_span.x);
    const auto z = static_cast<std::int64_t>(i_span.z);
    const auto value = i_side == 0 ? -z : i_side == 1 ? x + 1 : i_side == 2 ? z + 1 : -x;
    if ((value > 0 && io_area > limit - value) ||
        (value < 0 && io_area < std::numeric_limits<std::int64_t>::min() - value))
    {
        throw std::length_error("輪郭の面積が表現可能な上限を超えました。");
    }
    io_area += value;
}

/**
 * @brief 境界辺の開始点をセル格子上で求める。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_edge 開始点を取得する辺。
 * @return XZ 格子上の開始点。
 */
std::pair<std::size_t, std::size_t> StartPoint(const voxelization::WalkableField& i_field, const Edge& i_edge)
{
    const auto& span = i_field.spans.at(i_edge.span);
    using voxelization::Direction;
    switch (i_edge.direction)
    {
    case Direction::North: return {span.x, span.z};
    case Direction::East: return {span.x + 1, span.z};
    case Direction::South: return {span.x + 1, span.z + 1};
    case Direction::West: return {span.x, span.z + 1};
    default: throw std::invalid_argument("輪郭辺の方向が不正です。");
    }
}

/**
 * @brief 境界の接続先を領域と高さで比較できる形にする。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_patches 平坦な領域の識別子。
 * @param i_edge 接続先を調べる辺。
 * @return 接続先の領域と高さの組。
 */
std::vector<std::pair<std::uint32_t, std::uint32_t>> Destinations(
    const voxelization::WalkableField& i_field, const region::Regions& i_patches, const Edge& i_edge)
{
    auto result = std::vector<std::pair<std::uint32_t, std::uint32_t>>{};
    const auto& span = i_field.spans.at(i_edge.span);
    for (const auto target : span.neighbors.at(static_cast<std::size_t>(i_edge.direction)))
    {
        result.emplace_back(i_patches.ids.at(target), i_field.spans.at(target).floor);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

/**
 * @brief 形状と接続状態を変えない範囲で直線上の輪郭点を省略する。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_flat 分割済み領域と元の輪郭。
 * @return 各輪郭で保持する元の辺の添字。
 */
std::vector<std::vector<std::size_t>> SelectEdges(const voxelization::WalkableField& i_field,
                                                  const FlatContours& i_flat)
{
    // 同じ領域の輪郭頂点が複数回現れる場所を記録し、接点を簡略化で失わないようにする。
    using PointKey = std::tuple<std::uint32_t, std::size_t, std::size_t>;
    auto occurrences = std::map<PointKey, std::size_t>{};
    for (const auto& loop : i_flat.contours)
    {
        for (const auto& edge : loop.edges)
        {
            const auto [x, z] = StartPoint(i_field, edge);
            ++occurrences[{loop.region, x, z}];
        }
    }
    auto result = std::vector<std::vector<std::size_t>>(i_flat.contours.size());
    // 方向が変わる点、輪郭の接点、前後の辺で接続先が変わる点を頂点として残す。
    for (auto loopIndex = std::size_t{}; loopIndex < i_flat.contours.size(); ++loopIndex)
    {
        const auto& loop = i_flat.contours[loopIndex];
        auto& selected = result[loopIndex];
        if (loop.edges.empty()) { throw std::invalid_argument("空の輪郭は簡略化できません。"); }
        for (auto index = std::size_t{}; index < loop.edges.size(); ++index)
        {
            const auto& previous = loop.edges[(index + loop.edges.size() - 1) % loop.edges.size()];
            const auto& current = loop.edges[index];
            const auto [x, z] = StartPoint(i_field, current);
            if (previous.direction != current.direction ||
                occurrences.at({loop.region, x, z}) > 1 ||
                Destinations(i_field, i_flat.patches, previous) != Destinations(i_field, i_flat.patches, current))
            {
                selected.push_back(index);
            }
        }
        if (selected.empty()) { throw std::invalid_argument("輪郭の頂点を選択できません。"); }
    }
    return result;
}
}

/**
 * @brief 所有領域の境界辺を接続に沿って閉じた輪郭にまとめる。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 外周・穴・退化輪郭。座標と高さは入力の歩行面から参照する。
 * @throws std::invalid_argument 入力や領域の接続が不正な場合。
 * @throws std::length_error 輪郭または面積が上限を超える場合。
 * @details タイル外の面は同一領域に含めず、タイル境界に辺を残す。
 * 面の高さの異なる辺は省略せず、元の面と方向をそのまま保持する。
 */
std::vector<Contour> BuildContours(const voxelization::WalkableField& i_field,
                                   const std::vector<std::size_t>& i_coreSpans,
                                   const std::vector<std::uint8_t>& i_eroded,
                                   const region::Regions& i_regions)
{
    // 入力領域の所有面と連結性を検証し、出力輪郭の面積配列を安全に扱えるようにする。
    const auto counts = region::ValidateRegions(i_field, i_coreSpans, i_eroded, i_regions);

    // 各面の四方向接続から、同じ領域に属する接続だけを内部辺として記録する。
    // 領域が違う接続、未所有面、タイル外へ続く方向は境界のまま残す。
    auto links = std::vector<std::array<std::size_t, 4>>(i_field.spans.size());
    for (auto& link : links) { link.fill(kMissing); }
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        const auto id = i_regions.ids[index];
        if (!id) { continue; }
        for (auto side = std::size_t{}; side < kDirections.size(); ++side)
        {
            for (const auto target : i_field.spans[index].neighbors[static_cast<std::size_t>(kDirections[side])])
            {
                if (i_regions.ids[target] == id) { links[index][side] = target; }
            }
        }
    }

    // 未訪問の境界辺を始点にし、外周・穴・接触輪郭を閉じるまで面の辺をたどる。
    // 境界では角を曲がり、内部辺では隣接面へ移ることで、領域の右側を保って周回する。
    auto visited = std::vector<std::uint8_t>(i_field.spans.size());
    auto contours = std::vector<Contour>{};
    for (auto start = std::size_t{}; start < i_field.spans.size(); ++start)
    {
        if (!i_regions.ids[start]) { continue; }
        for (auto firstSide = std::size_t{}; firstSide < kDirections.size(); ++firstSide)
        {
            const auto firstBit = static_cast<std::uint8_t>(1U << firstSide);
            if (links[start][firstSide] != kMissing || (visited[start] & firstBit)) { continue; }
            auto contour = Contour{i_regions.ids[start]};
            auto current = start;
            auto side = firstSide;
            auto steps = std::size_t{};
            if (counts[contour.region] > std::numeric_limits<std::size_t>::max() / (kDirections.size() * 2))
            {
                throw std::length_error("輪郭の辺数が上限を超えました。");
            }
            const auto maxSteps = counts[contour.region] * kDirections.size() * 2;
            do
            {
                // 上限を超える巡回や同じ辺の再訪を検出し、不正な接続で無限にたどらないようにする。
                if (++steps > maxSteps) { throw std::invalid_argument("領域の輪郭を閉じることができません。"); }
                if (links[current][side] == kMissing)
                {
                    const auto bit = static_cast<std::uint8_t>(1U << side);
                    if (visited[current] & bit) { throw std::invalid_argument("領域の輪郭が不正に重なっています。"); }
                    visited[current] |= bit;
                    contour.edges.push_back({current, kDirections[side]});
                    AddArea(contour.signedAreaTwice, i_field.spans[current], side);
                    side = (side + 1) % kDirections.size();
                }
                else
                {
                    current = links[current][side];
                    side = (side + kDirections.size() - 1) % kDirections.size();
                }
            }
            while (current != start || side != firstSide);
            contours.push_back(std::move(contour));
        }
    }
    return contours;
}

/**
 * @brief 領域を同じ高さの連結面に分割し、それぞれの輪郭を作る。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 平坦な連結面ごとの輪郭と、元の領域・高さの対応。
 * @throws std::invalid_argument 入力や領域の接続が不正な場合。
 * @throws std::length_error 領域または輪郭が上限を超える場合。
 * @details 段差を越える接続は別の輪郭に分ける。
 * 通行できない辺は同じ高さでも境界として残す。
 */
FlatContours BuildFlatContours(const voxelization::WalkableField& i_field,
                               const std::vector<std::size_t>& i_coreSpans,
                               const std::vector<std::uint8_t>& i_eroded,
                               const region::Regions& i_regions)
{
    // まず元の領域と歩行面の対応を検証し、段差をまたがない平坦領域を列挙する。
    static_cast<void>(region::ValidateRegions(i_field, i_coreSpans, i_eroded, i_regions));

    // 元の領域 ID と床高さの両方が一致する四方向連結成分を個別の patch として割り当てる。
    auto result = FlatContours{};
    result.patches.ids.resize(i_field.spans.size());
    result.sourceRegions.push_back(0);
    result.floors.push_back(0);
    auto queue = std::vector<std::size_t>{};
    for (auto seed = std::size_t{}; seed < i_field.spans.size(); ++seed)
    {
        const auto source = i_regions.ids[seed];
        if (!source || result.patches.ids[seed]) { continue; }
        if (result.patches.count == std::numeric_limits<std::uint32_t>::max())
        {
            throw std::length_error("平坦な領域数が上限を超えました。");
        }
        const auto patch = ++result.patches.count;
        const auto floor = i_field.spans[seed].floor;
        result.sourceRegions.push_back(source);
        result.floors.push_back(floor);
        queue.clear();
        queue.push_back(seed);
        result.patches.ids[seed] = patch;
        for (auto position = std::size_t{}; position < queue.size(); ++position)
        {
            for (const auto direction : kDirections)
            {
                for (const auto target : i_field.spans[queue[position]].neighbors[static_cast<std::size_t>(direction)])
                {
                    if (i_regions.ids[target] == source && i_field.spans[target].floor == floor &&
                        !result.patches.ids[target])
                    {
                        result.patches.ids[target] = patch;
                        queue.push_back(target);
                    }
                }
            }
        }
    }

    // 新しい patch ID を領域 ID として境界を抽出し、三角形化で必要な輪郭頂点だけを選ぶ。
    result.contours = BuildContours(i_field, i_coreSpans, i_eroded, result.patches);
    result.selectedEdges = SelectEdges(i_field, result);
    return result;
}
}
