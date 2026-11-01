/**
 * @file polygon_links.cpp
 * @brief 三角形の境界制約と歩行面接続からポリゴンのリンクを作る。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "polygon_links.h"
#include "contour_triangulation.h"
#include "../voxelization/tiles/dynamic_tiles.h"
#include <algorithm>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace demo::navmesh::polygon
{
namespace
{
using voxelization::Direction;
using SpanKey = std::tuple<std::size_t, std::size_t, std::uint32_t>;
using SideKey = std::tuple<std::size_t, std::size_t, Direction>;
using LinkKey = std::tuple<std::size_t, std::uint32_t, std::size_t, std::uint32_t>;
constexpr auto kMaxLinkSegments = std::size_t{8000000};

/**
 * @brief 歩行面をタイル間で照合するための共通キーを作る。
 * @param i_tile 歩行面を所有するタイル。
 * @param i_span タイル作業範囲内の歩行面。
 * @return ワールド格子上の X・Z 座標と床高さ。
 * @details タイル内の相対座標に作業範囲の原点を加えることで、
 * 隣り合うタイルの同じ歩行面を同一のキーで検索できるようにする。
 */
SpanKey Key(const voxelization::VoxelTile& i_tile, const voxelization::WalkableSpan& i_span)
{
    return {i_tile.region.work.beginX + i_span.x, i_tile.region.work.beginZ + i_span.z, i_span.floor};
}

/**
 * @brief 四方向接続の反対方向を返す。
 * @param i_direction 元の接続方向。
 * @return 元の方向と逆向きの方向。
 * @throws std::invalid_argument 四方向以外の値が指定された場合。
 */
Direction Opposite(const Direction i_direction)
{
    switch (i_direction)
    {
    case Direction::West: return Direction::East;
    case Direction::East: return Direction::West;
    case Direction::North: return Direction::South;
    case Direction::South: return Direction::North;
    default: throw std::invalid_argument("ポリゴン接続の方向が不正です。");
    }
}

/**
 * @brief 歩行面の指定された境界をタイル内格子座標で表す。
 * @param i_span 対象の歩行面。
 * @param i_direction 境界を取得する方向。
 * @return 境界線分の両端。端点順は方向ごとに一定。
 * @throws std::invalid_argument 四方向以外の値が指定された場合。
 */
std::array<std::array<double, 2>, 2> Side(const voxelization::WalkableSpan& i_span, const Direction i_direction)
{
    const auto x = static_cast<double>(i_span.x);
    const auto z = static_cast<double>(i_span.z);
    switch (i_direction)
    {
    case Direction::West: return {{{x, z}, {x, z + 1}}};
    case Direction::East: return {{{x + 1, z}, {x + 1, z + 1}}};
    case Direction::North: return {{{x, z}, {x + 1, z}}};
    case Direction::South: return {{{x, z + 1}, {x + 1, z + 1}}};
    default: throw std::invalid_argument("ポリゴン接続の方向が不正です。");
    }
}

/**
 * @brief 歩行面の境界線分がポリゴンの辺に含まれるか調べる。
 * @param i_mesh 頂点とポリゴンを保持するメッシュ。
 * @param i_polygon 所属を確認するポリゴン。
 * @param i_side タイル内格子座標で表した境界線分。
 * @return 線分全体を包含する水平または垂直なポリゴン辺がある場合は true。
 * @details 境界線分がポリゴン辺の途中にある場合も認める。頂点だけが接する状態を
 * 辺の共有と誤認しないよう、線分の両端が同じ直線上にあり、辺の範囲内にあることを確認する。
 */
bool Covers(const PolygonMesh& i_mesh, const Polygon& i_polygon,
            const std::array<std::array<double, 2>, 2>& i_side)
{
    // ポリゴンの各辺を調べ、セル境界の線分全体が水平または垂直な辺に含まれるか判定する。
    for (auto index = std::size_t{}; index < i_polygon.vertices.size(); ++index)
    {
        const auto& a = i_mesh.vertices.at(i_polygon.vertices[index]);
        const auto& b = i_mesh.vertices.at(i_polygon.vertices[(index + 1) % i_polygon.vertices.size()]);
        if (a.x == b.x && a.x == i_side[0][0] && a.x == i_side[1][0] &&
            std::min(a.z, b.z) <= i_side[0][1] && std::max(a.z, b.z) >= i_side[1][1]) { return true; }
        if (a.z == b.z && a.z == i_side[0][1] && a.z == i_side[1][1] &&
            std::min(a.x, b.x) <= i_side[0][0] && std::max(a.x, b.x) >= i_side[1][0]) { return true; }
    }
    return false;
}

/**
 * @brief 二つのポリゴン参照をタイル番号・ポリゴン番号の順で比較する。
 * @param i_a 比較する最初のポリゴン。
 * @param i_b 比較する二番目のポリゴン。
 * @return i_a が i_b より先に並ぶ場合は true。
 * @details 接続元と接続先の順序を正規化し、同じ接続を一つのキーに集約するために使う。
 */
bool Less(const PolygonRef& i_a, const PolygonRef& i_b)
{
    return std::tie(i_a.tile, i_a.polygon) < std::tie(i_b.tile, i_b.polygon);
}
}

/**
 * @brief 凸ポリゴン間の通行可能な共有辺を集約し、組ごとに双方向リンクを構築する。
 * @param i_tiles 行優先のタイルと、対応するポリゴン・領域。
 * @return 接続一覧と各ポリゴンからの参照。
 * @throws std::invalid_argument 入力に不整合がある場合。
 * @throws std::length_error 接続数が上限を超える場合。
 * @details まず歩行面の所有タイルと領域境界を索引化し、同一タイル内では三角形の
 * 内部辺を、タイル間では双方向に接続された歩行面の境界を接続候補として集める。
 * その後、同じポリゴン対に属する辺区間を統合し、リンクと各ポリゴンからの逆引きを作る。
 */
PolygonLinkGraph BuildPolygonLinks(const std::vector<PolygonLinkTile>& i_tiles)
{
    // リンク候補を種類ごとに照合できるよう、歩行面の所有者・輪郭境界・ポリゴン対の索引を用意する。
    // 最終的な adjacency はタイル・ポリゴン単位だが、接続の重複判定には共通格子座標を使う。
    auto graph = PolygonLinkGraph{};
    graph.adjacency.resize(i_tiles.size());
    auto spans = std::map<SpanKey, std::pair<std::size_t, std::size_t>>{};
    auto boundaries = std::map<SideKey, std::uint32_t>{};
    auto segments = std::map<LinkKey, std::vector<PolygonPortal>>{};
    auto segmentCount = std::size_t{};

    // 共通格子座標で得たポータルを、それぞれのタイル局所座標に戻してポリゴン辺へ対応付ける。
    // 出力には両ポリゴン上の辺番号が必要なため、候補を追加する段階で対応先の誤りも検出する。
    const auto findEdge = [&](const PolygonRef& i_ref, const std::array<double, 2>& i_begin,
                              const std::array<double, 2>& i_end)
    {
        const auto& view = i_tiles.at(i_ref.tile);
        const auto& mesh = *view.mesh;
        const auto& poly = mesh.polygons.at(i_ref.polygon);
        const auto x0 = i_begin[0] - static_cast<double>(view.tile->region.work.beginX);
        const auto z0 = i_begin[1] - static_cast<double>(view.tile->region.work.beginZ);
        const auto x1 = i_end[0] - static_cast<double>(view.tile->region.work.beginX);
        const auto z1 = i_end[1] - static_cast<double>(view.tile->region.work.beginZ);
        // 端点が同一直線上にあり、接続区間全体が辺の範囲内に収まる辺だけを採用する。
        for (auto index = std::size_t{}; index < poly.vertices.size(); ++index)
        {
            const auto& a = mesh.vertices.at(poly.vertices[index]);
            const auto& b = mesh.vertices.at(poly.vertices[(index + 1) % poly.vertices.size()]);
            if (ExactOrientation(a, b, x0, z0) == 0 && ExactOrientation(a, b, x1, z1) == 0 &&
                std::min(a.x, b.x) <= std::min(x0, x1) && std::max(a.x, b.x) >= std::max(x0, x1) &&
                std::min(a.z, b.z) <= std::min(z0, z1) && std::max(a.z, b.z) >= std::max(z0, z1))
            {
                return index;
            }
        }
        throw std::invalid_argument("接続区間をポリゴンの辺に対応付けられません。");
    };

    // 自分自身への接続は除き、ポリゴン対と端点を正規化して同じ接続を同じキーに集約する。
    // 区間数に上限を設け、無効な退化区間や大量入力による過剰なメモリ使用を拒否する。
    const auto append = [&](PolygonRef i_a, PolygonRef i_b,
                            std::array<double, 2> i_begin, std::array<double, 2> i_end)
    {
        if (i_a == i_b) { return; }
        if (Less(i_b, i_a)) { std::swap(i_a, i_b); }
        if (i_end < i_begin) { std::swap(i_begin, i_end); }
        if (i_begin == i_end) { throw std::invalid_argument("ポリゴンの接続辺が退化しています。"); }
        if (segmentCount >= kMaxLinkSegments) { throw std::length_error("ポリゴン接続の区間数が上限を超えました。"); }
        const auto key = LinkKey{i_a.tile, i_a.polygon, i_b.tile, i_b.polygon};
        segments[key].push_back({{findEdge(i_a, i_begin, i_end), findEdge(i_b, i_begin, i_end)}, i_begin, i_end});
        ++segmentCount;
    };

    // 各タイルの入力ポインターと面の参照先を検証しながら、所有面を共通格子座標で索引化する。
    // 同じ座標・高さの面が複数タイルに所有されていたら、接続先を一意に決められないため拒否する。
    for (auto tileIndex = std::size_t{}; tileIndex < i_tiles.size(); ++tileIndex)
    {
        const auto& view = i_tiles[tileIndex];
        if (!view.tile || !view.mesh || !view.regions) { throw std::invalid_argument("タイル接続の入力が不足しています。"); }
        const auto& tile = *view.tile;
        const auto& mesh = *view.mesh;
        graph.adjacency[tileIndex].resize(mesh.polygons.size());
        for (const auto spanIndex : tile.coreSpans)
        {
            if (!view.regions->ids.at(spanIndex)) { continue; }
            if (!spans.emplace(Key(tile, tile.retained.spans.at(spanIndex)), std::pair{tileIndex, spanIndex}).second)
            {
                throw std::invalid_argument("歩行面の所有タイルが重複しています。");
            }
        }

        // 三角形の辺を頂点番号の順序によらないキーで照合し、二つの三角形が共有する辺を探す。
        // 境界制約付きの辺は領域境界として扱い、制約のない辺だけを異なるポリゴン間の候補にする。
        auto edges = std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<std::size_t, bool>>{};
        for (auto index = std::size_t{}; index < mesh.triangles.size(); ++index)
        {
            const auto& tri = mesh.triangles[index];
            for (auto side = std::size_t{}; side < 3; ++side)
            {
                const auto a = tri.vertices[side];
                const auto b = tri.vertices[(side + 1) % 3];
                const auto edge = std::minmax(a, b);
                const auto [it, inserted] = edges.emplace(edge, std::pair{index, tri.boundary[side]});
                if (inserted) { continue; }
                if (it->second.second || tri.boundary[side]) { continue; }
                const auto& p = mesh.vertices.at(a);
                const auto& q = mesh.vertices.at(b);
                append({tileIndex, mesh.triangles[it->second.first].polygon},
                       {tileIndex, mesh.triangles[index].polygon},
                       {tile.region.work.beginX + p.x, tile.region.work.beginZ + p.z},
                       {tile.region.work.beginX + q.x, tile.region.work.beginZ + q.z});
            }
        }

        // 実際に通行できる歩行面境界を輪郭から求め、セルごとの境界をポリゴン辺へ対応付ける。
        // 複数ポリゴンへの重複所属や未対応の境界があれば、接続グラフの不整合として扱う。
        const auto flat = contour::BuildFlatContours(tile.retained, tile.coreSpans, tile.eroded, *view.regions);
        for (const auto& loop : flat.contours)
        {
            for (const auto& edge : loop.edges)
            {
                const auto& span = tile.retained.spans.at(edge.span);
                const auto segment = Side(span, edge.direction);
                auto found = kNoPolygon;
                for (const auto id : mesh.spanPolygons.at(edge.span))
                {
                    if (!Covers(mesh, mesh.polygons.at(id), segment)) { continue; }
                    if (found != kNoPolygon && found != id)
                    {
                        throw std::invalid_argument("歩行面の境界に複数のポリゴンが重なっています。");
                    }
                    found = id;
                }
                if (found == kNoPolygon) { throw std::invalid_argument("歩行面の境界にポリゴンがありません。"); }
                boundaries.emplace(SideKey{tileIndex, edge.span, edge.direction}, found);
            }
        }
    }

    // 所有面索引を使って隣接面が別タイルにある場合も同じ格子で検索し、境界同士を照合する。
    // 接続元からの辺だけでなく、移動先から元へ戻る逆接続も確認して、片方向だけの不整合を除外する。
    for (const auto& [sideKey, polygon] : boundaries)
    {
        const auto [tileIndex, spanIndex, direction] = sideKey;
        const auto& tile = *i_tiles[tileIndex].tile;
        const auto& span = tile.retained.spans.at(spanIndex);
        for (const auto neighborIndex : span.neighbors[static_cast<std::size_t>(direction)])
        {
            const auto& neighbor = tile.retained.spans.at(neighborIndex);
            const auto target = spans.find(Key(tile, neighbor));
            if (target == spans.end()) { continue; }
            const auto [targetTileIndex, targetSpanIndex] = target->second;
            const auto& targetTile = *i_tiles[targetTileIndex].tile;
            const auto& targetSpan = targetTile.retained.spans.at(targetSpanIndex);
            const auto opposite = Opposite(direction);
            const auto other = boundaries.find({targetTileIndex, targetSpanIndex, opposite});
            if (other == boundaries.end()) { continue; }
            const auto& reverse = targetSpan.neighbors[static_cast<std::size_t>(opposite)];
            const auto sourceKey = Key(tile, span);
            if (std::none_of(reverse.begin(), reverse.end(), [&](const auto i_id)
                { return Key(targetTile, targetTile.retained.spans.at(i_id)) == sourceKey; })) { continue; }
            auto segment = Side(span, direction);
            for (auto& point : segment)
            {
                point[0] += tile.region.work.beginX;
                point[1] += tile.region.work.beginZ;
            }
            append({tileIndex, polygon}, {targetTileIndex, other->second}, segment[0], segment[1]);
        }
    }

    // 同じポリゴン対の候補を辺番号・座標順に並べ、同じ辺にある重複または接触区間を一つにまとめる。
    // 統合後にリンクを一度だけ登録し、両側のポリゴンから同じリンクを引けるよう逆引き配列も更新する。
    for (auto& [key, pieces] : segments)
    {
        std::sort(pieces.begin(), pieces.end(), [](const auto& i_a, const auto& i_b)
        {
            return std::tie(i_a.edges, i_a.begin, i_a.end) < std::tie(i_b.edges, i_b.begin, i_b.end);
        });
        auto portals = std::vector<PolygonPortal>{};
        for (const auto& piece : pieces)
        {
            // 辺番号が一致し、次の区間が現在の終点を越えない場合に限り連続区間として統合する。
            if (!portals.empty() && portals.back().edges == piece.edges && piece.begin <= portals.back().end)
            {
                portals.back().end = std::max(portals.back().end, piece.end);
            }
            else
            {
                portals.push_back(piece);
            }
        }
        const auto [tileA, polygonA, tileB, polygonB] = key;
        const auto index = graph.links.size();
        graph.links.push_back({std::array{PolygonRef{tileA, polygonA}, PolygonRef{tileB, polygonB}},
                               std::move(portals)});
        graph.adjacency.at(tileA).at(polygonA).push_back(index);
        graph.adjacency.at(tileB).at(polygonB).push_back(index);
    }
    return graph;
}
}
