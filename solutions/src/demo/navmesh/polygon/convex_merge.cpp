/**
 * @file convex_merge.cpp
 * @brief 境界を越えずに三角形を凸多角形へ結合する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "convex_merge.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace demo::navmesh::polygon
{
namespace
{
using Edge = std::pair<std::uint32_t, std::uint32_t>;
constexpr auto kMissing = std::numeric_limits<std::size_t>::max();

/**
 * @brief 実距離と、丸め誤差を含まない格子上の位置。
 */
struct Point
{
    double first;  // X 方向の実距離。
    double second; // Z 方向の実距離。
    double gridX;  // X 方向の作業範囲内の格子座標。
    double gridZ;  // Z 方向の作業範囲内の格子座標。
    double lengthX; // 短い地形端を反映した X 座標。セル幅を単位とする。
    double lengthZ; // 短い地形端を反映した Z 座標。セル幅を単位とする。
    int sideX;     // 地形端の短いセルとの X 方向の位置関係。
    int sideZ;     // 地形端の短いセルとの Z 方向の位置関係。
};

/**
 * @brief 内部処理中のポリゴン。
 */
struct WorkPolygon
{
    std::vector<std::uint32_t> ring;    // 結合前の辺の区切りを残した境界。
    std::vector<std::uint32_t> corners; // 共一直線上の点を除いた角。
    std::vector<std::size_t> triangles; // 元の三角形の添字。
    std::uint32_t patch = 0;            // 平坦な連結領域の ID。
    std::size_t version = 0;            // 候補の更新判定に使う世代。
    bool active = true;                 // 別のポリゴンに結合済みなら false。
};

/**
 * @brief 優先順位付きの結合候補。
 */
struct Candidate
{
    double length;       // 共一直線上に続く共有辺の最大長。
    std::size_t first;   // 結合元の添字。
    std::size_t second;  // もう一方の添字。
    std::size_t versionFirst;  // 登録時の結合元の世代。
    std::size_t versionSecond; // 登録時のもう一方の世代。
};

/**
 * @brief 長い辺を優先し、同じ長さでは添字順で選ぶ。
 */
struct CandidateOrder
{
    /**
     * @brief 共有辺が短い候補を先に取り出し、同じ長さでは添字順で安定させる。
     * @param i_a 比較する候補。
     * @param i_b 比較する候補。
     * @return i_a の優先度が i_b より低い場合は true。
     */
    bool operator()(const Candidate& i_a, const Candidate& i_b) const
    {
        // priority_queue が最長の共有辺を先頭にするよう、長さと添字を逆順で比較する。
        if (i_a.length != i_b.length) { return i_a.length < i_b.length; }
        if (i_a.first != i_b.first) { return i_a.first > i_b.first; }
        return i_a.second > i_b.second;
    }
};

/**
 * @brief 頂点添字の順序に依存しない辺を作る。
 * @param i_a 一方の端点。
 * @param i_b 他方の端点。
 * @return 昇順に揃えた端点。
 */
Edge Undirected(const std::uint32_t i_a, const std::uint32_t i_b)
{
    return std::minmax(i_a, i_b);
}

/**
 * @brief 上向きの巻き順が正になる外積を求める。
 * @param i_a 最初の頂点。
 * @param i_b 中間の頂点。
 * @param i_c 最後の頂点。
 * @return 上向きなら 1、下向きなら -1、同一直線上なら 0。
 * @details 地形端の短いセルをまたがない場合は格子座標を使い、変換時の丸めを避ける。
 * 端セルをまたぐ場合は格子間隔が一定でないため、実距離座標で判定する。
 */
int Cross(const Point& i_a, const Point& i_b, const Point& i_c)
{
    // 同じ幅のセル内では格子座標の向きが実距離と一致し、小数のセル幅による丸めを避けられる。
    const auto sameX = std::min({i_a.sideX, i_b.sideX, i_c.sideX}) >= 0 ||
                       std::max({i_a.sideX, i_b.sideX, i_c.sideX}) <= 0;
    const auto sameZ = std::min({i_a.sideZ, i_b.sideZ, i_c.sideZ}) >= 0 ||
                       std::max({i_a.sideZ, i_b.sideZ, i_c.sideZ}) <= 0;
    if (sameX && sameZ)
    {
        return -ExactOrientation({i_a.gridX, i_a.gridZ, 0}, {i_b.gridX, i_b.gridZ, 0},
                                 i_c.gridX, i_c.gridZ);
    }
    // 地形端の短いセルをまたぐ場合は、格子から実距離への変換が線形でない。
    return -ExactOrientation({i_a.first, i_a.second, 0}, {i_b.first, i_b.second, 0}, i_c.first, i_c.second);
}

/**
 * @brief 結合した境界を作り、単純性・凸性・頂点数を確認する。
 * @param i_first 一方のポリゴン。
 * @param i_second 他方のポリゴン。
 * @param i_points 実距離に換算した頂点。
 * @param i_boundaries 結合を禁止する境界辺。
 * @param o_result 条件を満たした場合の結合結果。
 * @param o_length 共一直線上に続く共有辺の最大長。
 * @return 結合可能なら true。
 * @details 同じ平坦パッチ内の二つのポリゴンについて、共有辺の向き・境界制約・
 * 結合後の輪の閉路・凸性・頂点数を確認する。出力引数は結合可能な場合のみ完成させる。
 */
bool TryMerge(const WorkPolygon& i_first, const WorkPolygon& i_second, const std::vector<Point>& i_points,
              const std::set<Edge>& i_boundaries, WorkPolygon& o_result, double& o_length)
{
    // 高さの異なる面や別々の平坦パッチは、辺を共有していても同一ポリゴンにしない。
    if (i_first.patch != i_second.patch) { return false; }
    // 共有辺を端点の昇順で索引化し、両ポリゴンで逆向きに使われている辺だけを探す。
    auto otherEdges = std::map<Edge, std::pair<std::uint32_t, std::uint32_t>>{};
    for (auto index = std::size_t{}; index < i_second.ring.size(); ++index)
    {
        const auto a = i_second.ring[index];
        const auto b = i_second.ring[(index + 1) % i_second.ring.size()];
        otherEdges.emplace(Undirected(a, b), std::pair{a, b});
    }
    auto shared = std::set<Edge>{};
    for (auto index = std::size_t{}; index < i_first.ring.size(); ++index)
    {
        const auto a = i_first.ring[index];
        const auto b = i_first.ring[(index + 1) % i_first.ring.size()];
        const auto key = Undirected(a, b);
        const auto found = otherEdges.find(key);
        if (found == otherEdges.end()) { continue; }
        if (found->second != std::pair{b, a} || i_boundaries.contains(key)) { return false; }
        shared.insert(key);
    }
    if (shared.empty()) { return false; }

    // 共有辺は内部になるため除外し、残った辺を始点から終点へつないで一つの閉じた境界を作る。
    // 重複する出辺や閉路にならない辺があれば、自己交差・分岐した形状として結合を拒否する。
    auto next = std::map<std::uint32_t, std::uint32_t>{};
    for (const auto* polygon : {&i_first, &i_second})
    {
        for (auto index = std::size_t{}; index < polygon->ring.size(); ++index)
        {
            const auto a = polygon->ring[index];
            const auto b = polygon->ring[(index + 1) % polygon->ring.size()];
            if (shared.contains(Undirected(a, b))) { continue; }
            if (!next.emplace(a, b).second) { return false; }
        }
    }
    if (next.size() < 3) { return false; }
    auto ring = std::vector<std::uint32_t>{};
    auto current = next.begin()->first;
    const auto start = current;
    do
    {
        const auto found = next.find(current);
        if (found == next.end() || ring.size() >= next.size()) { return false; }
        ring.push_back(current);
        current = found->second;
    }
    while (current != start);
    if (ring.size() != next.size()) { return false; }

    // 辺の分割点は輪に残して境界との対応を維持しつつ、凸性と実際の角数を別に判定する。
    auto corners = std::vector<std::uint32_t>{};
    for (auto index = std::size_t{}; index < ring.size(); ++index)
    {
        const auto& a = i_points[ring[(index + ring.size() - 1) % ring.size()]];
        const auto& b = i_points[ring[index]];
        const auto& c = i_points[ring[(index + 1) % ring.size()]];
        const auto turn = Cross(a, b, c);
        if (turn < 0) { return false; }
        if (turn == 0)
        {
            // 同一直線上でも折り返す頂点は単純な凸境界ではないため、進行方向の内積も確認する。
            const auto dot = (b.first - a.first) * (c.first - b.first) +
                             (b.second - a.second) * (c.second - b.second);
            if (dot <= 0) { return false; }
        }
        else { corners.push_back(ring[index]); }
    }
    if (corners.size() < 3 || corners.size() > kMaxConvexVertices) { return false; }

    // 同じ直線上で複数辺に分割された共有部分を連続区間にまとめ、
    // 候補の優先順位に使う最大共有長を求める。
    o_length = 0;
    auto runStart = kMissing;
    auto previous = kMissing;
    auto firstUnshared = std::size_t{};
    while (firstUnshared < i_first.ring.size() &&
           shared.contains(Undirected(i_first.ring[firstUnshared],
                                      i_first.ring[(firstUnshared + 1) % i_first.ring.size()])))
    {
        ++firstUnshared;
    }
    if (firstUnshared == i_first.ring.size()) { return false; }
    for (auto offset = std::size_t{1}; offset <= i_first.ring.size(); ++offset)
    {
        const auto index = (firstUnshared + offset) % i_first.ring.size();
        const auto a = i_first.ring[index];
        const auto b = i_first.ring[(index + 1) % i_first.ring.size()];
        if (!shared.contains(Undirected(a, b)))
        {
            if (runStart != kMissing)
            {
                o_length = std::max(o_length, std::hypot(i_points[a].lengthX - i_points[runStart].lengthX,
                                                          i_points[a].lengthZ - i_points[runStart].lengthZ));
            }
            runStart = kMissing;
            previous = kMissing;
            continue;
        }
        if (previous != kMissing && Cross(i_points[i_first.ring[previous]], i_points[a], i_points[b]) != 0)
        {
            // 曲がり角で共有辺の連続区間が終わるため、その区間長を確定して次の区間へ移る。
            o_length = std::max(o_length, std::hypot(i_points[a].lengthX - i_points[runStart].lengthX,
                                                      i_points[a].lengthZ - i_points[runStart].lengthZ));
            runStart = kMissing;
        }
        if (runStart == kMissing) { runStart = a; }
        previous = index;
    }
    if (!std::isfinite(o_length) || o_length <= 0) { throw std::overflow_error("共有辺の長さを表現できません。"); }
    o_result.ring = std::move(ring);
    o_result.corners = std::move(corners);
    o_result.patch = i_first.patch;
    return true;
}
}

/**
 * @brief 長い共有辺を優先し、凸性と頂点数上限を守って結合する。
 * @param i_mesh 共通頂点と境界制約を持つ三角形。
 * @param i_metric 地形端を含む実距離の換算情報。
 * @return 凸多角形と三角形の対応。
 * @throws std::invalid_argument 三角形または寸法が不正な場合。
 * @throws std::length_error 頂点またはポリゴン数が上限を超える場合。
 * @throws std::overflow_error 凸性や辺の長さを表現できない場合。
 * @details 境界制約のある辺は結合しない。すべての共有辺は端点が一致する必要がある。
 */
ConvexMerge MergeContourTriangles(const ContourTriangulation& i_mesh, const GridMetric& i_metric)
{
    // 距離換算に必要な寸法と、三角形番号を結果に格納できる上限を先に確認する。
    if (!std::isfinite(i_metric.cellSize) || i_metric.cellSize <= 0 ||
        !std::isfinite(i_metric.extentX) || i_metric.extentX <= 0 ||
        !std::isfinite(i_metric.extentZ) || i_metric.extentZ <= 0)
    {
        throw std::invalid_argument("三角形の格子寸法が不正です。");
    }
    if (i_mesh.triangles.size() >= static_cast<std::size_t>(kNoPolygon))
    {
        throw std::length_error("三角形の数が上限を超えました。");
    }

    // 地形端でセル幅が変わる位置と端セルの比率を求め、各頂点の座標変換を一度だけ行う。
    // 判定用に実距離・格子座標・短い端セルからの位置関係を併せて保持する。
    auto points = std::vector<Point>{};
    points.reserve(i_mesh.vertices.size());
    const auto transitionX = std::floor(i_metric.extentX / i_metric.cellSize);
    const auto transitionZ = std::floor(i_metric.extentZ / i_metric.cellSize);
    const auto shortX = transitionX * i_metric.cellSize < i_metric.extentX;
    const auto shortZ = transitionZ * i_metric.cellSize < i_metric.extentZ;
    const auto fractionX = shortX ? (i_metric.extentX - transitionX * i_metric.cellSize) / i_metric.cellSize : 1.0;
    const auto fractionZ = shortZ ? (i_metric.extentZ - transitionZ * i_metric.cellSize) / i_metric.cellSize : 1.0;
    for (const auto& vertex : i_mesh.vertices)
    {
        const auto gridX = static_cast<double>(i_metric.beginX) + vertex.x;
        const auto gridZ = static_cast<double>(i_metric.beginZ) + vertex.z;
        const auto x = GridCoordinate(gridX,
                                      i_metric.cellSize, i_metric.extentX);
        const auto z = GridCoordinate(gridZ,
                                      i_metric.cellSize, i_metric.extentZ);
        if (!std::isfinite(x) || !std::isfinite(z))
        {
            throw std::overflow_error("三角形の座標を表現できません。");
        }
        const auto clippedX = std::min(gridX, transitionX + (shortX ? 1.0 : 0.0));
        const auto clippedZ = std::min(gridZ, transitionZ + (shortZ ? 1.0 : 0.0));
        // 端セルの外側に続く頂点は境界位置へまとめ、凸性判定に使う距離座標も端幅で補正する。
        const auto sideX = shortX ? (clippedX > transitionX) - (clippedX < transitionX) : 0;
        const auto sideZ = shortZ ? (clippedZ > transitionZ) - (clippedZ < transitionZ) : 0;
        const auto localX = clippedX - static_cast<double>(i_metric.beginX);
        const auto localZ = clippedZ - static_cast<double>(i_metric.beginZ);
        const auto lengthX = localX - (sideX > 0 ? (clippedX - transitionX) * (1.0 - fractionX) : 0.0);
        const auto lengthZ = localZ - (sideZ > 0 ? (clippedZ - transitionZ) * (1.0 - fractionZ) : 0.0);
        points.push_back({std::min(x, i_metric.extentX), std::min(z, i_metric.extentZ),
                          localX, localZ, lengthX, lengthZ, sideX, sideZ});
    }

    // 入力三角形を一時ポリゴンとして登録し、隣接探索に使う辺利用数と結合禁止辺を収集する。
    auto polygons = std::vector<WorkPolygon>(i_mesh.triangles.size());
    auto owners = std::vector<std::size_t>(i_mesh.triangles.size());
    auto edgeUses = std::map<Edge, std::vector<std::pair<std::size_t, bool>>>{};
    auto boundaries = std::set<Edge>{};
    for (auto index = std::size_t{}; index < i_mesh.triangles.size(); ++index)
    {
        const auto& triangle = i_mesh.triangles[index];
        if (!triangle.patch || triangle.patch >= i_mesh.flat.floors.size())
        {
            throw std::invalid_argument("三角形の領域が不正です。");
        }
        for (const auto vertex : triangle.vertices)
        {
            if (vertex >= points.size() || i_mesh.vertices[vertex].floor != i_mesh.flat.floors[triangle.patch])
            {
                throw std::invalid_argument("三角形の頂点または高さが不正です。");
            }
        }
        if (Cross(points[triangle.vertices[0]], points[triangle.vertices[1]], points[triangle.vertices[2]]) <= 0)
        {
            throw std::invalid_argument("三角形の向きまたは面積が不正です。");
        }
        auto& polygon = polygons[index];
        polygon.ring.assign(triangle.vertices.begin(), triangle.vertices.end());
        polygon.corners = polygon.ring;
        polygon.triangles.push_back(index);
        polygon.patch = triangle.patch;
        owners[index] = index;
        for (auto side = std::size_t{}; side < triangle.vertices.size(); ++side)
        {
            const auto key = Undirected(triangle.vertices[side], triangle.vertices[(side + 1) % 3]);
            edgeUses[key].emplace_back(index, triangle.boundary[side]);
            if (triangle.boundary[side]) { boundaries.insert(key); }
        }
    }

    // 各辺の利用数と境界フラグを検証する。内部辺は二つの三角形が逆向きに共有し、
    // 領域境界は一つの三角形だけが使用する前提で、これに反する入力を拒否する。
    auto neighbors = std::vector<std::vector<std::size_t>>(i_mesh.triangles.size());
    for (const auto& [edge, uses] : edgeUses)
    {
        static_cast<void>(edge);
        if (uses.size() > 2 || (uses.size() == 1 && !uses[0].second) ||
            (uses.size() == 2 && uses[0].second != uses[1].second))
        {
            throw std::invalid_argument("三角形の共有辺または境界が不正です。");
        }
        if (uses.size() == 2 && !uses[0].second)
        {
            neighbors[uses[0].first].push_back(uses[1].first);
            neighbors[uses[1].first].push_back(uses[0].first);
        }
    }

    // 隣接する組だけを結合候補にし、長い共有辺を先に選べる優先キューへ登録する。
    // 候補作成時の世代番号も記録し、後から形状が変化した候補を識別する。
    auto queue = std::priority_queue<Candidate, std::vector<Candidate>, CandidateOrder>{};
    const auto enqueue = [&](const std::size_t i_a, const std::size_t i_b)
    {
        const auto first = std::min(i_a, i_b);
        const auto second = std::max(i_a, i_b);
        if (first == second) { return; }
        auto combined = WorkPolygon{};
        auto length = 0.0;
        if (TryMerge(polygons[first], polygons[second], points, boundaries, combined, length))
        {
            queue.push({length, first, second, polygons[first].version, polygons[second].version});
        }
    };
    for (auto index = std::size_t{}; index < neighbors.size(); ++index)
    {
        for (const auto neighbor : neighbors[index]) { if (index < neighbor) { enqueue(index, neighbor); } }
    }

    // 最長候補から順に処理し、すでに結合済みの形状を指す古い候補は世代番号で破棄する。
    // 結合後は三角形の所有者を更新し、新たに隣接したポリゴンとの候補だけを作り直す。
    while (!queue.empty())
    {
        const auto candidate = queue.top();
        queue.pop();
        auto& first = polygons[candidate.first];
        auto& second = polygons[candidate.second];
        if (!first.active || !second.active || first.version != candidate.versionFirst ||
            second.version != candidate.versionSecond) { continue; }
        auto combined = WorkPolygon{};
        auto length = 0.0;
        if (!TryMerge(first, second, points, boundaries, combined, length))
        {
            throw std::logic_error("有効な結合候補が変化しました。");
        }
        combined.triangles = std::move(first.triangles);
        combined.triangles.insert(combined.triangles.end(), second.triangles.begin(), second.triangles.end());
        combined.version = first.version + 1;
        first = std::move(combined);
        second.active = false;
        auto adjacent = std::set<std::size_t>{};
        for (const auto triangle : first.triangles)
        {
            owners[triangle] = candidate.first;
        }
        // 同じ相手が複数三角形の隣接先として現れても、一組につき一度だけ候補を追加する。
        for (const auto triangle : first.triangles)
        {
            for (const auto neighbor : neighbors[triangle])
            {
                if (owners[neighbor] != candidate.first) { adjacent.insert(owners[neighbor]); }
            }
        }
        for (const auto neighbor : adjacent) { enqueue(candidate.first, neighbor); }
    }

    // 結合後も有効なポリゴンだけを出力し、各構成三角形から最終ポリゴンへの逆引きを作る。
    auto result = ConvexMerge{};
    result.trianglePolygons.resize(i_mesh.triangles.size(), kNoPolygon);
    for (const auto& polygon : polygons)
    {
        if (!polygon.active) { continue; }
        const auto index = static_cast<std::uint32_t>(result.polygons.size());
        result.polygons.push_back({polygon.corners, polygon.triangles, polygon.patch});
        for (const auto triangle : polygon.triangles) { result.trianglePolygons[triangle] = index; }
    }
    return result;
}
}
