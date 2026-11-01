/**
 * @file contour_triangulation.cpp
 * @brief 平坦な輪郭の選択済み頂点から三角形を構築する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "contour_triangulation.h"
#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace demo::navmesh::polygon
{
namespace
{
using Point = std::pair<std::size_t, std::size_t>;
}

/**
 * @brief 地形端の短いセルを考慮して、選択済み輪郭を三角形に分割する。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_flat 平坦な領域と選択済みの輪郭点。
 * @param i_patch 分割する平坦な領域の識別子。
 * @param i_metric 地形端を含むセル寸法。
 * @return 対象領域の三角形。
 * @throws std::invalid_argument 輪郭の連結または分割に失敗した場合。
 * @throws std::length_error 頂点数または座標が上限を超える場合。
 * @details 輪郭点を共有頂点へ統合し、穴は輪郭を横切らない補助線で外周につなぐ。
 * 境界辺の印を保ったまま耳切りで三角形化するため、後段の凸多角形結合が領域境界を越えない。
 */
ContourTriangulation TriangulateSelectedPatch(const voxelization::WalkableField& i_field,
                                              const contour::FlatContours& i_flat,
                                              const std::uint32_t i_patch,
                                              const GridMetric& i_metric)
{
    // 対象の平坦領域と輪郭データの対応を確認してから、三角形化用の作業データを用意する。
    if (i_patch == 0 || i_patch >= i_flat.floors.size() ||
        i_flat.contours.size() != i_flat.selectedEdges.size())
    {
        throw std::invalid_argument("三角形分割する領域または輪郭の対応が不正です。");
    }
    auto result = ContourTriangulation{};
    result.flat = i_flat;
    auto positions = std::vector<Vertex>{};
    auto vertices = std::map<Point, std::uint32_t>{};
    auto boundary = std::set<std::pair<std::uint32_t, std::uint32_t>>{};
    auto loops = std::vector<std::vector<std::uint32_t>>{};
    auto signs = std::vector<std::int64_t>{};
    auto outer = std::size_t{};
    auto outerCount = std::size_t{};

    // 選択された輪郭辺だけを取り出し、同じ格子位置を共有頂点として外周・穴の輪を作る。
    // 併せて元の輪郭辺を記録し、後で補助線と本来の境界を区別できるようにする。
    for (auto loopIndex = std::size_t{}; loopIndex < i_flat.contours.size(); ++loopIndex)
    {
        const auto& loop = i_flat.contours[loopIndex];
        if (loop.region != i_patch) { continue; }
        const auto& selected = i_flat.selectedEdges[loopIndex];
        if (selected.size() < (loop.signedAreaTwice == 0 ? 2U : 3U) ||
            !std::is_sorted(selected.begin(), selected.end()) ||
            std::adjacent_find(selected.begin(), selected.end()) != selected.end())
        {
            throw std::invalid_argument("輪郭の面積または選択された頂点が不正です。");
        }
        // 面積符号は輪の向きを示す。正の輪だけを外周として数え、他の輪は穴として扱う。
        if (loop.signedAreaTwice > 0) { outer = loops.size(); ++outerCount; }
        auto ring = std::vector<std::uint32_t>{};
        for (const auto edgeIndex : selected)
        {
            const auto& edge = loop.edges.at(edgeIndex);
            const auto& span = i_field.spans.at(edge.span);
            constexpr auto maxCoordinate = std::size_t{1000000000};
            if (span.x >= maxCoordinate || span.z >= maxCoordinate)
            {
                throw std::length_error("輪郭の座標が上限を超えました。");
            }
            auto point = Point{span.x, span.z};
            using voxelization::Direction;
            // 境界辺の向きに応じて、その辺の終点をセル原点からの整数格子座標へ戻す。
            switch (edge.direction)
            {
            case Direction::North: break;
            case Direction::East: ++point.first; break;
            case Direction::South: ++point.first; ++point.second; break;
            case Direction::West: ++point.second; break;
            default: throw std::invalid_argument("輪郭辺の方向が不正です。");
            }
            auto found = vertices.find(point);
            if (found == vertices.end())
            {
                if (result.vertices.size() >= static_cast<std::size_t>(kNoPolygon))
                {
                    throw std::length_error("輪郭の頂点数が上限を超えました。");
                }
                const auto id = static_cast<std::uint32_t>(result.vertices.size());
                // ポリゴン処理では局所格子座標を保持し、交差判定用には地形端を反映した実距離座標も保存する。
                result.vertices.push_back({static_cast<double>(point.first), static_cast<double>(point.second),
                                           i_flat.floors[i_patch]});
                positions.push_back({
                    GridCoordinate(static_cast<double>(i_metric.beginX) + static_cast<double>(point.first),
                                   i_metric.cellSize, i_metric.extentX),
                    GridCoordinate(static_cast<double>(i_metric.beginZ) + static_cast<double>(point.second),
                                   i_metric.cellSize, i_metric.extentZ),
                    i_flat.floors[i_patch]});
                found = vertices.emplace(point, id).first;
            }
            ring.push_back(found->second);
        }
        for (auto index = std::size_t{}; index < ring.size(); ++index)
        {
            const auto a = ring[index];
            const auto b = ring[(index + 1) % ring.size()];
            if (a == b) { throw std::invalid_argument("輪郭に長さのない辺があります。"); }
            boundary.insert(std::minmax(a, b));
        }
        loops.push_back(std::move(ring));
        signs.push_back(loop.signedAreaTwice);
    }
    if (outerCount != 1) { throw std::invalid_argument("領域の外周を一つに特定できません。"); }

    // 以下の判定関数を共有し、穴をつなぐ候補線が境界と交差しないことや耳の内外を一貫して確認する。
    const auto orientation = [&](const std::uint32_t i_a, const std::uint32_t i_b, const std::uint32_t i_c)
    {
        const auto& c = positions[i_c];
        return ExactOrientation(positions[i_a], positions[i_b], c.x, c.z);
    };
    const auto conflicts = [&](const std::uint32_t i_a, const std::uint32_t i_b,
                               const std::uint32_t i_c, const std::uint32_t i_d)
    {
        // 軸方向の範囲が重ならない辺は交差しない。範囲が重なる場合は向きの反転で交差を判定する。
        const auto& a = positions[i_a];
        const auto& b = positions[i_b];
        const auto& c = positions[i_c];
        const auto& d = positions[i_d];
        const auto left = std::max(std::min(a.x, b.x), std::min(c.x, d.x));
        const auto right = std::min(std::max(a.x, b.x), std::max(c.x, d.x));
        const auto top = std::max(std::min(a.z, b.z), std::min(c.z, d.z));
        const auto bottom = std::min(std::max(a.z, b.z), std::max(c.z, d.z));
        if (left > right || top > bottom) { return false; }
        const auto ac = orientation(i_a, i_b, i_c);
        const auto ad = orientation(i_a, i_b, i_d);
        const auto ca = orientation(i_c, i_d, i_a);
        const auto cb = orientation(i_c, i_d, i_b);
        if (ac * ad > 0 || ca * cb > 0) { return false; }
        const auto shares = i_a == i_c || i_a == i_d || i_b == i_c || i_b == i_d;
        if (shares && (ac != 0 || ad != 0 || (left == right && top == bottom))) { return false; }
        return true;
    };
    const auto inside = [&](const double i_x, const double i_z)
    {
        // 輪郭辺上は内部とみなさず、全ての輪を横切る水平線の交差回数で内外を判定する。
        auto within = false;
        for (const auto& loop : loops)
        {
            for (auto index = std::size_t{}; index < loop.size(); ++index)
            {
                const auto& a = positions[loop[index]];
                const auto& b = positions[loop[(index + 1) % loop.size()]];
                if (ExactOrientation(a, b, i_x, i_z) == 0 &&
                    std::min(a.x, b.x) <= i_x && i_x <= std::max(a.x, b.x) &&
                    std::min(a.z, b.z) <= i_z && i_z <= std::max(a.z, b.z))
                {
                    return false;
                }
                if ((a.z > i_z) != (b.z > i_z) &&
                    i_x < (b.x - a.x) * (i_z - a.z) / (b.z - a.z) + a.x)
                {
                    within = !within;
                }
            }
        }
        return within;
    };

    // 各穴について、内部を通り、既存輪郭・接続済みの輪と交差しない最短の補助線を選ぶ。
    // 補助線は輪を連結するためだけに使い、元の境界辺集合には追加しない。
    auto ring = loops[outer];
    for (auto holeIndex = std::size_t{}; holeIndex < loops.size(); ++holeIndex)
    {
        if (holeIndex == outer) { continue; }
        if (signs[holeIndex] > 0) { throw std::invalid_argument("領域の穴の向きが不正です。"); }
        const auto& hole = loops[holeIndex];
        auto bestOuter = ring.size();
        auto bestHole = hole.size();
        auto bestLength = std::numeric_limits<double>::infinity();
        // 外周と穴の全頂点対を候補にし、長さの短いものから成立条件を確認する。
        for (auto h = std::size_t{}; h < hole.size(); ++h)
        {
            for (auto o = std::size_t{}; o < ring.size(); ++o)
            {
                const auto a = ring[o];
                const auto b = hole[h];
                const auto& p = positions[a];
                const auto& q = positions[b];
                const auto dx = p.x - q.x;
                const auto dz = p.z - q.z;
                const auto length = dx * dx + dz * dz;
                if (length >= bestLength) { continue; }
                if (a != b)
                {
                    // 中点が領域内にあることを確認し、線分が輪郭や既に結合した輪を横切らないか調べる。
                    if (!inside((p.x + q.x) * 0.5, (p.z + q.z) * 0.5)) { continue; }
                    auto blocked = false;
                    for (const auto& other : loops)
                    {
                        for (auto side = std::size_t{}; side < other.size(); ++side)
                        {
                            if (conflicts(a, b, other[side], other[(side + 1) % other.size()]))
                            {
                                blocked = true;
                                break;
                            }
                        }
                        if (blocked) { break; }
                    }
                    if (!blocked)
                    {
                        for (auto side = std::size_t{}; side < ring.size(); ++side)
                        {
                            if (conflicts(a, b, ring[side], ring[(side + 1) % ring.size()]))
                            {
                                blocked = true;
                                break;
                            }
                        }
                    }
                    if (blocked) { continue; }
                }
                bestLength = length;
                bestOuter = o;
                bestHole = h;
            }
        }
        if (bestOuter == ring.size()) { throw std::invalid_argument("穴と外周を連結できません。"); }
        // 選んだ接続点を二度通る形で輪を連結し、穴を外周と同じ耳切り対象の輪へ組み込む。
        auto joined = std::vector<std::uint32_t>{};
        joined.reserve(ring.size() + hole.size() + 2);
        joined.insert(joined.end(), ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(bestOuter + 1));
        if (ring[bestOuter] != hole[bestHole]) { joined.push_back(hole[bestHole]); }
        for (auto offset = std::size_t{1}; offset <= hole.size(); ++offset)
        {
            joined.push_back(hole[(bestHole + offset) % hole.size()]);
        }
        if (ring[bestOuter] != hole[bestHole]) { joined.push_back(ring[bestOuter]); }
        joined.insert(joined.end(), ring.begin() + static_cast<std::ptrdiff_t>(bestOuter + 1), ring.end());
        ring = std::move(joined);
    }

    // 耳切りでできる三角形を上向きの頂点順で保存し、辺ごとに元の輪郭境界かを引き継ぐ。
    const auto emit = [&](const std::uint32_t i_a, const std::uint32_t i_b, const std::uint32_t i_c)
    {
        auto triangle = ContourTriangle{};
        triangle.vertices = {i_c, i_b, i_a};
        triangle.patch = i_patch;
        for (auto side = std::size_t{}; side < triangle.vertices.size(); ++side)
        {
            triangle.boundary[side] = boundary.contains(
                std::minmax(triangle.vertices[side], triangle.vertices[(side + 1) % triangle.vertices.size()]));
        }
        result.triangles.push_back(triangle);
    };

    // 反時計回りで面積のある耳を探し、その内部に他の輪頂点がない場合だけ三角形として切り出す。
    // 接点や補助線の両端は同じ頂点を再訪するため、候補の頂点自身は内点検査から除く。
    while (ring.size() > 3)
    {
        auto found = false;
        for (auto index = std::size_t{}; index < ring.size(); ++index)
        {
            const auto a = ring[(index + ring.size() - 1) % ring.size()];
            const auto b = ring[index];
            const auto c = ring[(index + 1) % ring.size()];
            if (orientation(a, b, c) <= 0) { continue; }
            // 切り出す耳の内側に頂点があると、その部分を取り残すため候補から除外する。
            auto occupied = false;
            for (const auto point : ring)
            {
                if (point == a || point == b || point == c) { continue; }
                if (orientation(a, b, point) >= 0 && orientation(b, c, point) >= 0 &&
                    orientation(c, a, point) >= 0)
                {
                    occupied = true;
                    break;
                }
            }
            if (occupied) { continue; }
            emit(a, b, c);
            ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(index));
            found = true;
            break;
        }
        if (!found) { throw std::invalid_argument("穴を含む輪郭の三角形分割に失敗しました。"); }
    }
    // 最後に残った三頂点も正の向き・面積を持つことを確認して出力する。
    if (orientation(ring[0], ring[1], ring[2]) <= 0)
    {
        throw std::invalid_argument("最後の三角形の向きまたは面積が不正です。");
    }
    emit(ring[0], ring[1], ring[2]);
    return result;
}

/**
 * @brief 外周と穴の選択済み頂点から一つの平坦な領域を三角形に分割する。
 * @param i_field 輪郭が参照する歩行面。
 * @param i_flat 平坦な領域と選択済みの輪郭点。
 * @param i_patch 分割する平坦な領域の識別子。
 * @return 対象領域の三角形。ほかの領域の三角形は含まない。
 * @throws std::invalid_argument 輪郭の連結または分割に失敗した場合。
 * @throws std::length_error 頂点数または座標が上限を超える場合。
 */
ContourTriangulation TriangulateSelectedPatch(const voxelization::WalkableField& i_field,
                                               const contour::FlatContours& i_flat,
                                               const std::uint32_t i_patch)
{
    // 単位セル幅と原点を使い、寸法を明示するオーバーロードへ処理を委譲する。
    return TriangulateSelectedPatch(i_field, i_flat, i_patch,
                                    {1, static_cast<double>(i_field.width),
                                     static_cast<double>(i_field.depth), 0, 0});
}

/**
 * @brief 地形端の短いセルを考慮して、全平坦領域の輪郭を三角形に分割する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @param i_metric 地形端を含むセル寸法。
 * @return 平坦な輪郭と三角形。
 * @throws std::invalid_argument 入力や輪郭が不正な場合。
 * @throws std::length_error 頂点または三角形が上限を超える場合。
 */
ContourTriangulation TriangulateContours(const voxelization::WalkableField& i_field,
                                         const std::vector<std::size_t>& i_coreSpans,
                                         const std::vector<std::uint8_t>& i_eroded,
                                         const region::Regions& i_regions, const GridMetric& i_metric)
{
    // タイルに属する平坦輪郭をまとめて作り、パッチごとに独立して三角形化する。
    auto result = ContourTriangulation{};
    result.flat = contour::BuildFlatContours(i_field, i_coreSpans, i_eroded, i_regions);
    for (auto index = std::size_t{1}; index <= result.flat.patches.count; ++index)
    {
        auto patch = TriangulateSelectedPatch(i_field, result.flat, static_cast<std::uint32_t>(index), i_metric);
        if (patch.vertices.size() >= static_cast<std::size_t>(kNoPolygon) - result.vertices.size() ||
            patch.triangles.size() >= static_cast<std::size_t>(kNoPolygon) - result.triangles.size())
        {
            throw std::length_error("三角形または頂点の数が上限を超えました。");
        }

        // パッチ内で始まる頂点添字を全体配列上の位置に合わせ、三角形参照を同じ量だけずらす。
        // これにより異なるパッチの頂点配列を統合しても、各三角形の参照が保たれる。
        const auto offset = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.insert(result.vertices.end(), patch.vertices.begin(), patch.vertices.end());
        for (auto& triangle : patch.triangles)
        {
            for (auto& vertex : triangle.vertices) { vertex += offset; }
            result.triangles.push_back(triangle);
        }
    }
    return result;
}

/**
 * @brief 同じ高さの輪郭から境界を保持する三角形を生成する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 平坦な輪郭と三角形。
 * @throws std::invalid_argument 入力や輪郭が不正な場合。
 * @throws std::length_error 頂点または三角形が上限を超える場合。
 * @details 直線上の不要点を省いた輪郭頂点から三角形を作り、穴や通行不能辺を境界として残す。
 * 単位格子を使用する。地形端が短い場合はセル寸法を受け取るオーバーロードを使用する。
 */
ContourTriangulation TriangulateContours(const voxelization::WalkableField& i_field,
                                         const std::vector<std::size_t>& i_coreSpans,
                                         const std::vector<std::uint8_t>& i_eroded,
                                         const region::Regions& i_regions)
{
    // 地形端に短いセルがない単位格子として、寸法を受け取るオーバーロードへ委譲する。
    return TriangulateContours(i_field, i_coreSpans, i_eroded, i_regions,
                               {1, static_cast<double>(i_field.width),
                                static_cast<double>(i_field.depth), 0, 0});
}

}
