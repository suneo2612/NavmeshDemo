/**
 * @file polygon_mesh.cpp
 * @brief 輪郭を三角形に分割し、凸多角形に結合する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "polygon_mesh.h"
#include "convex_merge.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace demo::navmesh::polygon
{
namespace
{
using Coordinate = std::int64_t;                         // 三角形とセルの向き判定に使う符号付き整数座標。
constexpr auto kMaxCoordinate = std::size_t{1000000000}; // 外積計算を符号付き64ビット整数で安全に行うための座標上限。

// 後続の bit_cast と指数・仮数の分解が IEEE 754 binary64 を前提とするため、
// 条件を満たさない環境では誤った幾何判定を避ける目的でコンパイルを停止する。
static_assert(sizeof(double) == sizeof(std::uint64_t) &&    // double と uint64_t が同じ64ビット幅であること。
              std::numeric_limits<double>::is_iec559 &&     // double が IEEE 754 形式であること。
              std::numeric_limits<double>::digits == 53);   // 仮数の有効桁数が binary64 の53ビットであること。

/**
 * @brief 安全な整数範囲内の格子座標で三点の外積を求める。
 * @param i_a 辺の始点。
 * @param i_b 辺の終点。
 * @param i_x 判定対象点の X 座標。
 * @param i_z 判定対象点の Z 座標。
 * @return 外積の符号を含む整数値。正なら左側、負なら右側、0 なら同一直線上。
 * @details 呼び出し側で座標上限を確認し、積と差を int64_t の範囲内に収める。
 */
Coordinate Cross(const Vertex& i_a, const Vertex& i_b, const Coordinate i_x, const Coordinate i_z)
{
    const auto ax = static_cast<Coordinate>(i_a.x);
    const auto az = static_cast<Coordinate>(i_a.z);
    return (static_cast<Coordinate>(i_b.x) - ax) * (i_z - az) -
           (static_cast<Coordinate>(i_b.z) - az) * (i_x - ax);
}

/**
 * @brief 二つの double の差と、丸めで失われた誤差を調べる。
 * @param i_a 被減数。
 * @param i_b 減数。
 * @return 差と、その差が厳密に表現できているかの組。
 * @throws std::overflow_error 差が有限値として表せない場合。
 * @details
 * 仮想的な加減算で残差を復元し、座標差を整数化する前に丸めの有無を判定する。
 * 差が厳密なら座標差を使うことで、大きな共通原点があっても小さな差分を保って整数化できる。
 * 差が丸められている場合は元の座標を使う経路へ切り替え、丸め誤差を含む差分から
 * 誤った向きを導かないようにする。
 */
std::pair<double, bool> Difference(const double i_a, const double i_b)
{
    // 減算誤差を復元し、通常の差の計算で失われた残差を求める。
    const auto value        =   i_a - i_b;
    const auto virtualB     =   i_a - value;
    const auto virtualA     =   value + virtualB;
    const auto remainder    =   (i_a - virtualA) + (virtualB - i_b);
    if (!std::isfinite(value))
    {
        throw std::overflow_error("三角形とセルの交差を正確に判定できません。");
    }
    return {value, remainder == 0};
}

/**
 * @brief 64 ビット符号なし整数同士の積を上位・下位 64 ビットで返す。
 * @param i_a 積の第一因子。
 * @param i_b 積の第二因子。
 * @return 128 ビット相当の積を {上位, 下位} に分けた組。
 * @details 32 ビット単位の部分積と桁上がりを合成し、
 * コンパイラー固有の 128 ビット整数型に依存せず積を比較できるようにする。
 */
std::pair<std::uint64_t, std::uint64_t> MagnitudeProduct(const std::uint64_t i_a, const std::uint64_t i_b)
{
    constexpr auto mask     =   std::uint64_t{0xffffffff};              // 積の各因子から下位32ビットを取り出すマスク。
    constexpr auto halfBits =   32;                                     // 64ビット値を上位・下位の32ビットに分ける幅。
    const auto low          =   (i_a & mask) * (i_b & mask);            // 下位同士の積。下位64ビットの基礎となる。
    const auto first        =   (i_a >> halfBits) * (i_b & mask);       // i_a上位とi_b下位の交差項。
    const auto second       =   (i_a & mask) * (i_b >> halfBits);       // i_a下位とi_b上位の交差項。
    const auto high         =   (i_a >> halfBits) * (i_b >> halfBits);  // 上位同士の積。上位64ビット側へ加算する。

    // low と交差項の下位32ビットを合わせ、64ビット境界を越える桁上がりも含める。
    const auto middle       =   (low >> halfBits) + (first & mask) + (second & mask);
    
    // 128ビット積を上位64ビットと下位64ビットに分けて組み立てる。
    // 上位には各部分積の上位と middle の桁上がりを加え、下位には middle と low の下位を連結する。
    return {high + (first >> halfBits) + (second >> halfBits) + (middle >> halfBits),
            (middle << halfBits) | (low & mask)};
}

/**
 * @brief 小数を含む三点の向きを二進整数の外積で正確に判定する。
 * @param i_a 辺の始点。
 * @param i_b 辺の終点。
 * @param i_x 判定対象点の X 座標。
 * @param i_z 判定対象点の Z 座標。
 * @return 反時計回りなら 1、時計回りなら -1、同一直線上なら 0。
 * @throws std::overflow_error 座標を安全な共通整数尺度に変換できない場合。
 * @details 差が厳密に表せる場合は差分を、丸めを含む場合は元座標を共通の
 * 2 の累乗で拡大して整数化する。外積の積は上位・下位に分けて比較し、
 * double の積算誤差で向きが反転することを避ける。
 */
int FractionalTurn(const Vertex& i_a, const Vertex& i_b, const double i_x, const double i_z)
{
    // 差分を先に計算し、丸めなしで表現できる場合は原点からの距離を小さく保つ。
    const auto dx   =   Difference(i_b.x, i_a.x);       // 辺の X 座標の差分。
    const auto dz   =   Difference(i_b.z, i_a.z);       // 辺の Z 座標の差分。
    const auto px   =   Difference(i_x, i_a.x);         // 判定対象点の X 座標との差分。
    const auto pz   =   Difference(i_z, i_a.z);         // 判定対象点の Z 座標との差分。
    const auto exact = dx.second && dz.second && px.second && pz.second;
    const auto values = exact ? std::array{dx.first, dz.first, px.first, pz.first, 0.0, 0.0} :
                                std::array{i_a.x, i_a.z, i_b.x, i_b.z, i_x, i_z};
    // 各値の二進指数と仮数から、非ゼロ値を整数にできる共通の 2 の累乗倍率を求める。
    auto shift = 0;
    for (const auto value : values)
    {
        if (value == 0) { continue; }
        const auto bits = std::bit_cast<std::uint64_t>(std::abs(value));
        const auto exponent = static_cast<int>((bits >> 52) & 0x7ff);
        const auto mantissa = (bits & ((std::uint64_t{1} << 52) - 1)) |
                              (exponent ? std::uint64_t{1} << 52 : 0);
        const auto power = (exponent ? exponent - 1075 : -1074) + std::countr_zero(mantissa);
        shift = std::max(shift, -power);
    }
    const auto limit = std::uint64_t{1} << (exact ? 62 : 61);
    auto scaled = std::array<Coordinate, 6>{};
    // 整数化後の値が安全域内で、倍率による丸めも起きていないことを確認する。
    for (auto index = std::size_t{}; index < values.size(); ++index)
    {
        const auto number = std::ldexp(values[index], shift);
        if (!std::isfinite(number) || std::abs(number) > static_cast<double>(limit) ||
            std::floor(number) != number)
        {
            throw std::overflow_error("三角形とセルの交差を正確に判定できません。");
        }
        scaled[index] = static_cast<Coordinate>(number);
    }
    if (!exact)
    {
        // 差分が丸められている場合は原点を引き、外積の平行移動不変性を使って桁を抑える。
        const auto originX = scaled[0];
        const auto originZ = scaled[1];
        scaled[0] = scaled[2] - originX;
        scaled[1] = scaled[3] - originZ;
        scaled[2] = scaled[4] - originX;
        scaled[3] = scaled[5] - originZ;
    }
    const auto sign = [](const Coordinate i_a, const Coordinate i_b)
    {
        if (i_a == 0 || i_b == 0) { return 0; }
        return (i_a < 0) == (i_b < 0) ? 1 : -1;
    };
    const auto magnitude = [](const Coordinate i_value)
    {
        return static_cast<std::uint64_t>(i_value < 0 ? -i_value : i_value);
    };
    const auto firstSign = sign(scaled[0], scaled[3]);
    const auto secondSign = sign(scaled[1], scaled[2]);
    // 二つの積の符号が違えば大小比較は不要。同符号なら 128 ビット相当の絶対値を比べる。
    if (firstSign != secondSign) { return firstSign ? firstSign : -secondSign; }
    if (firstSign == 0) { return 0; }
    const auto first = MagnitudeProduct(magnitude(scaled[0]), magnitude(scaled[3]));
    const auto second = MagnitudeProduct(magnitude(scaled[1]), magnitude(scaled[2]));
    if (first == second) { return 0; }
    return firstSign * (first > second ? 1 : -1);
}
}

/**
 * @brief 三点の XZ 座標から向きを正確に判定する。
 * @param i_a 最初の頂点。
 * @param i_b 二番目の頂点。
 * @param i_cx 三番目の頂点の X 座標。
 * @param i_cz 三番目の頂点の Z 座標。
 * @return 反時計回りなら 1、時計回りなら -1、同一直線上なら 0。
 * @throws std::overflow_error 値または正確な演算結果を表現できない場合。
 */
int ExactOrientation(const Vertex& i_a, const Vertex& i_b, const double i_cx, const double i_cz)
{
    // 無限値や非数を整数化処理へ渡さず、正確な向き判定が可能な座標だけを受け付ける。
    if (!std::isfinite(i_a.x) || !std::isfinite(i_a.z) || !std::isfinite(i_b.x) ||
        !std::isfinite(i_b.z) || !std::isfinite(i_cx) || !std::isfinite(i_cz))
    {
        throw std::overflow_error("三角形とセルの交差を正確に判定できません。");
    }
    return FractionalTurn(i_a, i_b, i_cx, i_cz);
}

/**
 * @brief 小数を含む共通格子の座標を地形の端で切り詰めた距離へ変換する。
 * @param i_coordinate 共通格子の座標。
 * @param i_cellSize セルの幅。
 * @param i_extent 地形の幅。
 * @return 地形の最小座標からの距離。
 * @throws std::invalid_argument 座標または寸法が不正な場合。
 * @throws std::overflow_error 距離を表現できない場合。
 */
double GridCoordinate(const double i_coordinate, const double i_cellSize, const double i_extent)
{
    // セル座標と寸法を検証し、整数部分からセルの開始位置を求める。
    if (!std::isfinite(i_coordinate) || i_coordinate < 0 || !std::isfinite(i_cellSize) || i_cellSize <= 0 ||
        !std::isfinite(i_extent) || i_extent < 0)
    {
        throw std::invalid_argument("格子座標または地形寸法が不正です。");
    }
    const auto whole = std::floor(i_coordinate);
    const auto begin = whole * i_cellSize;
    if (!std::isfinite(begin)) { throw std::overflow_error("格子の距離を表現できません。"); }
    if (begin >= i_extent) { return i_extent; }
    // 地形端では最後のセル幅を短くし、セル内の小数位置をその実寸に補間する。
    const auto end = std::min(i_extent, begin + i_cellSize);
    return std::lerp(begin, end, i_coordinate - whole);
}

/**
 * @brief 格子上の三角形とセルが正の面積で重なるか判定する。
 * @param i_triangle 時計回りまたは反時計回りの三角形。
 * @param i_x セルの X 座標。
 * @param i_z セルの Z 座標。
 * @return 面積をもって重なる場合は true。辺や点だけの接触は false。
 * @throws std::invalid_argument 三角形が退化している場合。
 * @throws std::length_error 座標が安全に扱える範囲を超える場合。
 * @throws std::overflow_error 小数座標の向きを正確に判定できない場合。
 */
bool TriangleOverlapsCell(const std::array<Vertex, 3>& i_triangle, const std::size_t i_x, const std::size_t i_z)
{
    // セルと三角形の座標範囲を先に検証し、以後の整数化と外積計算を安全な範囲に限定する。
    if (i_x >= kMaxCoordinate || i_z >= kMaxCoordinate)
    {
        throw std::length_error("三角形とセルの座標が上限を超えました。");
    }
    for (const auto& vertex : i_triangle)
    {
        if (!std::isfinite(vertex.x) || !std::isfinite(vertex.z) || vertex.x < 0 || vertex.z < 0)
        {
            throw std::invalid_argument("三角形の座標が不正です。");
        }
        if (vertex.x > kMaxCoordinate || vertex.z > kMaxCoordinate)
        {
            throw std::length_error("三角形とセルの座標が上限を超えました。");
        }
    }
    // 軸方向の外接範囲から候補セルとの重なりを早期に除外する。
    const auto minX = std::min({i_triangle[0].x, i_triangle[1].x, i_triangle[2].x});
    const auto maxX = std::max({i_triangle[0].x, i_triangle[1].x, i_triangle[2].x});
    const auto minZ = std::min({i_triangle[0].z, i_triangle[1].z, i_triangle[2].z});
    const auto maxZ = std::max({i_triangle[0].z, i_triangle[1].z, i_triangle[2].z});
    const auto integer = std::all_of(i_triangle.begin(), i_triangle.end(), [](const auto& i_vertex)
    {
        return std::floor(i_vertex.x) == i_vertex.x && std::floor(i_vertex.z) == i_vertex.z;
    });
    // 整数頂点は int64 の外積、小数頂点は厳密な二進整数判定を使う。
    const auto orientation = integer ?
        Cross(i_triangle[0], i_triangle[1], static_cast<Coordinate>(i_triangle[2].x),
              static_cast<Coordinate>(i_triangle[2].z)) :
        static_cast<Coordinate>(FractionalTurn(i_triangle[0], i_triangle[1], i_triangle[2].x, i_triangle[2].z));
    if (orientation == 0) { throw std::invalid_argument("面積のない三角形はセルと対応付けられません。"); }
    // 軸に沿った外接範囲で明らかに離れている組み合わせを先に除く。
    if (i_x >= maxX || i_x + 1 <= minX || i_z >= maxZ || i_z + 1 <= minZ) { return false; }

    // 三角形の各辺に対するセル四隅の向きを調べ、辺・点だけの接触を重なりに数えない。
    const auto x = static_cast<Coordinate>(i_x);
    const auto z = static_cast<Coordinate>(i_z);
    for (auto side = std::size_t{}; side < 3; ++side)
    {
        const auto& a = i_triangle[side];
        const auto& b = i_triangle[(side + 1) % 3];
        const auto crosses = integer ?
            std::array<Coordinate, 4>{Cross(a, b, x, z), Cross(a, b, x + 1, z),
                                      Cross(a, b, x + 1, z + 1), Cross(a, b, x, z + 1)} :
            std::array<Coordinate, 4>{FractionalTurn(a, b, static_cast<double>(x), static_cast<double>(z)),
                FractionalTurn(a, b, static_cast<double>(x + 1), static_cast<double>(z)),
                FractionalTurn(a, b, static_cast<double>(x + 1), static_cast<double>(z + 1)),
                FractionalTurn(a, b, static_cast<double>(x), static_cast<double>(z + 1))};
        if ((orientation > 0 && *std::max_element(crosses.begin(), crosses.end()) <= 0) ||
            (orientation < 0 && *std::min_element(crosses.begin(), crosses.end()) >= 0))
        {
            return false;
        }
    }
    return true;
}

namespace
{
/**
 * @brief ワールド距離の三角形とセル矩形の正の面積による重なりを判定する。
 * @param i_triangle ワールド距離へ変換済みの三角形。
 * @param i_minX 矩形の最小 X 座標。
 * @param i_minZ 矩形の最小 Z 座標。
 * @param i_maxX 矩形の最大 X 座標。
 * @param i_maxZ 矩形の最大 Z 座標。
 * @return 三角形と矩形が正の面積で重なる場合は true。
 * @throws std::invalid_argument 三角形が退化している場合。
 * @throws std::overflow_error 座標の向きを正確に判定できない場合。
 * @details 三角形の各辺を分離軸として矩形の四隅を検査する。
 * すべての角が辺の同じ外側にある場合は、辺または一点で触れているだけとして false にする。
 */
bool TriangleOverlapsBounds(const std::array<Vertex, 3>& i_triangle, const double i_minX, const double i_minZ,
                            const double i_maxX, const double i_maxZ)
{
    // 実距離座標で三角形の外接範囲と向きを求め、矩形と離れている場合は以降の判定を省く。
    const auto minX = std::min({i_triangle[0].x, i_triangle[1].x, i_triangle[2].x});
    const auto maxX = std::max({i_triangle[0].x, i_triangle[1].x, i_triangle[2].x});
    const auto minZ = std::min({i_triangle[0].z, i_triangle[1].z, i_triangle[2].z});
    const auto maxZ = std::max({i_triangle[0].z, i_triangle[1].z, i_triangle[2].z});
    const auto orientation = ExactOrientation(i_triangle[0], i_triangle[1], i_triangle[2].x, i_triangle[2].z);
    if (orientation == 0) { throw std::invalid_argument("面積のない三角形はセルと対応付けられません。"); }
    if (i_minX >= maxX || i_maxX <= minX || i_minZ >= maxZ || i_maxZ <= minZ) { return false; }
    // 三角形の各辺について矩形の四隅を向き判定し、同じ側に分離できる辺があれば交差しない。
    for (auto side = std::size_t{}; side < 3; ++side)
    {
        const auto& a = i_triangle[side];
        const auto& b = i_triangle[(side + 1) % 3];
        const auto crosses = std::array<int, 4>{ExactOrientation(a, b, i_minX, i_minZ),
                                                ExactOrientation(a, b, i_maxX, i_minZ),
                                                ExactOrientation(a, b, i_maxX, i_maxZ),
                                                ExactOrientation(a, b, i_minX, i_maxZ)};
        if ((orientation > 0 && *std::max_element(crosses.begin(), crosses.end()) <= 0) ||
            (orientation < 0 && *std::min_element(crosses.begin(), crosses.end()) >= 0))
        {
            return false;
        }
    }
    return true;
}
}

/**
 * @brief 構築済みの制約付き三角形を凸多角形に結合し、歩行面に対応付ける。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @param i_triangulation 対象の平坦な領域を重複なく覆う三角形。
 * @param i_metric 地形端を含む格子の実距離。
 * @return 凸多角形と、それを覆う三角形。
 * @throws std::invalid_argument 入力や三角形の接続が不正な場合。
 * @throws std::length_error 頂点またはポリゴンの数が上限を超える場合。
 * @throws std::overflow_error 座標や辺の長さを表現できない場合。
 * @details 呼び出し側は、三角形の重複がなく境界を越えていないことを保証する。
 */
PolygonMesh BuildPolygonMesh(const voxelization::WalkableField& i_field,
                             const std::vector<std::size_t>& i_coreSpans,
                             const std::vector<std::uint8_t>& i_eroded,
                             const region::Regions& i_regions, ContourTriangulation i_triangulation,
                             const GridMetric& i_metric)
{
    // 領域配列の整合性を検証し、三角形化結果のパッチ ID と面ごとの領域・高さを照合する。
    static_cast<void>(region::ValidateRegions(i_field, i_coreSpans, i_eroded, i_regions));
    const auto& flat = i_triangulation.flat;
    if (flat.patches.ids.size() != i_field.spans.size() ||
        flat.floors.size() != static_cast<std::size_t>(flat.patches.count) + 1 ||
        flat.sourceRegions.size() != flat.floors.size())
    {
        throw std::invalid_argument("三角形の連結領域が不正です。");
    }
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        const auto patch = flat.patches.ids[index];
        if (patch > flat.patches.count || (patch == 0) != (i_regions.ids[index] == 0) ||
            (patch && (flat.sourceRegions[patch] != i_regions.ids[index] ||
                       flat.floors[patch] != i_field.spans[index].floor)))
        {
            throw std::invalid_argument("三角形と歩行面の領域または高さが一致しません。");
        }
    }

    // 輪郭境界を維持して隣接三角形を凸形状へまとめ、各ポリゴンに元の領域 ID を戻す。
    auto merged = MergeContourTriangles(i_triangulation, i_metric);
    auto result = PolygonMesh{};
    result.spanPolygons.resize(i_field.spans.size());
    result.polygons.reserve(merged.polygons.size());
    result.triangles.reserve(i_triangulation.triangles.size());
    for (const auto& polygon : merged.polygons)
    {
        result.polygons.push_back({polygon.vertices, flat.sourceRegions.at(polygon.patch)});
    }

    // 各三角形について、まず頂点から走査するセル範囲を絞り、ポリゴン所属と境界情報を保存する。
    // 歩行面への対応付けは三角形の頂点位置だけでなく、セルとの正の面積の交差で決定する。
    for (auto index = std::size_t{}; index < i_triangulation.triangles.size(); ++index)
    {
        const auto& triangle = i_triangulation.triangles[index];
        const auto polygon = merged.trianglePolygons.at(index);
        result.triangles.push_back({triangle.vertices, polygon, triangle.boundary});
        auto x0 = static_cast<double>(i_field.width);
        auto x1 = 0.0;
        auto z0 = static_cast<double>(i_field.depth);
        auto z1 = 0.0;
        auto points = std::array<Vertex, 3>{};
        for (auto corner = std::size_t{}; corner < triangle.vertices.size(); ++corner)
        {
            const auto& point = i_triangulation.vertices.at(triangle.vertices[corner]);
            points[corner] = point;
            x0 = std::min(x0, point.x);
            x1 = std::max(x1, point.x);
            z0 = std::min(z0, point.z);
            z1 = std::max(z1, point.z);
        }
        if (!std::isfinite(x0) || !std::isfinite(x1) || !std::isfinite(z0) || !std::isfinite(z1) ||
            x0 < 0 || z0 < 0 || x0 >= x1 || x1 > i_field.width || z0 >= z1 || z1 > i_field.depth)
        {
            throw std::logic_error("三角形の走査範囲が不正です。");
        }

        // 端に短いセルがある境界を横切ると、格子上の直線とワールド上の直線は一致しない。
        // その境界を横切る三角形だけ実距離座標に変換し、それ以外は格子座標で判定する。
        const auto shortX = std::floor(i_metric.extentX / i_metric.cellSize);
        const auto shortZ = std::floor(i_metric.extentZ / i_metric.cellSize);
        const auto transitionX = shortX - static_cast<double>(i_metric.beginX);
        const auto transitionZ = shortZ - static_cast<double>(i_metric.beginZ);
        const auto physical = (shortX * i_metric.cellSize < i_metric.extentX && x0 < transitionX && x1 > transitionX) ||
                              (shortZ * i_metric.cellSize < i_metric.extentZ && z0 < transitionZ && z1 > transitionZ);
        auto worldPoints = points;
        if (physical)
        {
            for (auto& point : worldPoints)
            {
                point.x = GridCoordinate(static_cast<double>(i_metric.beginX) + point.x, i_metric.cellSize, i_metric.extentX);
                point.z = GridCoordinate(static_cast<double>(i_metric.beginZ) + point.z, i_metric.cellSize, i_metric.extentZ);
            }
        }
        for (auto z = static_cast<std::size_t>(std::floor(z0));
             z < static_cast<std::size_t>(std::ceil(z1)); ++z)
        {
            for (auto x = static_cast<std::size_t>(std::floor(x0));
                 x < static_cast<std::size_t>(std::ceil(x1)); ++x)
            {
                const auto overlaps = physical ?
                    TriangleOverlapsBounds(worldPoints,
                        GridCoordinate(static_cast<double>(i_metric.beginX) + static_cast<double>(x), i_metric.cellSize, i_metric.extentX),
                        GridCoordinate(static_cast<double>(i_metric.beginZ) + static_cast<double>(z), i_metric.cellSize, i_metric.extentZ),
                        GridCoordinate(static_cast<double>(i_metric.beginX) + static_cast<double>(x) + 1, i_metric.cellSize, i_metric.extentX),
                        GridCoordinate(static_cast<double>(i_metric.beginZ) + static_cast<double>(z) + 1, i_metric.cellSize, i_metric.extentZ)) :
                    TriangleOverlapsCell(points, x, z);
                if (!overlaps) { continue; }
                // 同じセル内に高さ違いの面が複数ある場合、三角形と同じ平坦パッチの面だけを選ぶ。
                const auto cell = z * i_field.width + x;
                auto found = false;
                // 三角形の領域と一致する歩行面だけにポリゴンを対応付ける。
                for (auto span = i_field.offsets[cell]; span < i_field.offsets[cell + 1]; ++span)
                {
                    if (flat.patches.ids[span] == triangle.patch)
                    {
                        result.spanPolygons[span].push_back(polygon);
                        found = true;
                        break;
                    }
                }
                if (!found) { throw std::logic_error("ポリゴンに対応する歩行面がありません。"); }
            }
        }
    }
    // 重なった複数三角形から同じポリゴンが重複登録される場合を整理し、
    // 領域に属する面は必ずポリゴンに覆われ、領域外の面は覆われないことを確認する。
    for (auto index = std::size_t{}; index < result.spanPolygons.size(); ++index)
    {
        auto& polygons = result.spanPolygons[index];
        std::sort(polygons.begin(), polygons.end());
        polygons.erase(std::unique(polygons.begin(), polygons.end()), polygons.end());
        if (polygons.empty() != (i_regions.ids[index] == 0))
        {
            throw std::logic_error("歩行面とポリゴンの被覆が一致しません。");
        }
    }
    result.vertices = std::move(i_triangulation.vertices);
    return result;
}

/**
 * @brief 所有面を穴を埋めずに分割し、三角形と凸多角形を作る。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @param i_metric 地形端を含む格子の実距離。
 * @return 凸多角形と、それを覆う三角形。
 * @throws std::invalid_argument 入力や領域が不正な場合。
 * @throws std::length_error 頂点またはポリゴンの数が上限を超える場合。
 * @throws std::overflow_error 座標や辺の長さを表現できない場合。
 * @details 同じ領域・高さで四方向に接続する面の輪郭から三角形を作る。
 * 輪郭の穴、接点、通行できない共通辺を越えて結合しない。
 * 高さの異なる面は結合せず、段差は別のポリゴンとなる。
 * ポリゴン間の接続はタイル全体の構築後に別途生成する。
 */
PolygonMesh BuildPolygonMesh(const voxelization::WalkableField& i_field,
                             const std::vector<std::size_t>& i_coreSpans,
                             const std::vector<std::uint8_t>& i_eroded,
                             const region::Regions& i_regions, const GridMetric& i_metric)
{
    // 領域輪郭から制約付き三角形を生成し、事前生成済み三角形を受け取る共通処理へ渡す。
    auto triangulation = TriangulateContours(i_field, i_coreSpans, i_eroded, i_regions, i_metric);
    return BuildPolygonMesh(i_field, i_coreSpans, i_eroded, i_regions, std::move(triangulation), i_metric);
}

/**
 * @brief 単位格子を使って所有面から凸多角形を生成する。
 * @param i_field 侵食面への接続を除いた歩行面。
 * @param i_coreSpans タイルの所有面の添字。
 * @param i_eroded 面ごとに侵食された場合は 1。
 * @param i_regions 所有面に対応する領域。
 * @return 凸多角形と、それを覆う三角形。
 */
PolygonMesh BuildPolygonMesh(const voxelization::WalkableField& i_field,
                             const std::vector<std::size_t>& i_coreSpans,
                             const std::vector<std::uint8_t>& i_eroded,
                             const region::Regions& i_regions)
{
    // このオーバーロードは単位格子を前提とするため、地形寸法と原点を含む GridMetric を作って委譲する。
    return BuildPolygonMesh(i_field, i_coreSpans, i_eroded, i_regions,
                            {1, static_cast<double>(i_field.width), static_cast<double>(i_field.depth), 0, 0});
}
}
