/**
 * @file heightfield.cpp
 * @brief 地形をセル単位の高さ方向の占有区間に変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "heightfield.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace demo::navmesh::voxelization
{
namespace
{
// X/Z 方向それぞれのセル数と、格子全体のセル数に対する上限。
constexpr auto kMaxColumns  =   std::size_t{1000000};
// 高さ場全体に生成できる占有区間の総数に対する上限。
constexpr auto kMaxSpans    =   std::size_t{8000000};
// 高さ方向のセル数の上限。占有区間の上端を最低 1 セル確保する加算に備え、型の最大値より 1 小さくする。
constexpr auto kMaxHeight   =   std::numeric_limits<std::uint32_t>::max() - 1U;

/**
 *  @brief 3次元座標を表す構造体。
 */
struct Point
{
    double x; // X 座標。
    double y; // Y 座標。
    double z; // Z 座標。
};

/**
 * @brief 多角形を指定した X または Z 座標の境界で片側に切り取る。
 * @param i_polygon 切り取り対象の多角形の頂点。
 * @param i_xAxis true の場合は X 座標、false の場合は Z 座標を境界判定に使う。
 * @param i_limit 切り取り境界の座標。
 * @param i_keepGreater true の場合は境界以上、false の場合は境界以下を残す。
 * @return 境界で切り取った後の多角形の頂点。境界との交点も含む。
 * @details 各辺の両端が境界の異なる側にある場合、辺上の交点を線形補間で求める。
 * 境界上の頂点は、残す側に含める。
 */
std::vector<Point> ClipPolygonToAxisHalfPlane(const std::vector<Point>& i_polygon,
                                              const bool i_xAxis,
                                              const double i_limit,
                                              const bool i_keepGreater)
{
    auto result = std::vector<Point>{};

    // 切り取る頂点がなければ、空の結果を返す。
    if (i_polygon.empty()) { return result; }

    // 指定された軸の座標を取り出し、各頂点が境界の残す側にあるか判定する。
    const auto xzCoordinate =   [i_xAxis](const Point& i_point) { return i_xAxis ? i_point.x : i_point.z; };
    const auto inside       =   [&](const Point& i_point)
    {
        return i_keepGreater ? xzCoordinate(i_point) >= i_limit : xzCoordinate(i_point) <= i_limit;
    };

    // 最初の辺も処理できるよう、最後の頂点を最初の頂点の前にある頂点として扱う。
    auto previous       =   i_polygon.back();
    auto previousInside =   inside(previous);
    for (const auto& current : i_polygon)
    {
        // 前の頂点から現在の頂点へ続く辺について、現在の頂点が境界の残す側にあるか調べる。
        const auto currentInside = inside(current);

        // 辺が境界をまたぐ場合、残す側と捨てる側の切り替わり位置を求める。
        if (previousInside != currentInside)
        {
            // 境界までの距離の比率を使い、交点の X/Y/Z 座標を辺上で補間する。
            const auto t    =   (i_limit - xzCoordinate(previous)) / (xzCoordinate(current) - xzCoordinate(previous));
            result.push_back({previous.x + (current.x - previous.x) * t,
                              previous.y + (current.y - previous.y) * t,
                              previous.z + (current.z - previous.z) * t});
        }

        // 現在の頂点が残す側にあれば結果に加える。
        if (currentInside) { result.push_back(current); }

        // 次の辺では、現在の頂点を前の頂点として判定する。
        previous = current;
        previousInside = currentInside;
    }
    return result;
}

/**
 * @brief 設定値と地形の座標を検証する。
 * @param i_terrain 入力地形。
 * @param i_settings 構築設定。
 * @throws std::invalid_argument 設定、地形範囲、三角形、または頂点が不正な場合。
 * @details 全軸の地形範囲と各頂点を検査し、三角形が存在して XZ 平面上に幅があることを確認する。
 */
void Validate(const Terrain& i_terrain, const RasterSettings& i_settings)
{
    ValidateRasterSettings(i_settings);

    // 地形の範囲が有限であることを確認する。
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (!std::isfinite(i_terrain.bounds.minimum[axis]) ||
            !std::isfinite(i_terrain.bounds.maximum[axis]) ||
            i_terrain.bounds.minimum[axis] > i_terrain.bounds.maximum[axis])
        {
            throw std::invalid_argument("地形の範囲が不正です。");
        }
    }

    // 地形に三角形が存在し、XZ 平面上に幅があることを確認する。
    if (i_terrain.triangles.empty() ||                                  // 地形モデルの三角形が存在しない
        i_terrain.bounds.minimum.x == i_terrain.bounds.maximum.x ||     // X 軸方向の幅がない
        i_terrain.bounds.minimum.z == i_terrain.bounds.maximum.z)       // Z 軸方向の幅がない
    {
        throw std::invalid_argument("ボクセル化には XZ の幅を持つ地形が必要です。");
    }
    for (const auto& triangle : i_terrain.triangles)
    {
        for (const auto& vertex : triangle)
        {
            for (auto axis = std::size_t{}; axis < 3; ++axis)
            {
                // 頂点が有限であり、地形のAABB 内に収まっていることを確認する。
                if (!std::isfinite(vertex[axis]) ||
                    vertex[axis] < i_terrain.bounds.minimum[axis] ||
                    vertex[axis] > i_terrain.bounds.maximum[axis])
                {
                    throw std::invalid_argument("地形の頂点が範囲外または非有限です。");
                }
            }
        }
    }
}

/**
 * @brief 座標からセル添字を求め、範囲外の値は端のセルに割り当てる。
 * @param i_value   座標。
 * @param i_origin  格子の最小座標。
 * @param i_size    セル幅。
 * @param i_count   セル数。
 * @return  範囲内のセル添字。
 * @details 座標が地形範囲の外側にある場合は、最初または最後のセルに割り当てる。
 */
std::size_t ToCellIndex(const double i_value, const double i_origin, const double i_size, const std::size_t i_count)
{
    const auto  value   =   std::floor((i_value - i_origin) / i_size);
    if (value <= 0) { return 0; }
    if (value >= static_cast<double>(i_count)) { return i_count - 1; }
    return static_cast<std::size_t>(value);
}

/**
 * @brief 隣接または重複する区間を高さ順に統合する。
 * @param io_column 一つのセルにある占有区間。
 * @details 区間を下端の高さ順に並べ、重なる区間や接する区間を一つにまとめる。
 * 統合後の上端が最も高い区間の歩行可否を保ち、上端が同じ場合は歩行可能な状態を引き継ぐ。
 */
void Merge(std::vector<SolidSpan>& io_column)
{
    // 区間を下端の高さ順に並べる。
    std::sort(
        io_column.begin(),
        io_column.end(),
        [](const auto& i_a, const auto& i_b)
    {
        return  i_a.minimum < i_b.minimum ||                                // スパンの下端が低いこと
                (i_a.minimum == i_b.minimum && i_a.maximum < i_b.maximum);  // 下端が同じ場合は上端が低いこと
    });

    auto count = std::size_t{};
    for (const auto span : io_column)
    {
        // 初回か下端が前の区間より高い場合は新しい区間として追加する。
        if (count == 0 || span.minimum > io_column[count - 1].maximum)
        {
            io_column[count++] = span;
            continue;
        }

        // 上端が前の区画より高くかつ下端が前の区間と重なる場合は統合する。
        auto& current = io_column[count - 1];
        if (span.maximum > current.maximum)
        {
            current.maximum = span.maximum;
            current.walkableTop = span.walkableTop;
        }
        else if (span.maximum == current.maximum) 
        { 
            // 上端が同じ場合は歩行可能な状態を引き継ぐ。
            current.walkableTop = current.walkableTop || span.walkableTop; 
        }
    }
    io_column.resize(count);
}
}

/**
 * @brief ボクセル化のセル寸法と最大傾斜角を検証する。
 * @param i_settings 検証するボクセル化設定。
 * @throws std::invalid_argument セル幅・高さが有限の正数でない、または傾斜角が 0 以上 90 未満でない場合。
 */
void ValidateRasterSettings(const RasterSettings& i_settings)
{
    if (!std::isfinite(i_settings.cellSize)         ||  i_settings.cellSize <= 0 ||         // セル幅が有限の正数でない
        !std::isfinite(i_settings.cellHeight)       ||  i_settings.cellHeight <= 0 ||       // セル高さが有限の正数でない
        !std::isfinite(i_settings.maxSlopeDegrees)  ||  i_settings.maxSlopeDegrees < 0 ||   // 最大傾斜角が有限の0以上でない、または0度未満
        i_settings.maxSlopeDegrees >= 90)                                                   // 最大傾斜角が 90 未満でない
    {
        throw std::invalid_argument("ボクセルの大きさまたは最大傾斜角が不正です。");
    }
}

/**
 * @brief 指定した XZ セルにある地形スパンを取得する。
 * @param i_x X 方向のセル添字。
 * @param i_z Z 方向のセル添字。
 * @return 指定セルの地形スパンを高さ順に格納した配列への参照。
 * @throws std::out_of_range セル添字が高さ場の範囲外の場合。
 */
const std::vector<SolidSpan>& Heightfield::Column(const std::size_t i_x, const std::size_t i_z) const
{
    if (i_x >= width || i_z >= depth) { throw std::out_of_range("ボクセルのセル添字が範囲外です。"); }
    return columns.at(i_z * width + i_x);
}

/**
 * @brief 地形をセルごとの高さ方向の占有区間に変換する。
 * @param i_terrain ボクセル化する地形。
 * @param i_settings セル寸法と歩行可能な最大傾斜角。
 * @return 地形の各 XZ セルに占有区間を持つ高さ場。
 * @throws std::invalid_argument 地形、頂点、または設定が不正な場合。
 * @throws std::length_error セル数または占有区間数が上限を超える場合。
 * @details 三角形を交差するセルごとに切り取り、Y 座標をセル単位の半開区間へ変換する。
 * 区間は地形範囲の最低 Y を基準とし、傾斜条件を満たす三角形の上面を歩行可能として記録する。
 */
Heightfield Rasterize(const Terrain& i_terrain, const RasterSettings& i_settings)
{
    Validate(i_terrain, i_settings);

    // 地形のAABBからボクセル格子の幅・奥行・高さを計算する。
    const auto width = std::ceil((static_cast<double>(i_terrain.bounds.maximum.x) -
        i_terrain.bounds.minimum.x) / i_settings.cellSize);
    const auto depth = std::ceil((static_cast<double>(i_terrain.bounds.maximum.z) -
        i_terrain.bounds.minimum.z) / i_settings.cellSize);
    const auto height = (static_cast<double>(i_terrain.bounds.maximum.y) -
        i_terrain.bounds.minimum.y) / i_settings.cellHeight;
    if (!std::isfinite(width) || !std::isfinite(depth) || !std::isfinite(height) || // 有限か確認
        width < 1 || depth < 1 ||           // 1 以上か確認
        width > kMaxColumns ||              // 横のボクセル数が上限を超えていないか確認
        depth > kMaxColumns ||              // 奥行のボクセル数が上限を超えていないか確認
        width * depth > kMaxColumns ||      // 総セル数が上限を超えていないか確認
        height > kMaxHeight)                // 高さが上限を超えていないか確認
    {
        throw std::length_error("ボクセル格子の大きさが上限を超えました。");
    }

    // 地形ボクセル化結果の戻り値を初期化
    auto result = Heightfield{i_terrain.bounds,                 // 地形のAABB         
                              i_settings,                       // 地形のラスタライズ設定
                              static_cast<std::size_t>(width),  // 幅
                              static_cast<std::size_t>(depth),  // 奥行
                              {}};                              // 占有区間の初期値
    result.columns.resize(result.width * result.depth);     // 地形の各セルの地形区間を格納するための領域を確保
    
    // 各三角形の XZ 範囲だけを調べ、セル境界で切り取った高さを占有区間として保存する。
    auto spanCount          =   std::size_t{};

    // 傾斜の閾値を計算
    const auto threshold    =   std::cos(static_cast<double>(i_settings.maxSlopeDegrees) * std::numbers::pi_v<double> / 180.0);
    for (const auto& triangle : i_terrain.triangles)
    {
        // 三角形の頂点を取得
        const auto a    =   Point{triangle[0].x, triangle[0].y, triangle[0].z};      // 三角形の頂点 A
        const auto b    =   Point{triangle[1].x, triangle[1].y, triangle[1].z};      // 三角形の頂点 B
        const auto c    =   Point{triangle[2].x, triangle[2].y, triangle[2].z};      // 三角形の頂点 C

        // 辺 AB と AC の成分を計算
        const auto ux   =   b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;                          // 辺 AB の成分
        const auto vx   =   c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;                          // 辺 AC の成分
        const auto nx   =   uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;  // 法線ベクトルの成分
        const auto normalLength = std::hypot(nx, ny, nz);                                       // 法線ベクトルの長さ
        // 法線ベクトルの長さが 0 の場合は三角形が閉じているので無視する。
        if (normalLength == 0) { continue; }
        const auto walkable = ny > 0 && ny / normalLength >= threshold;                         // 歩ける傾斜かどうか
        
        // 最小座標がセル境界上なら、境界よりわずかに小さい値で添字を求め、
        // 境界の手前側にあるセルも候補に含める。
        const auto firstX = ToCellIndex(std::nextafter(std::min({a.x, b.x, c.x}),
                                                       -std::numeric_limits<double>::infinity()),
                                        result.bounds.minimum.x,
                                        i_settings.cellSize,
                                        result.width);
        const auto lastX = ToCellIndex(std::max({a.x, b.x, c.x}),
                                       result.bounds.minimum.x,
                                       i_settings.cellSize,
                                       result.width);
        const auto firstZ = ToCellIndex(std::nextafter(std::min({a.z, b.z, c.z}),
                                                       -std::numeric_limits<double>::infinity()),
                                        result.bounds.minimum.z,
                                        i_settings.cellSize,
                                        result.depth);
        const auto lastZ = ToCellIndex(std::max({a.z, b.z, c.z}), result.bounds.minimum.z, i_settings.cellSize, result.depth);
        for (auto z = firstZ; z <= lastZ; ++z)
        {
            const auto minZ = static_cast<double>(result.bounds.minimum.z) + z * static_cast<double>(i_settings.cellSize);
            for (auto x = firstX; x <= lastX; ++x)
            {
                const auto minX = static_cast<double>(result.bounds.minimum.x) + x * static_cast<double>(i_settings.cellSize);

                // クリッピング用の多角形を作成
                auto polygon = std::vector<Point>{a, b, c};

                // セル中で三角形が占めるXZ平面上の領域にクリップ
                polygon = ClipPolygonToAxisHalfPlane(polygon, true, minX, true);                            // X軸の最小境界でクリップ
                polygon = ClipPolygonToAxisHalfPlane(polygon, true, minX + i_settings.cellSize, false);     // X軸の最大境界でクリップ
                polygon = ClipPolygonToAxisHalfPlane(polygon, false, minZ, true);                           // Z軸の最小境界でクリップ
                polygon = ClipPolygonToAxisHalfPlane(polygon, false, minZ + i_settings.cellSize, false);    // Z軸の最大境界でクリップ
                if (polygon.empty()) { continue; }
                
                // セル内でのポリゴンの最大・最小Y座標を求める
                auto minY = polygon.front().y;
                auto maxY = minY;
                for (const auto& point : polygon)
                {
                    minY = std::min(minY, point.y);
                    maxY = std::max(maxY, point.y);
                }

                // 底面のY座標をセルの高さ方向に変換してボクセル区間を求める
                const auto lo = static_cast<std::uint32_t>(std::floor(
                    std::max(0.0, (minY - result.bounds.minimum.y) / i_settings.cellHeight)
                ));

                // 天面のY座標をセルの高さ方向に変換してボクセル区間を求める
                const auto hi = static_cast<std::uint32_t>(std::ceil(
                    std::max(0.0, (maxY - result.bounds.minimum.y) / i_settings.cellHeight)
                ));

                // ボクセル区間が上限を超えないかチェックしてから地形Spanを追加
                if (++spanCount > kMaxSpans) { throw std::length_error("ボクセル区間数が上限を超えました。"); }
                result.columns[z * result.width + x].push_back({lo, std::max(lo + 1U, hi), walkable});
            }
        }
    }

    // 各カラムのスパンをマージして重複を排除
    for (auto& column : result.columns) { Merge(column); }
    return result;
}
}
