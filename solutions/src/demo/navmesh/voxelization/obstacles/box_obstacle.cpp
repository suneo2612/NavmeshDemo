/**
 * @file box_obstacle.cpp
 * @brief 箱型障害物の範囲判定と高さ場への重ね合わせを実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "box_obstacle.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::voxelization
{
namespace
{
constexpr auto kMaxHeight = std::numeric_limits<std::uint32_t>::max() - 1U;

/**
 * @brief 共通格子の寸法とボクセル幅を確認する。
 * @param i_field 対象の高さ場。
 */
void ValidateGrid(const Heightfield& i_field)
{
    // セル配列の大きさとボクセル寸法を確認し、無効な格子で添字計算しないようにする。
    if (i_field.width == 0 || i_field.depth == 0 ||
        i_field.width > std::numeric_limits<std::size_t>::max() / i_field.depth ||
        i_field.columns.size() != i_field.width * i_field.depth ||
        !std::isfinite(i_field.settings.cellSize) || i_field.settings.cellSize <= 0 ||
        !std::isfinite(i_field.settings.cellHeight) || i_field.settings.cellHeight <= 0)
    {
        throw std::invalid_argument("障害物を適用する高さ場が不正です。");
    }
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (!std::isfinite(i_field.bounds.minimum[axis]) ||
            !std::isfinite(i_field.bounds.maximum[axis]) ||
            i_field.bounds.minimum[axis] > i_field.bounds.maximum[axis])
        {
            throw std::invalid_argument("高さ場の地形範囲が不正です。");
        }
    }
    // XZ のワールド範囲をセル幅で割ったセル数が、格子の幅・奥行きと一致することを確認する。
    if (i_field.bounds.minimum.x == i_field.bounds.maximum.x ||
        i_field.bounds.minimum.z == i_field.bounds.maximum.z ||
        std::ceil((static_cast<double>(i_field.bounds.maximum.x) - i_field.bounds.minimum.x) /
                  i_field.settings.cellSize) != i_field.width ||
        std::ceil((static_cast<double>(i_field.bounds.maximum.z) - i_field.bounds.minimum.z) /
                  i_field.settings.cellSize) != i_field.depth)
    {
        throw std::invalid_argument("高さ場の地形範囲とセル数が一致しません。");
    }
}

/**
 * @brief 共通格子内の空でないセル範囲を確認する。
 * @param i_field 対象の高さ場。
 * @param i_rect 判定する範囲。
 */
void ValidateRect(const Heightfield& i_field, const CellRect& i_rect)
{
    // 半開区間で空でなく、終端も格子の幅・奥行きを越えない範囲だけを受け付ける。
    if (i_rect.beginX >= i_rect.endX || i_rect.beginZ >= i_rect.endZ ||
        i_rect.endX > i_field.width || i_rect.endZ > i_field.depth)
    {
        throw std::invalid_argument("障害物を適用するセル範囲が不正です。");
    }
}

/**
 * @brief 内側のセル範囲が外側のセル範囲に含まれるか判定する。
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
 * @brief 指定セル範囲と箱の XZ 上の重なりを調べる。
 * @param i_field 共通格子。
 * @param i_rect 半開のセル範囲。
 * @param i_bounds 障害物の範囲。
 * @return 範囲が接触する場合は true。
 */
bool Touches(const Heightfield& i_field, const CellRect& i_rect, const Bounds& i_bounds)
{
    // セル範囲をワールド座標へ変換し、地形外へはみ出す終端を地形境界で切り詰める。
    const auto size = static_cast<double>(i_field.settings.cellSize);
    const auto minX = static_cast<double>(i_field.bounds.minimum.x) + i_rect.beginX * size;
    const auto minZ = static_cast<double>(i_field.bounds.minimum.z) + i_rect.beginZ * size;
    const auto maxX = std::min(static_cast<double>(i_field.bounds.maximum.x),
                               static_cast<double>(i_field.bounds.minimum.x) + i_rect.endX * size);
    const auto maxZ = std::min(static_cast<double>(i_field.bounds.maximum.z),
                               static_cast<double>(i_field.bounds.minimum.z) + i_rect.endZ * size);
    // XZ 平面で範囲が触れるか調べる。Y は高さ方向の占有や頭上障害物に関わるため使わない。
    return i_bounds.maximum.x >= minX && i_bounds.minimum.x <= maxX &&
           i_bounds.maximum.z >= minZ && i_bounds.minimum.z <= maxZ;
}

/**
 * @brief 一つの占有区間を既存の高さ順区間に統合する。
 * @param io_column 更新するセル。
 * @param i_obstacle 障害物の占有区間。
 */
void MergeObstacle(std::vector<SolidSpan>& io_column, const SolidSpan& i_obstacle)
{
    // 新しい占有区間を追加し、高さの下端・上端の順に並べる。
    io_column.push_back(i_obstacle);
    std::sort(io_column.begin(), io_column.end(), [](const auto& i_a, const auto& i_b)
    {
        return i_a.minimum < i_b.minimum || (i_a.minimum == i_b.minimum && i_a.maximum < i_b.maximum);
    });
    // 重なった区間を前から統合し、互いに重ならない高さ区間の列に戻す。
    auto count = std::size_t{};
    for (const auto span : io_column)
    {
        if (count == 0 || span.minimum > io_column[count - 1].maximum)
        {
            io_column[count++] = span;
            continue;
        }
        auto& current = io_column[count - 1];
        if (span.maximum > current.maximum)
        {
            current.maximum = span.maximum;
            current.walkableTop = span.walkableTop;
        }
        else if (span.maximum == current.maximum)
        {
            current.walkableTop = current.walkableTop && span.walkableTop;
        }
    }
    // 統合後に残った区間数へ縮め、末尾に残った古い要素を有効データから外す。
    io_column.resize(count);
}
}

/**
 * @brief 障害物の範囲を検証する。
 * @param i_bounds 箱の範囲。
 * @throws std::invalid_argument 非有限値または幅のない箱の場合。
 */
void ValidateBoxObstacle(const Bounds& i_bounds)
{
    // X、Y、Z の全軸で有限値かつ正の幅があることを確認する。
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (!std::isfinite(i_bounds.minimum[axis]) || !std::isfinite(i_bounds.maximum[axis]) ||
            i_bounds.minimum[axis] >= i_bounds.maximum[axis])
        {
            throw std::invalid_argument("障害物の範囲は有限で各軸に正の幅が必要です。");
        }
    }
}

/**
 * @brief タイルの管理セルと障害物が XZ 上で接触するか判定する。
 * @param i_field 共通格子を表す地形の高さ場。
 * @param i_managed 管理セルの範囲。
 * @param i_bounds 検索対象の箱。
 * @return 接触または重複する場合は true。
 * @throws std::invalid_argument 高さ場、セル範囲、または箱が不正な場合。
 * @details 床より上の障害物も頭上空間を塞ぐため Y 範囲では除外しない。
 */
bool ObstacleTouches(const Heightfield& i_field, const CellRect& i_managed, const Bounds& i_bounds)
{
    // 範囲判定に使う格子、セル矩形、障害物を個別に検証してから接触を調べる。
    ValidateGrid(i_field);
    ValidateRect(i_field, i_managed);
    ValidateBoxObstacle(i_bounds);
    return Touches(i_field, i_managed, i_bounds);
}

/**
 * @brief 作業範囲に入る障害物を作業範囲の高さ場に重ねる。
 * @param i_base 元の共通高さ場。
 * @param i_region 対象タイルのセル範囲。
 * @param i_obstacles 検証済み障害物の一覧。
 * @return 作業範囲の座標を原点とする高さ場。高さの原点は元の地形と同じ。
 * @throws std::invalid_argument 高さ場、セル範囲、または障害物が不正な場合。
 * @throws std::length_error 障害物の高さが表現できる範囲を超えた場合。
 */
Heightfield ApplyBoxObstacles(const Heightfield& i_base, const TileRegion& i_region,
                              const std::vector<BoxObstacle>& i_obstacles)
{
    const auto& work = i_region.work;
    // 入力地形とタイル内の各範囲を検査し、core ⊆ managed ⊆ work の関係を保証する。
    ValidateGrid(i_base);
    ValidateRect(i_base, i_region.core);
    ValidateRect(i_base, i_region.managed);
    ValidateRect(i_base, work);
    if (!ContainsRect(i_region.managed, i_region.core) || !ContainsRect(work, i_region.managed))
    {
        throw std::invalid_argument("タイルの所有範囲、管理範囲、作業範囲の関係が不正です。");
    }
    // 障害物全体を先に検証してから、作業範囲に対応する元のセル列を複製する。
    for (const auto& obstacle : i_obstacles) { ValidateBoxObstacle(obstacle.bounds); }
    auto result = Heightfield{i_base.bounds, i_base.settings, work.endX - work.beginX,
                              work.endZ - work.beginZ, {}};
    result.columns.reserve(result.width * result.depth);
    for (auto z = work.beginZ; z < work.endZ; ++z)
    {
        for (auto x = work.beginX; x < work.endX; ++x)
        {
            result.columns.push_back(i_base.columns[z * i_base.width + x]);
        }
    }

    // 作業範囲に触れ、地形の下端より上にある障害物をボクセル高さ区間へ変換する。
    // 管理範囲の外側も含めて重ねることで、侵食・接続処理に必要な余白を正しく保つ。
    for (const auto& obstacle : i_obstacles)
    {
        if (!Touches(i_base, work, obstacle.bounds) ||
            obstacle.bounds.maximum.y < i_base.bounds.minimum.y) { continue; }
        const auto origin = static_cast<double>(i_base.bounds.minimum.y);
        const auto cellHeight = static_cast<double>(i_base.settings.cellHeight);
        const auto low = std::floor(std::max(0.0, (obstacle.bounds.minimum.y - origin) / cellHeight));
        const auto high = std::ceil((obstacle.bounds.maximum.y - origin) / cellHeight);
        if (!std::isfinite(low) || !std::isfinite(high) || low > kMaxHeight || high > kMaxHeight)
        {
            throw std::length_error("障害物の高さがボクセルの上限を超えました。");
        }
        const auto minimum = static_cast<std::uint32_t>(low);
        const auto maximum = std::max(minimum + 1U, static_cast<std::uint32_t>(high));
        const auto solid = SolidSpan{minimum, maximum, false};
        // 箱が触れる各セルの高さ列へ区間を追加し、既存区間との重複を統合する。
        for (auto z = work.beginZ; z < work.endZ; ++z)
        {
            for (auto x = work.beginX; x < work.endX; ++x)
            {
                if (Touches(i_base, {x, z, x + 1, z + 1}, obstacle.bounds))
                {
                    MergeObstacle(result.columns[(z - work.beginZ) * result.width + x - work.beginX], solid);
                }
            }
        }
    }
    return result;
}
}
