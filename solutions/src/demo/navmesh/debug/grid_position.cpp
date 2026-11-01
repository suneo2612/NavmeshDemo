/**
 * @file grid_position.cpp
 * @brief ボクセル格子の座標を描画用ワールド座標に変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "grid_position.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
/**
 * @brief 描画可能な浮動小数点座標に変換する。
 * @param i_value ワールド座標。
 * @return 描画用の座標。
 */
float Coordinate(const double i_value)
{
    // GPU 頂点へ格納する前に float の有限範囲を確認し、無効座標を描画へ流さない。
    const auto limit = static_cast<double>(std::numeric_limits<float>::max());
    if (!std::isfinite(i_value) || i_value < -limit || i_value > limit)
    {
        throw std::overflow_error("格子の描画座標を表現できません。");
    }
    return static_cast<float>(i_value);
}
}

/**
 * @brief 共通格子の頂点を端の短いセルも考慮してワールド座標に変換する。
 * @param i_navmesh 構築済みの地形とボクセル設定。
 * @param i_x 共通格子上の X 座標。
 * @param i_z 共通格子上の Z 座標。
 * @param i_floor 地形最低 Y からの高さセル数。
 * @param i_offset 描画時に高さへ加える値。
 * @return 描画用のワールド座標。
 * @throws std::overflow_error 座標が表現できない場合。
 */
std::array<float, 3> GridPosition(const Navmesh& i_navmesh, const double i_x, const double i_z,
                                  const std::uint32_t i_floor, const double i_offset)
{
    const auto& bounds = i_navmesh.TerrainBounds();
    const auto& raster = i_navmesh.Settings().raster;
    // XZ はセル位置を実寸に補間して地形端で切り詰め、Y は高さセルと表示オフセットで求める。
    const auto x = static_cast<double>(bounds.minimum.x) + polygon::GridCoordinate(
        i_x, raster.cellSize, static_cast<double>(bounds.maximum.x) - bounds.minimum.x);
    const auto z = static_cast<double>(bounds.minimum.z) + polygon::GridCoordinate(
        i_z, raster.cellSize, static_cast<double>(bounds.maximum.z) - bounds.minimum.z);
    const auto y = static_cast<double>(bounds.minimum.y) + i_floor * static_cast<double>(raster.cellHeight) + i_offset;
    return {Coordinate(x), Coordinate(y), Coordinate(z)};
}

/**
 * @brief 整数格子の頂点をワールド座標に変換する。
 * @param i_navmesh 構築済みの地形とボクセル設定。
 * @param i_x 共通格子上の X 座標。
 * @param i_z 共通格子上の Z 座標。
 * @param i_floor 地形最低 Y からの高さセル数。
 * @param i_offset 描画時に高さへ加える値。
 * @return 描画用のワールド座標。
 * @throws std::overflow_error 座標が表現できない場合。
 */
std::array<float, 3> GridPosition(const Navmesh& i_navmesh, const std::size_t i_x, const std::size_t i_z,
                                  const std::uint32_t i_floor, const double i_offset)
{
    // double の仮数で整数を正確に保持できる範囲を確認してから小数版へ委譲する。
    constexpr auto maxExact = std::uint64_t{1} << std::numeric_limits<double>::digits;
    if (i_x > maxExact || i_z > maxExact) { throw std::overflow_error("格子座標を正確に表現できません。"); }
    return GridPosition(i_navmesh, static_cast<double>(i_x), static_cast<double>(i_z), i_floor, i_offset);
}
}
