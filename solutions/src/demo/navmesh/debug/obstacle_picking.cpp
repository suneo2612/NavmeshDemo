/**
 * @file obstacle_picking.cpp
 * @brief 画面からの光線と地形・箱の交差を求める。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "obstacle_picking.h"
#include "../../../render/defs/camera_constants.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
using Vector = general::vec::Vector<double, 3>;
constexpr auto kTriangleTolerance = 1e-12;

/**
 * @brief float の座標を計算用の倍精度に変換する。
 * @param i_value 元の座標。
 * @return 倍精度の座標。
 */
Vector ToDouble(const general::vec::Vector3& i_value)
{
    // 幾何演算で float の精度不足を避けるため、入力位置を倍精度ベクトルへ移す。
    return {i_value.x, i_value.y, i_value.z};
}

/**
 * @brief 光線の入力を検証し、単位方向を求める。
 * @param i_ray 対象の光線。
 * @return 単位方向。
 */
Vector UnitDirection(const PickRay& i_ray)
{
    // 始点と方向の各成分を検査し、ゼロ長でない方向を単位長へ正規化する。
    auto direction = ToDouble(i_ray.direction);
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (!std::isfinite(i_ray.origin[axis]) || !std::isfinite(direction[axis]))
        {
            throw std::invalid_argument("選択用光線の座標が不正です。");
        }
    }
    const auto length = direction.Length();
    if (!std::isfinite(length) || length <= 0) { throw std::invalid_argument("選択用光線の方向が不正です。"); }
    direction = direction * (1.0 / length);
    return direction;
}

/**
 * @brief 光線と三角形の交差距離を求める。
 * @param i_origin 光線の始点。
 * @param i_direction 光線の単位方向。
 * @param i_triangle 地形の三角形。
 * @return 前方に交点がある場合の距離。
 */
std::optional<double> TriangleDistance(const Vector& i_origin, const Vector& i_direction,
                                       const std::array<general::vec::Vector3, 3>& i_triangle)
{
    // 光線と三角形の辺ベクトルを使い、平行・退化した面を除いて交点の重心座標を計算する。
    const auto a = ToDouble(i_triangle[0]);
    const auto edge1 = ToDouble(i_triangle[1]) - a;
    const auto edge2 = ToDouble(i_triangle[2]) - a;
    const auto cross = i_direction.Cross(edge2);
    const auto determinant = edge1.Dot(cross);
    const auto scale = edge1.Length() * edge2.Length();
    if (scale == 0 || std::abs(determinant) <= kTriangleTolerance * scale) { return std::nullopt; }
    const auto inverse = 1.0 / determinant;
    const auto offset = i_origin - a;
    const auto u = offset.Dot(cross) * inverse;
    if (u < 0 || u > 1) { return std::nullopt; }
    const auto q = offset.Cross(edge1);
    const auto v = i_direction.Dot(q) * inverse;
    if (v < 0 || u + v > 1) { return std::nullopt; }
    const auto distance = edge2.Dot(q) * inverse;
    // 三角形の外側または光線の後方にある交点は選択候補にしない。
    if (distance < 0) { return std::nullopt; }
    return distance;
}

/**
 * @brief 光線と箱の最初の交差距離を求める。
 * @param i_origin 光線の始点。
 * @param i_direction 光線の単位方向。
 * @param i_bounds 障害物の箱。
 * @return 前方に交点がある場合の距離。
 */
std::optional<double> BoxDistance(const Vector& i_origin, const Vector& i_direction, const Bounds& i_bounds)
{
    // X/Y/Z 各軸の交差区間を重ね、光線が箱を通過する共通区間を絞り込む。
    auto nearDistance = 0.0;
    auto farDistance = std::numeric_limits<double>::infinity();
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (i_direction[axis] == 0)
        {
            if (i_origin[axis] < i_bounds.minimum[axis] || i_origin[axis] > i_bounds.maximum[axis])
            {
                return std::nullopt;
            }
            continue;
        }
        auto first = (i_bounds.minimum[axis] - i_origin[axis]) / i_direction[axis];
        auto last = (i_bounds.maximum[axis] - i_origin[axis]) / i_direction[axis];
        if (first > last) { std::swap(first, last); }
        nearDistance = std::max(nearDistance, first);
        farDistance = std::min(farDistance, last);
        if (nearDistance > farDistance) { return std::nullopt; }
    }
    return nearDistance;
}
}

/**
 * @brief 画面のマウス位置から投影に対応した光線を作る。
 * @param i_camera カメラ。
 * @param i_x ウィンドウ内の横座標。
 * @param i_y ウィンドウ内の縦座標。
 * @param i_width ウィンドウの論理幅。
 * @param i_height ウィンドウの論理高さ。
 * @param i_aspect フレームバッファーの横縦比。
 * @return 画面外の場合は空、それ以外はワールド空間の光線。
 * @throws std::invalid_argument 画面寸法または座標が不正な場合。
 */
std::optional<PickRay> ScreenPickRay(const render::Camera& i_camera, const float i_x, const float i_y,
                                     const float i_width, const float i_height, const float i_aspect)
{
    // 画面寸法とマウス座標を検証し、画面外の位置は交差判定を行わず空で返す。
    if (!std::isfinite(i_x) || !std::isfinite(i_y) ||
        !std::isfinite(i_width) || !std::isfinite(i_height) || !std::isfinite(i_aspect) ||
        i_width <= 0 || i_height <= 0 || i_aspect <= 0)
    {
        throw std::invalid_argument("選択用の画面座標または画面サイズが不正です。");
    }
    if (i_x < 0 || i_x >= i_width || i_y < 0 || i_y >= i_height) { return std::nullopt; }

    // 垂直視野角と画面比からカメラ基底上の方向を作り、ワールド空間の単位光線にする。
    const auto forward = ToDouble(i_camera.Forward());
    auto right = forward.Cross({0, 1, 0});
    right.Normalize();
    const auto up = right.Cross(forward);
    const auto tangent = std::tan(static_cast<double>(render::defs::kCameraVerticalFovRadians) * 0.5);
    const auto x = (2 * static_cast<double>(i_x) / i_width - 1) * tangent * i_aspect;
    const auto y = (1 - 2 * static_cast<double>(i_y) / i_height) * tangent;
    auto direction = forward + right * x + up * y;
    direction.Normalize();
    return PickRay{i_camera.Position(), {static_cast<float>(direction.x), static_cast<float>(direction.y),
                                         static_cast<float>(direction.z)}};
}

/**
 * @brief 地形の三角形と光線の最も近い交点を求める。
 * @param i_terrain ワールド空間の地形。
 * @param i_ray 検索する光線。
 * @return 交差しない場合は空。
 * @throws std::invalid_argument 光線が不正な場合。
 */
std::optional<TerrainHit> PickTerrain(const Terrain& i_terrain, const PickRay& i_ray)
{
    // 全三角形との前方交差を比較し、描画距離範囲にある最も近い交点だけを返す。
    const auto direction = UnitDirection(i_ray);
    const auto origin = ToDouble(i_ray.origin);
    auto nearest = std::numeric_limits<double>::infinity();
    for (const auto& triangle : i_terrain.triangles)
    {
        const auto distance = TriangleDistance(origin, direction, triangle);
        if (distance && *distance < nearest) { nearest = *distance; }
    }
    if (!std::isfinite(nearest) || nearest > render::defs::kCameraFarPlane) { return std::nullopt; }
    const auto point = origin + direction * nearest;
    return TerrainHit{{static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)}, nearest};
}

/**
 * @brief 光線上の最も手前の障害物を探す。
 * @param i_obstacles ワールド空間の障害物。
 * @param i_ray 検索する光線。
 * @param i_maxDistance この距離より手前だけを対象とする。
 * @return 対象がない場合は空、それ以外は障害物の識別子。
 * @throws std::invalid_argument 光線または距離が不正な場合。
 */
std::optional<std::uint64_t> PickObstacle(const std::vector<voxelization::BoxObstacle>& i_obstacles,
                                          const PickRay& i_ray, const double i_maxDistance)
{
    // 距離制限を検証し、地形交点と描画の遠平面のうち近い方を検索上限にする。
    if (std::isnan(i_maxDistance) || i_maxDistance < 0)
    {
        throw std::invalid_argument("障害物の選択距離が不正です。");
    }
    const auto direction = UnitDirection(i_ray);
    const auto origin = ToDouble(i_ray.origin);
    auto nearest = std::min(i_maxDistance, static_cast<double>(render::defs::kCameraFarPlane));
    auto selected = std::optional<std::uint64_t>{};
    // 上限内で最も近い箱の識別子を保持し、後方に隠れた箱を選ばない。
    for (const auto& obstacle : i_obstacles)
    {
        const auto distance = BoxDistance(origin, direction, obstacle.bounds);
        if (distance && *distance <= nearest && (!selected || *distance < nearest))
        {
            nearest = *distance;
            selected = obstacle.id;
        }
    }
    return selected;
}
}
