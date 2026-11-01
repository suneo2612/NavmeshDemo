/**
 * @file tile_field.cpp
 * @brief 共通格子をタイルごとに振り分ける。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "tile_field.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace demo::navmesh::voxelization
{
namespace
{
/**
 * @brief 地形範囲内にあるセル部分の中心を含むタイルを求める。
 * @param i_cell 軸方向のセル添字。
 * @param i_minimum 地形範囲の軸方向の最小座標。
 * @param i_maximum 地形範囲の軸方向の最大座標。
 * @param i_size 軸方向のセル幅。
 * @param i_boundaries 実際のタイル境界。地形範囲の両端は含まない。
 * @return タイル添字。
 * @details 地形範囲からはみ出すセル部分を除いて中心を求め、その中心がいくつの
 * タイル境界を越えているかで所属先を決める。中心が境界上にある場合は、その境界の先のタイルに属する。
 */
std::size_t Owner(const std::size_t i_cell,
                  const double i_minimum,
                  const double i_maximum,
                  const double i_size,
                  const std::vector<double>& i_boundaries)
{
    // その軸のセルの始点を求める。
    const auto start    =   i_minimum + static_cast<double>(i_cell) * i_size;
    // 地形の端を越えるセルでは、終点を地形の最大位置までに制限する。
    const auto end      =   std::min(start + i_size, i_maximum);
    // 地形内に含まれるセル部分の中心を、所属タイルの判定位置とする。
    const auto center   =   start + (end - start) * 0.5;

    // 中心より手前（中心と同じ位置を含む）にある境界の数が、タイルの添字になる。
    // 境界と中心が一致した場合は、その境界の先にあるタイルを選ぶ。
    return static_cast<std::size_t>(
        std::upper_bound(i_boundaries.begin(), i_boundaries.end(), center) - i_boundaries.begin());
}

/**
 * @brief タイル数が正で、総数を安全に計算できることを確認する。
 * @param i_grid 確認対象のタイル格子。
 * @return タイル数が有効な場合は true。
 */
bool HasValidTileLayout(const TileGrid& i_grid)
{
    const auto columns  =   i_grid.Columns();
    const auto rows     =   i_grid.Rows();
    return  columns != 0 &&                                             // X 方向のタイル数が 0 でないことを確認する。
            rows != 0 &&                                                // Z 方向のタイル数が 0 でないことを確認する。
            columns <= std::numeric_limits<std::size_t>::max() / rows;  // タイル総数の乗算がオーバーフローしないことを確認する。
}

/**
 * @brief 高さ場のセル寸法と列配列の大きさが一致することを確認する。
 * @param i_heightfield 確認対象の高さ場。
 * @return セル数を安全に計算でき、全セル分の列がある場合は true。
 */
bool HasValidHeightfieldLayout(const Heightfield& i_heightfield)
{
    if (i_heightfield.width == 0 ||     // 幅が 0 でないことを確認する。
        i_heightfield.depth == 0 ||     // 奥行きが 0 でないことを確認する。
        i_heightfield.width > std::numeric_limits<std::size_t>::max() / i_heightfield.depth)    // セル総数の乗算がオーバーフローしないことを確認する。
    {
        return false;
    }

    // 列配列のサイズがセル総数と一致することを確認する。
    return i_heightfield.columns.size() == i_heightfield.width * i_heightfield.depth;
}

/**
 * @brief 高さ場と歩行面が同じセル寸法で作られていることを確認する。
 * @param i_heightfield 元の高さ場。
 * @param i_field 対応する歩行面。
 * @return 幅と奥行きが一致する場合は true。
 */
bool HasMatchingWalkableDimensions(const Heightfield& i_heightfield, const WalkableField& i_field)
{
    // 幅と奥行きが一致することを確認する。
    return i_heightfield.width == i_field.width && i_heightfield.depth == i_field.depth;
}

/**
 * @brief 地形範囲とセル幅から求めたセル数が高さ場の寸法と一致することを確認する。
 * @param i_heightfield 確認対象の高さ場。
 * @return 範囲と設定が有効で、X/Z のセル数が一致する場合は true。
 */
bool HasMatchingHeightfieldDimensions(const Heightfield& i_heightfield)
{
    const auto& bounds = i_heightfield.bounds;
    const auto cellSize = static_cast<double>(i_heightfield.settings.cellSize);

    // セル幅が有限で正の値であることを確認する。
    if (!std::isfinite(cellSize) || cellSize <= 0 ||
        !std::isfinite(bounds.minimum.x) || !std::isfinite(bounds.maximum.x) ||
        !std::isfinite(bounds.minimum.z) || !std::isfinite(bounds.maximum.z) ||
        bounds.minimum.x >= bounds.maximum.x || bounds.minimum.z >= bounds.maximum.z)
    {
        return false;
    }

    // 期待される幅と奥行きを計算する。
    const auto expectedWidth = std::ceil((static_cast<double>(bounds.maximum.x) - bounds.minimum.x) / cellSize);
    const auto expectedDepth = std::ceil((static_cast<double>(bounds.maximum.z) - bounds.minimum.z) / cellSize);
    return expectedWidth == i_heightfield.width && expectedDepth == i_heightfield.depth;
}

/**
 * @brief 高さ場の外周とタイル格子の外周が一致することを確認する。
 * @param i_bounds 高さ場が表す地形範囲。
 * @param i_first タイル格子の左上端にあるタイル範囲。
 * @param i_last タイル格子の右下端にあるタイル範囲。
 * @return 地形範囲とタイル格子の外周が一致する場合は true。
 */
bool HasMatchingTileBounds(const Bounds& i_bounds, const Bounds& i_first, const Bounds& i_last)
{
    // 高さ場の外周とタイル格子の外周が一致することを確認する。
    return  i_bounds.minimum.x == i_first.minimum.x &&
            i_bounds.minimum.y == i_first.minimum.y &&
            i_bounds.minimum.z == i_first.minimum.z &&
            i_bounds.maximum.x == i_last.maximum.x &&
            i_bounds.maximum.y == i_last.maximum.y &&
            i_bounds.maximum.z == i_last.maximum.z;
}
}


/**
 * @brief 共通格子の接続を維持したまま歩行面をタイルに振り分ける。
 * @param i_heightfield 歩行面を作成した高さ場。
 * @param i_field 高さ場から作成した歩行面。結果にコピーされる。
 * @param i_grid 高さ場と同じ地形範囲のタイル格子。
 * @return 距離場とタイル別の参照を持つ共通格子。
 * @throws std::invalid_argument 範囲、格子、歩行面の構成が不正な場合。
 * @throws std::length_error 距離が表現できる範囲を超えた場合。
 * @details ボクセルとタイルの境界は一致しなくてもよい。セルの地形範囲内の中心が
 * 属するタイルを所有者とするため、セルがタイルの範囲をはみ出すことがある。
 * タイル間接続と距離は共通格子全体で計算される。タイル単独の再構築ではない。
 */
TiledWalkableField PartitionWalkableField(const Heightfield& i_heightfield,
                                          const WalkableField& i_field,
                                          const TileGrid& i_grid)
{
    // タイル数が不正な場合は、外周タイルを参照する前に拒否する。
    if (!HasValidTileLayout(i_grid))
    {
        throw std::invalid_argument("タイルと高さ場の格子または地形範囲が一致しません。");
    }

    // 高さ場の境界とタイル格子の外周タイルを取得する。
    const auto& bounds      =   i_heightfield.bounds;
    const auto& settings    =   i_heightfield.settings;
    const auto first        =   i_grid.TileBounds(0, 0);
    const auto last         =   i_grid.TileBounds(i_grid.Columns() - 1, i_grid.Rows() - 1);

    // 高さ場・歩行面・タイル格子が同じ地形とセル配置を表していることを確認する。
    if (!HasValidHeightfieldLayout(i_heightfield) ||
        !HasMatchingWalkableDimensions(i_heightfield, i_field) ||
        !HasMatchingHeightfieldDimensions(i_heightfield) ||
        !HasMatchingTileBounds(bounds, first, last))
    {
        throw std::invalid_argument("タイルと高さ場の格子または地形範囲が一致しません。");
    }

    // タイルの境界で距離が途切れないよう、歩行面を分割する前に共通格子全体の距離場を作る。
    auto distance = BuildDistanceField(i_field);

    // 各歩行面が、入力された高さ場の同じセル・同じ高さの占有区間から作られているか確認する。
    for (const auto& span : i_field.spans)
    {
        const auto& column = i_heightfield.columns[span.z * i_field.width + span.x];
        // 元区間が存在し、歩行可能な上面・床の高さ・天井の位置が一致しなければ入力の組み合わせが不正。
        if (span.sourceIndex >= column.size() || !column[span.sourceIndex].walkableTop ||
            column[span.sourceIndex].maximum != span.floor ||
            (span.sourceIndex + 1 < column.size()
                ? span.ceiling != std::optional<std::uint32_t>{column[span.sourceIndex + 1].minimum}
                : span.ceiling.has_value()))
        {
            throw std::invalid_argument("歩行面が指定された高さ場と一致しません。");
        }
    }

    // 共通格子の歩行面と距離場を結果に保持する。タイルごとの情報はこの後で追加する。
    auto result = TiledWalkableField{i_field, std::move(distance), {}};

    // タイル格子の各区画に、実際のワールド空間の範囲を持つ入れ物を用意する。
    result.tiles.reserve(i_grid.Columns() * i_grid.Rows());
    for (auto row = std::size_t{}; row < i_grid.Rows(); ++row)
    {
        for (auto column = std::size_t{}; column < i_grid.Columns(); ++column)
        {
            result.tiles.push_back({i_grid.TileBounds(column, row), {}, {}});
        }
    }

    // タイル間の境界を格子と同じ計算結果から取得する。
    // 境界を再計算せず、描画時の位置とセルの振り分け位置がずれないようにする。
    auto xBoundaries = std::vector<double>{};
    auto zBoundaries = std::vector<double>{};
    // 地形の外周ではなく、隣り合うタイルを分ける内部境界だけを保存する。
    for (auto column = std::size_t{1}; column < i_grid.Columns(); ++column)
    {
        xBoundaries.push_back(i_grid.TileBounds(column, 0).minimum.x);
    }
    for (auto row = std::size_t{1}; row < i_grid.Rows(); ++row)
    {
        zBoundaries.push_back(i_grid.TileBounds(0, row).minimum.z);
    }

    // X方向はセルごとの所属先を一度だけ求め、同じ列にあるセル間で再利用する。
    auto xOwners = std::vector<std::size_t>{};
    xOwners.reserve(i_field.width);
    for (auto x = std::size_t{}; x < i_field.width; ++x)
    {
        xOwners.push_back(Owner(x, bounds.minimum.x, bounds.maximum.x, settings.cellSize, xBoundaries));
    }

    // 地面を上から見たセル領域ごとに所属タイルを決め、そのセルにある全ての高さの歩行面を同じタイルへ割り当てる。
    for (auto z = std::size_t{}; z < i_field.depth; ++z)
    {
        // 同じZ位置に並ぶセルは同じタイル行に属するため、行の所属先を一度求める。
        const auto row = Owner(z, bounds.minimum.z, bounds.maximum.z, settings.cellSize, zBoundaries);
        for (auto x = std::size_t{}; x < i_field.width; ++x)
        {
            // セル座標に対応するタイルを選び、共通格子上のセル添字を記録する。
            const auto cell = z * i_field.width + x;
            auto& tile = result.tiles[row * i_grid.Columns() + xOwners[x]];
            tile.cells.push_back(cell);

            // このセルの全階層の歩行面を登録する。歩行面自体は共通格子に残し、添字で参照する。
            for (auto index = i_field.offsets[cell]; index < i_field.offsets[cell + 1]; ++index)
            {
                tile.spans.push_back(index);
            }
        }
    }
    return result;
}
}
