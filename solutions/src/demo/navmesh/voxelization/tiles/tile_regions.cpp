/**
 * @file tile_regions.cpp
 * @brief 地形セルの担当タイルと、境界確認に使う周辺範囲を求める。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#include "tile_regions.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace demo::navmesh::voxelization
{
namespace
{
constexpr auto kConnectionCells = std::size_t{1};     // 隣のセルとの接続を確認するために加える余白。
constexpr auto kSmoothingCells = std::size_t{1};      // 距離情報を平滑化する計算に必要な余白。
constexpr auto kMaxBorderCells = std::size_t{1000000}; // 受け入れる余白セル数の上限。

/**
 * @brief セル範囲を地形全体の端で止めながら広げる。
 * @param i_rect 広げる範囲。
 * @param i_cells 外側に追加するセル数。
 * @param i_width 格子の幅。
 * @param i_depth 格子の奥行き。
 * @return 地形全体のセル範囲内に収まる結果。
 */
CellRect Expand(const CellRect& i_rect, const std::size_t i_cells,
                const std::size_t i_width, const std::size_t i_depth)
{
    // X と Z を別々に広げる。開始位置は 0 より小さくせず、終端位置は全体のセル数を越えない。
    // 各範囲の終端は含まないため、追加数を終端に足してもセル数としてそのまま扱える。
    return {i_rect.beginX > i_cells ? i_rect.beginX - i_cells : 0,
            i_rect.beginZ > i_cells ? i_rect.beginZ - i_cells : 0,
            i_rect.endX + std::min(i_cells, i_width - i_rect.endX),
            i_rect.endZ + std::min(i_cells, i_depth - i_rect.endZ)};
}

/**
 * @brief 座標範囲に触れるセルをすべて含む区間を求める。
 * @param i_min 範囲の最小座標。
 * @param i_max 範囲の最大座標。
 * @param i_origin 地形セル範囲の開始座標。
 * @param i_size セル一つの幅。
 * @param i_count 対象方向のセル数。
 * @return 範囲に触れるセルを含む区間。終端の添字は含まない。
 */
std::pair<std::size_t, std::size_t> Cover(const double i_min, const double i_max,
                                          const double i_origin, const double i_size,
                                          const std::size_t i_count)
{
    // 最小座標がセル境界上にあっても、その境界に触れる手前側のセルを含める。
    // nextafter で値をわずかに小さくしてから切り下げ、境界上の値を一つ前のセルに対応させる。
    const auto first = std::floor(std::nextafter((i_min - i_origin) / i_size,
                                                  -std::numeric_limits<double>::infinity()));
    // 最大座標に触れるセルも含めるため、位置を切り下げた後で終端を一つ先へ進める。
    const auto last = std::floor((i_max - i_origin) / i_size);

    // 座標が地形の外にあっても不正な添字にならないよう、開始位置と終端位置をセル数で制限する。
    const auto begin = first <= 0 ? std::size_t{} :
        first >= static_cast<double>(i_count) ? i_count : static_cast<std::size_t>(first);
    const auto end = last < 0 ? std::size_t{} :
        last >= static_cast<double>(i_count - 1) ? i_count : static_cast<std::size_t>(last) + 1;
    // [begin, end) の形で返し、隣り合う範囲を重複なく連結できるようにする。
    return {begin, end};
}

/**
 * @brief 各セルの中心位置を基準に、タイルごとの担当セル範囲を求める。
 * @param i_count 対象方向のセル数。
 * @param i_min 地形の開始座標。
 * @param i_max 地形の終端座標。
 * @param i_size セル一つの幅。
 * @param i_boundaries タイル境界の座標。地形の両端も含む。
 * @return タイルごとの担当セル区間。終端の添字は含まない。
 * @throws std::invalid_argument 担当するセルが一つもないタイルがある場合。
 */
std::vector<std::pair<std::size_t, std::size_t>> Owners(
    const std::size_t i_count, const double i_min, const double i_max, const double i_size,
    const std::vector<double>& i_boundaries)
{
    const auto tiles = i_boundaries.size() - 1;
    // 未割り当てをセル数で表し、各タイルが受け持つ範囲の開始・終端を記録する。
    auto ranges = std::vector<std::pair<std::size_t, std::size_t>>(tiles, {i_count, i_count});

    // 地形の端からはみ出す最後のセルは地形内にある部分だけを使い、その中心で担当タイルを決める。
    // 境界と中心が一致した場合は upper_bound により境界の右側へ割り当て、担当を一意にする。
    for (auto index = std::size_t{}; index < i_count; ++index)
    {
        const auto start = i_min + static_cast<double>(index) * i_size;
        const auto center = start + (std::min(start + i_size, i_max) - start) * 0.5;
        const auto owner = static_cast<std::size_t>(
            std::upper_bound(i_boundaries.begin() + 1, i_boundaries.end() - 1, center) -
            (i_boundaries.begin() + 1));

        // セルは X または Z の順に調べるため、最初に見つけた位置が開始、最後に見つけた位置の次が終端になる。
        if (ranges[owner].first == i_count) { ranges[owner].first = index; }
        ranges[owner].second = index + 1;
    }

    // 担当セルのないタイルは後続の範囲計算ができないため、空の範囲を結果として返さず入力誤りにする。
    for (const auto& range : ranges)
    {
        if (range.first == i_count)
        {
            throw std::invalid_argument("ボクセルを所有しないタイルがあります。セル幅またはタイル数を調整してください。");
        }
    }
    return ranges;
}

/**
 * @brief 高さ場のセル寸法、格納数および地形範囲が互いに一致するか確認する。
 * @param i_field 確認する高さ場。
 * @param i_cell セル一つの幅。
 * @return 寸法と地形範囲の対応が正しい場合は true。
 */
bool HasValidHeightfieldCellLayout(const Heightfield& i_field, const double i_cell)
{
    // 先に寸法を確認し、後続のセル数の乗算がゼロ割りや整数オーバーフローにならないようにする。
    if (!std::isfinite(i_cell) || i_cell <= 0 ||
        i_field.width == 0 || i_field.depth == 0 ||
        i_field.width > std::numeric_limits<std::size_t>::max() / i_field.depth)
    {
        return false;
    }
    // 格子の寸法だけでなく、実際に保持しているセル配列の要素数も一致する必要がある。
    if (i_field.columns.size() != i_field.width * i_field.depth)
    {
        return false;
    }

    // 地形の端で最後のセルが短くなる場合を考慮し、幅を切り上げたセル数と配列寸法を照合する。
    const auto& bounds = i_field.bounds;
    const auto expectedWidth = std::ceil((static_cast<double>(bounds.maximum.x) - bounds.minimum.x) / i_cell);
    const auto expectedDepth = std::ceil((static_cast<double>(bounds.maximum.z) - bounds.minimum.z) / i_cell);
    return expectedWidth == i_field.width && expectedDepth == i_field.depth;
}

/**
 * @brief 高さ場の最小端と最大端がタイル格子の外周に一致するか確認する。
 * @param i_bounds 高さ場の範囲。
 * @param i_first 格子の最初のタイルの範囲。
 * @param i_last 格子の最後のタイルの範囲。
 * @return X、Y、Z の両端がすべて一致する場合は true。
 */
bool HasMatchingTileBounds(const Bounds& i_bounds, const Bounds& i_first, const Bounds& i_last)
{
    // 最初のタイルの最小端と高さ場の最小端を比べ、格子の開始位置が一致するか確認する。
    const auto matchesMinimum = i_bounds.minimum.x == i_first.minimum.x &&
                                i_bounds.minimum.y == i_first.minimum.y &&
                                i_bounds.minimum.z == i_first.minimum.z;
    // 最後のタイルの最大端も照合し、格子の終端まで同じ範囲を覆っているか確認する。
    const auto matchesMaximum = i_bounds.maximum.x == i_last.maximum.x &&
                                i_bounds.maximum.y == i_last.maximum.y &&
                                i_bounds.maximum.z == i_last.maximum.z;
    // 開始位置か終端位置のどちらかが異なれば、同じ地形を分割していない。
    return matchesMinimum && matchesMaximum;
}
}

/**
 * @brief セル範囲を地形全体の端を越えないように広げる。
 * @param i_rect 広げるセル範囲。開始位置は終端より小さく、地形の範囲内であること。
 * @param i_cells 各辺に追加するセル数。
 * @param i_width 地形全体の X 方向のセル数。
 * @param i_depth 地形全体の Z 方向のセル数。
 * @return 地形全体の範囲内に収まる拡張後のセル範囲。
 * @throws std::invalid_argument セル範囲が空、または地形の範囲を越えている場合。
 */
CellRect ExpandCellRect(const CellRect& i_rect, const std::size_t i_cells,
                        const std::size_t i_width, const std::size_t i_depth)
{
    // Expand は既存範囲から残りセル数を引くため、空の範囲や地形外の終端を先に拒否する。
    if (i_rect.beginX >= i_rect.endX || i_rect.beginZ >= i_rect.endZ ||
        i_rect.endX > i_width || i_rect.endZ > i_depth)
    {
        throw std::invalid_argument("拡張するセル範囲が不正です。");
    }
    // 入力が地形内にあることを確認した後で、指定数だけ四辺を広げる。
    return Expand(i_rect, i_cells, i_width, i_depth);
}

/**
 * @brief キャラクターがタイル境界で不自然に切れないための最小余白を求める。
 * @param i_cellSize 地面を区切るセル一つの幅。
 * @param i_agentRadius キャラクターの半径。
 * @return キャラクター半径に加え、接続確認と距離計算に必要なセル数。
 * @throws std::invalid_argument セル幅またはキャラクター半径が有限の正数でない場合。
 * @throws std::length_error 必要な余白が上限を超える場合。
 */
std::size_t MinimumTileBorder(const float i_cellSize, const float i_agentRadius)
{
    // 半径をセル数へ換算する前に、除算に使う値が有限かつ正であることを確認する。
    if (!std::isfinite(i_cellSize) || i_cellSize <= 0 ||
        !std::isfinite(i_agentRadius) || i_agentRadius <= 0)
    {
        throw std::invalid_argument("セル幅とキャラクター半径は有限の正数で指定してください。");
    }

    // 半径を覆うには端数を切り上げる。さらに接続確認と距離の平滑化に各一セルを加える。
    const auto cells = std::ceil(static_cast<double>(i_agentRadius) / i_cellSize);

    // 上限を超える値を size_t に変換する前に拒否し、加算後の余白も設定上限内に保つ。
    if (cells > static_cast<double>(kMaxBorderCells - kConnectionCells - kSmoothingCells))
    {
        throw std::length_error("タイルの必要余白が大きすぎます。");
    }
    return static_cast<std::size_t>(cells) + kConnectionCells + kSmoothingCells;
}

/**
 * @brief 各タイルの出力範囲と、計算に使う周辺のセル範囲を求める。
 * @param i_field 地形のセル数、セル幅、ワールド座標での範囲。
 * @param i_grid 地形全体を分割するタイル格子。
 * @param i_borderCells タイル範囲の外側に加える余白のセル数。
 * @param i_workCells 余白のさらに外側に加える接続確認用のセル数。
 * @return 行ごとに並べたタイル範囲の一覧。各セル範囲は終端を含まない。
 * @throws std::invalid_argument 地形とタイル格子が一致しない場合、またはセルを担当しないタイルがある場合。
 * @details セルの中心位置で出力を担当するタイルを決める。
 * タイル範囲に触れるセルも計算範囲に含め、境界での計算漏れを防ぐ。
 */
std::vector<TileRegion> BuildTileRegions(const Heightfield& i_field, const TileGrid& i_grid,
                                         const std::size_t i_borderCells, const std::size_t i_workCells)
{
    // 外周タイルを参照する前に、格子の行列数が正しく、総数の乗算も安全か確認する。
    if (i_grid.Columns() == 0 || i_grid.Rows() == 0 ||
        i_grid.Columns() > std::numeric_limits<std::size_t>::max() / i_grid.Rows())
    {
        throw std::invalid_argument("タイルと地形のボクセル格子が一致しません。");
    }

    // 地形全体と格子全体の両端を取得する。途中の境界も含めて同じ範囲を分割しているか後で照合する。
    const auto first = i_grid.TileBounds(0, 0);
    const auto last = i_grid.TileBounds(i_grid.Columns() - 1, i_grid.Rows() - 1);
    const auto& bounds = i_field.bounds;
    const auto cell = static_cast<double>(i_field.settings.cellSize);

    // 配列の寸法・地形の実寸・セル幅が一致し、タイル格子の外周も地形と同じ場合だけ処理を続ける。
    if (!HasValidHeightfieldCellLayout(i_field, cell) ||
        !HasMatchingTileBounds(bounds, first, last))
    {
        throw std::invalid_argument("タイルと地形のボクセル格子が一致しません。");
    }

    // タイルを作成したときと同じ境界座標を使う。再計算で生じる丸め差により、
    // セルの担当先が境界の両側で食い違うことを防ぐ。
    auto x = std::vector<double>{bounds.minimum.x};
    auto z = std::vector<double>{bounds.minimum.z};
    for (auto column = std::size_t{1}; column < i_grid.Columns(); ++column)
    {
        x.push_back(i_grid.TileBounds(column, 0).minimum.x);
    }
    for (auto row = std::size_t{1}; row < i_grid.Rows(); ++row)
    {
        z.push_back(i_grid.TileBounds(0, row).minimum.z);
    }
    x.push_back(bounds.maximum.x);
    z.push_back(bounds.maximum.z);

    // X 列と Z 行を別々に調べ、各セルを中心位置に応じて一つのタイルへ割り当てる。
    const auto columns = Owners(i_field.width, bounds.minimum.x, bounds.maximum.x, cell, x);
    const auto rows = Owners(i_field.depth, bounds.minimum.z, bounds.maximum.z, cell, z);

    // 各タイルの出力担当範囲に加えて、境界に触れるセルと余白を計算用に確保する。
    // 行を先、列を後に走査することで、返却配列を行優先の順にする。
    auto result = std::vector<TileRegion>{};
    result.reserve(i_grid.Columns() * i_grid.Rows());
    for (auto row = std::size_t{}; row < i_grid.Rows(); ++row)
    {
        for (auto column = std::size_t{}; column < i_grid.Columns(); ++column)
        {
            const auto core = CellRect{columns[column].first, rows[row].first,
                                       columns[column].second, rows[row].second};
            const auto tileBounds = i_grid.TileBounds(column, row);

            // タイルの座標範囲に触れるセルをまず求める。境界がセルの途中にある場合、
            // 担当セルだけではタイル面の一部を計算できないため、両側のセルを含める。
            const auto coveredX = Cover(tileBounds.minimum.x, tileBounds.maximum.x,
                                        bounds.minimum.x, cell, i_field.width);
            const auto coveredZ = Cover(tileBounds.minimum.z, tileBounds.maximum.z,
                                        bounds.minimum.z, cell, i_field.depth);
            const auto covered = CellRect{coveredX.first, coveredZ.first, coveredX.second, coveredZ.second};

            // 接触セルの外側に指定余白を加えて距離計算範囲とし、さらに接続確認用の範囲を追加する。
            const auto managed = Expand(covered, i_borderCells, i_field.width, i_field.depth);
            result.push_back({{column, row}, tileBounds, core, managed,
                              Expand(managed, i_workCells, i_field.width, i_field.depth)});
        }
    }
    return result;
}
}
