/**
 * @file dynamic_tiles.cpp
 * @brief 障害物の変更が及ぶタイルだけを作り直し、結果を一括で反映する。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#include "dynamic_tiles.h"
#include "../erosion/core_erosion.h"
#include "../erosion/filtered_walkable.h"
#include "../../region/monotone/monotone.h"
#include "../../region/watershed/watershed.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace demo::navmesh::voxelization
{
namespace
{
constexpr auto kMaxWorkCells = std::size_t{8000000};
constexpr auto kDistanceGuard = std::size_t{1};

/**
 * @brief 共通格子のセルが指定範囲に含まれるか判定する。
 * @param i_rect 判定範囲。
 * @param i_x X セル添字。
 * @param i_z Z セル添字。
 * @return 範囲内の場合は true。
 */
bool Contains(const CellRect& i_rect, const std::size_t i_x, const std::size_t i_z)
{
    // 矩形の終端は含めないため、隣り合う範囲が同じセルを重複して扱わない。
    return i_x >= i_rect.beginX && i_x < i_rect.endX &&
           i_z >= i_rect.beginZ && i_z < i_rect.endZ;
}

/**
 * @brief セル位置を地形全体の座標に対応させる寸法情報を作る。
 * @param i_base 元の地形の高さ場。
 * @param i_region 対象タイルの範囲。
 * @return 多角形の形状を実際の地形寸法に合わせるための情報。
 */
polygon::GridMetric PolygonMetric(const Heightfield& i_base, const TileRegion& i_region)
{
    // タイル内のセル座標を地形全体のワールド座標へ変換できるよう、
    // 元地形の寸法と計算範囲の開始セルを一緒に渡す。
    return {i_base.settings.cellSize,
            static_cast<double>(i_base.bounds.maximum.x) - i_base.bounds.minimum.x,
            static_cast<double>(i_base.bounds.maximum.z) - i_base.bounds.minimum.z,
            i_region.work.beginX, i_region.work.beginZ};
}

/**
 * @brief 通行できる面を指定方法で領域に分ける。
 * @param i_method 面をまとめる方法。
 * @param i_field キャラクター寸法による除外判定後の面と接続情報。
 * @param i_core このタイルが出力を担当する面の添字。
 * @param i_eroded 面が通行不可と判定されたかを示す配列。
 * @param i_distance 判定後の面から障害物までの距離情報。
 * @return タイル内の各面が属する領域。
 * @details 指定された分割方法だけを切り替え箇所に集約し、無効な方法は
 * 領域作成に入る前に拒否する。Watershed では平滑化距離も使用する。
 */
region::Regions BuildRegions(const region::PartitionMethod i_method, const WalkableField& i_field,
                             const std::vector<std::size_t>& i_core, const std::vector<std::uint8_t>& i_eroded,
                             const TileDistance& i_distance)
{
    region::ValidatePartitionMethod(i_method);
    if (i_method == region::PartitionMethod::Watershed)
    {
        // Watershed は面ごとの距離の高低を使って、障害物境界を意識した領域を作る。
        return region::BuildWatershedRegions(i_field, i_core, i_eroded, i_distance.smoothed);
    }
    // Monotone は距離に依存せず、指定された通行面を単調な領域へ分割する。
    return region::BuildMonotoneRegions(i_field, i_core, i_eroded);
}

/**
 * @brief 地形と障害物から一つのタイルの通行面と形状データを作る。
 * @param i_base 障害物を重ねる前の地形データ。
 * @param i_region 出力範囲と周辺の計算範囲。
 * @param i_settings 通行条件と面のまとめ方。
 * @param i_obstacles このタイルの作成に反映する障害物。
 * @return 通行面、距離、領域、輪郭、多角形を含むタイル結果。
 * @details 周辺の面は境界の接続確認に使い、タイルが出力を担当する面だけを最終形状に含める。
 */
VoxelTile BuildTile(const Heightfield& i_base, const TileRegion& i_region,
                    const TileBuildSettings& i_settings, const std::vector<BoxObstacle>& i_obstacles)
{
    // タイルの作業範囲に障害物を適用し、通行条件を満たす高さ区間と面の接続を得る。
    auto heightfield = ApplyBoxObstacles(i_base, i_region, i_obstacles);
    auto walkable = BuildWalkableField(heightfield, i_settings.walkable);
    auto managed = std::vector<std::uint8_t>(walkable.spans.size());
    auto coreSpans = std::vector<std::size_t>{};

    // 作業範囲の全ての面について、距離計算に使う管理範囲と、
    // このタイルが最終形状を出力する中核範囲を別々に記録する。
    for (auto index = std::size_t{}; index < walkable.spans.size(); ++index)
    {
        const auto x = i_region.work.beginX + walkable.spans[index].x;
        const auto z = i_region.work.beginZ + walkable.spans[index].z;
        managed[index] = Contains(i_region.managed, x, z) ? 1 : 0;
        if (Contains(i_region.core, x, z)) { coreSpans.push_back(index); }
    }
    // 通行可否の判定結果を管理範囲の外縁まで正しく求めるため、
    // まず隣接セル分を加え、さらにキャラクター半径分だけ入力側の範囲を広げる。
    const auto target = ExpandCellRect(i_region.managed, kDistanceGuard, i_base.width, i_base.depth);
    const auto radiusPadding = MinimumTileBorder(i_base.settings.cellSize, i_settings.agentRadius) - 1;
    const auto sources = ExpandCellRect(target, radiusPadding, i_base.width, i_base.depth);
    const auto erosionRegion = TileRegion{i_region.coordinate, i_region.bounds, target, sources, i_region.work};
    auto sourceMask = std::vector<std::uint8_t>(walkable.spans.size());
    auto targets = std::vector<std::size_t>{};
    // 判定に必要な周囲の面と、判定結果を記録する面を分けて渡す。
    for (auto index = std::size_t{}; index < walkable.spans.size(); ++index)
    {
        const auto x = i_region.work.beginX + walkable.spans[index].x;
        const auto z = i_region.work.beginZ + walkable.spans[index].z;
        sourceMask[index] = Contains(sources, x, z) ? 1 : 0;
        if (Contains(target, x, z)) { targets.push_back(index); }
    }
    // 周辺の入力面を使ってキャラクター半径を考慮した判定を行い、
    // 判定済みの結果から通行可能な面だけを後続処理用に取り出す。
    auto eroded = BuildCoreErosion(heightfield, erosionRegion, walkable, sourceMask, targets, i_settings.agentRadius);
    auto retained = FilterErodedWalkable(walkable, eroded);
    // 判定前の地形距離は表示・比較用に元の通行面から作り、
    // 領域分割に使う距離は管理範囲内かつ通行可能な面だけから作る。
    auto active = managed;
    for (auto index = std::size_t{}; index < active.size(); ++index)
    {
        if (eroded[index]) { active[index] = 0; }
    }
    auto terrainDistance = BuildTileDistance(walkable, managed, i_base.settings.cellSize, i_settings.agentRadius);
    auto distance = BuildTileDistance(retained, active, i_base.settings.cellSize, i_settings.agentRadius);

    // 中核範囲の面だけを出力対象として領域・輪郭・多角形を順に生成する。
    // 周辺面は隣接タイルとの境界処理に必要なため、タイル結果には残しておく。
    auto regions = BuildRegions(i_settings.partitionMethod, retained, coreSpans, eroded, distance);
    auto contours = contour::BuildContours(retained, coreSpans, eroded, regions);
    auto polygonMesh = polygon::BuildPolygonMesh(retained, coreSpans, eroded, regions, PolygonMetric(i_base, i_region));
    return {i_region, std::move(heightfield.columns), std::move(walkable),
            std::move(managed), std::move(coreSpans), std::move(distance), std::move(eroded),
            std::move(terrainDistance), std::move(retained), std::move(regions), target,
            std::move(contours), std::move(polygonMesh)};
}
}

/**
 * @brief 地形をセルに分け、指定された障害物を含む全タイルを作成する。
 * @param i_terrain 地形を構成する三角形と地形の範囲。
 * @param i_grid 地形全体を分割するタイル格子。
 * @param i_raster セルの幅など、地形をセルに分ける設定。
 * @param i_settings キャラクター寸法、境界の余白、面のまとめ方。
 * @param i_initialObstacles 最初から反映する障害物。識別子は配列順に割り当てる。
 * @param i_firstObstacleId 初期障害物と以後の追加に使い始める識別子。
 * @throws std::invalid_argument 入力、格子、余白または障害物が不正な場合。
 * @throws std::length_error セル数、計算範囲または識別子が上限を超える場合。
 */
DynamicVoxelTiles::DynamicVoxelTiles(const Terrain& i_terrain, const TileGrid& i_grid,
                                     const RasterSettings& i_raster, const TileBuildSettings& i_settings,
                                     const std::vector<Bounds>& i_initialObstacles,
                                     const std::uint64_t i_firstObstacleId)
    : m_stBase(Rasterize(i_terrain, i_raster)), m_stSettings(i_settings),
      m_instMinimumBorder(MinimumTileBorder(i_raster.cellSize, i_settings.agentRadius)),
      m_instBorder(i_settings.borderCells.value_or(m_instMinimumBorder)),
      m_instNextId(i_firstObstacleId)
{
    // 分割方法、障害物識別子、タイル境界の余白を検証する。
    // 余白はキャラクター半径による境界判定に不足しないことが必要。
    region::ValidatePartitionMethod(i_settings.partitionMethod);
    if (i_firstObstacleId == 0) { throw std::invalid_argument("障害物の開始識別子が不正です。"); }
    if (m_instBorder < m_instMinimumBorder)
    {
        throw std::invalid_argument("指定したタイル余白が必要最小値より小さいです。");
    }
    // タイル範囲を先に確定して作業セル数を合計する。
    // タイルを実際に構築する前に上限を確認し、過大な作業領域の確保を防ぐ。
    const auto regions = BuildTileRegions(m_stBase, i_grid, m_instBorder,
                                          m_instMinimumBorder + kDistanceGuard);
    auto total = std::size_t{};
    for (const auto& region : regions)
    {
        const auto cells = (region.work.endX - region.work.beginX) * (region.work.endZ - region.work.beginZ);
        if (cells > kMaxWorkCells - total)
        {
            throw std::length_error("タイルの合計作業セル数が上限を超えました。");
        }
        total += cells;
    }

    // 初期障害物を順番に検証し、識別子を重複・オーバーフローさせずに割り当てる。
    // 全ての障害物を登録してから各タイルを作ることで、初期状態を一貫させる。
    m_instObstacles.reserve(i_initialObstacles.size());
    for (const auto& bounds : i_initialObstacles)
    {
        ValidateBoxObstacle(bounds);
        if (m_instNextId == std::numeric_limits<std::uint64_t>::max())
        {
            throw std::length_error("障害物の識別子が上限に達しました。");
        }
        m_instObstacles.push_back({m_instNextId++, bounds});
    }

    // すべてのタイルを作成し、そのポリゴンと領域を使ってタイル間の接続を構築する。
    // 接続グラフは全タイルを参照するため、個々のタイル構築後にまとめて作る。
    m_instTiles.reserve(regions.size());
    for (const auto& region : regions)
    {
        m_instTiles.push_back(BuildTile(m_stBase, region, m_stSettings, m_instObstacles));
    }
    auto views = std::vector<polygon::PolygonLinkTile>{};
    views.reserve(m_instTiles.size());
    for (const auto& tile : m_instTiles) { views.push_back({&tile, &tile.polygonMesh, &tile.regions}); }
    m_stLinks = polygon::BuildPolygonLinks(views);
}

/**
 * @brief 現在の障害物を保ったまま、新しい地形と格子で全タイルを作り直す。
 * @param i_terrain 作り直し後の地形。
 * @param i_grid 新しい地形全体を分割するタイル格子。
 * @return 作り直した結果。次に使う障害物識別子も引き継ぐ。
 * @details 障害物の位置はワールド座標のまま保つ。失敗時は現在の結果を変更しない。
 */
std::unique_ptr<DynamicVoxelTiles> DynamicVoxelTiles::Rebuild(const Terrain& i_terrain, const TileGrid& i_grid) const
{
    return Rebuild(i_terrain, i_grid, m_stBase.settings, m_stSettings);
}

/**
 * @brief 障害物を保持し、指定された設定で全タイルを再構築する。
 * @param i_terrain 再構築する地形。
 * @param i_grid 新しいタイル格子。
 * @param i_raster 新しいセル設定。
 * @param i_settings 新しい歩行条件と境界設定。
 * @return 再構築したタイル。失敗時は現在の結果を変更しない。
 */
std::unique_ptr<DynamicVoxelTiles> DynamicVoxelTiles::Rebuild(
    const Terrain& i_terrain, const TileGrid& i_grid, const RasterSettings& i_raster,
    const TileBuildSettings& i_settings) const
{
    // 障害物の範囲を取り出し、新しい地形と格子に対して全タイルを新規構築する。
    // 元のオブジェクトを書き換えないため、構築に失敗しても現在の結果は維持される。
    auto bounds = std::vector<Bounds>{};
    bounds.reserve(m_instObstacles.size());
    for (const auto& obstacle : m_instObstacles) { bounds.push_back(obstacle.bounds); }
    auto result = std::make_unique<DynamicVoxelTiles>(i_terrain, i_grid, i_raster, i_settings, bounds);
    // 新しい構築で一時的に割り当てられた識別子を元の一覧に置き換え、
    // 次回追加時に既存の識別子を再利用しないよう、次の番号も引き継ぐ。
    result->m_instObstacles = m_instObstacles;
    result->m_instNextId = m_instNextId;
    return result;
}

/**
 * @brief 通行可能な面をまとめる方法を変え、全タイルの形状データを作り直す。
 * @param i_method 使用するまとめ方。
 * @throws std::invalid_argument 指定方法または距離情報が不正な場合。
 * @throws std::length_error 作成する領域数が上限を超える場合。
 * @details 失敗時は現在の設定と全タイルを保つ。障害物と識別子は変更しない。
 */
void DynamicVoxelTiles::SetPartitionMethod(const region::PartitionMethod i_method)
{
    // 分割方法を検証し、変更がない場合は不要な再計算と参照無効化を避ける。
    region::ValidatePartitionMethod(i_method);
    if (m_stSettings.partitionMethod == i_method) { return; }
    // すべてのタイルについて領域・輪郭・ポリゴンを一時領域に作成する。
    // どこかの作成に失敗しても、既存タイルはまだ変更されない。
    using Replacement = std::tuple<region::Regions, std::vector<contour::Contour>, polygon::PolygonMesh>;
    auto replacements = std::vector<Replacement>{};
    replacements.reserve(m_instTiles.size());
    for (const auto& tile : m_instTiles)
    {
        auto regions = BuildRegions(i_method, tile.retained, tile.coreSpans, tile.eroded, tile.distance);
        auto contours = contour::BuildContours(tile.retained, tile.coreSpans, tile.eroded, regions);
        auto mesh = polygon::BuildPolygonMesh(tile.retained, tile.coreSpans, tile.eroded, regions,
                                              PolygonMetric(m_stBase, tile.region));
        replacements.emplace_back(std::move(regions), std::move(contours), std::move(mesh));
    }

    // 作成候補のポリゴンと領域を使って接続グラフを先に準備する。
    // ここで失敗した場合も、現在公開中のタイルとグラフはそのまま残る。
    auto views = std::vector<polygon::PolygonLinkTile>{};
    views.reserve(m_instTiles.size());
    for (auto index = std::size_t{}; index < m_instTiles.size(); ++index)
    {
        views.push_back({&m_instTiles[index], &std::get<2>(replacements[index]), &std::get<0>(replacements[index])});
    }
    auto links = polygon::BuildPolygonLinks(views);
    static_assert(std::is_nothrow_swappable_v<region::Regions>);
    static_assert(std::is_nothrow_swappable_v<polygon::PolygonMesh>);
    static_assert(std::is_nothrow_swappable_v<polygon::PolygonLinkGraph>);

    // 以降の交換が例外を送出しないことを型で確認したうえで、
    // 全タイルの形状、接続グラフ、設定を新しい分割方法へ切り替える。
    for (auto index = std::size_t{}; index < m_instTiles.size(); ++index)
    {
        using std::swap;
        swap(m_instTiles[index].regions, std::get<0>(replacements[index]));
        swap(m_instTiles[index].contours, std::get<1>(replacements[index]));
        swap(m_instTiles[index].polygonMesh, std::get<2>(replacements[index]));
    }
    using std::swap;
    swap(m_stLinks, links);
    m_stSettings.partitionMethod = i_method;
}

/**
 * @brief 更新対象をすべて作成してから、障害物・タイル・接続情報を切り替える。
 * @param i_affected 更新が必要か調べる障害物の範囲。
 * @param io_obstacles 更新後に保持する障害物一覧。成功時に以前の一覧と交換する。
 * @details 更新後の一覧を使って影響タイルとタイル間接続を先に作る。
 * 途中で失敗した場合、現在の障害物・タイル・接続情報はいずれも変更しない。
 */
void DynamicVoxelTiles::ReplaceObstacles(const std::vector<Bounds>& i_affected,
                                          std::vector<BoxObstacle>& io_obstacles)
{
    auto replacements = std::vector<std::pair<std::size_t, VoxelTile>>{};
    // 変更前後の障害物範囲のいずれかが作業範囲に触れるタイルだけを再計算する。
    // 障害物一覧は候補として受け取り、タイル構築が終わるまでは現在の一覧を保つ。
    for (auto index = std::size_t{}; index < m_instTiles.size(); ++index)
    {
        const auto& region = m_instTiles[index].region;
        if (std::any_of(i_affected.begin(), i_affected.end(), [&](const auto& bounds)
            { return ObstacleTouches(m_stBase, region.work, bounds); }))
        {
            replacements.emplace_back(index, BuildTile(m_stBase, region, m_stSettings, io_obstacles));
        }
    }

    // 変更しない既存タイルと再作成した候補タイルを並べて参照一覧を作り、
    // タイル境界の接続が更新後の組み合わせに一致するグラフを先に構築する。
    auto views = std::vector<polygon::PolygonLinkTile>{};
    views.reserve(m_instTiles.size());
    auto replacementIndex = std::size_t{};
    // 更新しないタイルと作成済みのタイルを組み合わせ、新しい接続一覧を先に作る。
    for (auto index = std::size_t{}; index < m_instTiles.size(); ++index)
    {
        const auto* tile = &m_instTiles[index];
        if (replacementIndex < replacements.size() && replacements[replacementIndex].first == index)
        {
            tile = &replacements[replacementIndex++].second;
        }
        views.push_back({tile, &tile->polygonMesh, &tile->regions});
    }
    auto links = polygon::BuildPolygonLinks(views);

    // 構築がすべて成功した後、例外を送出しない交換だけで障害物一覧、
    // 接続グラフ、該当タイルをまとめて公開状態へ反映する。
    static_assert(std::is_nothrow_swappable_v<VoxelTile>);
    static_assert(std::is_nothrow_swappable_v<polygon::PolygonLinkGraph>);
    m_instObstacles.swap(io_obstacles);
    using std::swap;
    swap(m_stLinks, links);
    for (auto& replacement : replacements)
    {
        swap(m_instTiles[replacement.first], replacement.second);
    }
}

/**
 * @brief 箱型障害物を追加し、その範囲にかかるタイルを更新する。
 * @param i_bounds 障害物のワールド座標での範囲。
 * @return この障害物を後で個別に削除するための識別子。
 * @throws std::invalid_argument 範囲に有限でない値がある、または幅がない場合。
 * @throws std::length_error 識別子が上限に達した場合。
 */
std::uint64_t DynamicVoxelTiles::AddObstacle(const Bounds& i_bounds)
{
    // 不正な範囲と識別子の上限を先に検査し、現在の一覧を候補として複製する。
    ValidateBoxObstacle(i_bounds);
    if (m_instNextId == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::length_error("障害物の識別子が上限に達しました。");
    }
    auto obstacles = m_instObstacles;
    obstacles.push_back({m_instNextId, i_bounds});
    // 候補一覧で影響タイルと接続を更新する。更新が成功した場合に限り識別子を進める。
    ReplaceObstacles({i_bounds}, obstacles);
    return m_instNextId++;
}

/**
 * @brief 識別子で障害物を一つ削除し、その範囲にかかるタイルを更新する。
 * @param i_id AddObstacle が返した識別子。
 * @throws std::invalid_argument 指定された識別子の障害物がない場合。
 * @details 削除前の範囲を使って影響タイルを特定する。更新に失敗した場合は状態を保つ。
 */
void DynamicVoxelTiles::RemoveObstacle(const std::uint64_t i_id)
{
    // 現在の一覧を候補として複製し、指定識別子に一致する障害物を探す。
    // 元の一覧を直接編集しないため、後続のタイル再構築に失敗しても状態を保てる。
    auto obstacles = m_instObstacles;
    const auto found = std::find_if(obstacles.begin(), obstacles.end(),
                                    [i_id](const auto& obstacle) { return obstacle.id == i_id; });
    if (found == obstacles.end())
    {
        throw std::invalid_argument("指定された障害物が見つかりません。");
    }
    const auto bounds = found->bounds;
    obstacles.erase(found);
    // 削除前の範囲を更新対象として渡し、その範囲から障害物を除いたタイルを再作成する。
    ReplaceObstacles({bounds}, obstacles);
}

/**
 * @brief 全障害物を削除し、影響を受けていたタイルを再構築する。
 * @details 更新に失敗した場合は障害物一覧、タイル、タイル間の接続情報を変更しない。
 */
void DynamicVoxelTiles::ClearObstacles()
{
    // 全障害物の範囲を記録し、空の候補一覧を作る。
    // 各範囲に触れるタイルを再構築して、障害物のない地形の状態へ戻す。
    auto affected = std::vector<Bounds>{};
    affected.reserve(m_instObstacles.size());
    for (const auto& obstacle : m_instObstacles) { affected.push_back(obstacle.bounds); }
    auto obstacles = std::vector<BoxObstacle>{};
    ReplaceObstacles(affected, obstacles);
}
}
