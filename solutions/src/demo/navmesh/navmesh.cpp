/**
 * @file navmesh.cpp
 * @brief Navmesh 作成の窓口と現在の構築結果を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#include "navmesh.h"
#include "terrain/terrain_input.h"
#include <optional>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace demo::navmesh
{
namespace
{
/**
 * @brief 更新の成功または例外による終了までの実時間を記録する。
 */
class TimingScope final
{
public:
    /**
     * @brief 記録先を指定して計測を開始する。
     * @param o_result 更新終了時の計測先。
     */
    explicit TimingScope(std::optional<UpdateTiming>& o_result) noexcept
        : m_instResult(o_result), m_instStart(std::chrono::steady_clock::now()) {}
    /**
     * @brief 無操作でなければ終了時間と成否を保存する。
     */
    ~TimingScope()
    {
        if (m_bRecord)
        {
            const auto elapsed = std::chrono::steady_clock::now() - m_instStart;
            m_instResult = UpdateTiming{std::chrono::duration<double, std::milli>(elapsed).count(), m_bSucceeded};
        }
    }
    /**
     * @brief 更新が成功したことを記録する。
     */
    void Succeed() noexcept { m_bSucceeded = true; }
    /**
     * @brief 実際の更新を行わなかった場合は記録を取り消す。
     */
    void Cancel() noexcept { m_bRecord = false; }
private:
    std::optional<UpdateTiming>& m_instResult;             // 結果の記録先。
    std::chrono::steady_clock::time_point m_instStart;     // 計測開始時刻。
    bool m_bSucceeded = false;                            // 操作が成功したか。
    bool m_bRecord = true;                                // 結果を記録するか。
};

/**
 * @brief 地形の幅にかかわらず全構築設定を検証する。
 * @param i_settings 検証する設定。
 */
void ValidateSettings(const NavmeshSettings& i_settings)
{
    // 幅のある地形が与えられる前でも、分割数・ラスタライズ・歩行条件を一括検査する。
    static_cast<void>(TileGrid({{0, 0, 0}, {1, 0, 1}}, i_settings.tileColumns, i_settings.tileRows));
    voxelization::ValidateRasterSettings(i_settings.raster);
    voxelization::ValidateWalkableSettings(i_settings.voxel.walkable, i_settings.raster.cellHeight);
    region::ValidatePartitionMethod(i_settings.voxel.partitionMethod);

    // 指定余白がキャラクター半径に必要な最小セル数を満たすかを確認する。
    const auto minimum = voxelization::MinimumTileBorder(i_settings.raster.cellSize, i_settings.voxel.agentRadius);
    if (i_settings.voxel.borderCells && *i_settings.voxel.borderCells < minimum)
    {
        throw std::invalid_argument("指定したタイル余白が必要最小値より小さいです。");
    }
}

/**
 * @brief 地形の XZ 幅がある場合に指定された格子を作る。
 * @param i_terrain ワールド空間の地形。
 * @param i_settings タイル分割数。
 * @return XZ の幅がゼロの場合は空、それ以外は構築済み格子。
 */
std::optional<TileGrid> MakeGrid(const Terrain& i_terrain, const NavmeshSettings& i_settings)
{
    const auto& bounds = i_terrain.bounds;
    // XZ のどちらかが平坦な地形はタイル格子を作れないため、未作成状態として表す。
    if (bounds.minimum.x == bounds.maximum.x || bounds.minimum.z == bounds.maximum.z) { return std::nullopt; }
    return TileGrid(bounds, i_settings.tileColumns, i_settings.tileRows);
}

/**
 * @brief 格子がある場合に全タイルのボクセル結果を構築する。
 * @param i_terrain ワールド空間の地形。
 * @param i_grid 同じ地形のタイル格子。
 * @param i_settings ボクセルとキャラクターの設定。
 * @return 格子がない場合は空、それ以外は構築済みのタイル。
 */
std::unique_ptr<voxelization::DynamicVoxelTiles> MakeVoxelTiles(
    const Terrain& i_terrain, const std::optional<TileGrid>& i_grid, const NavmeshSettings& i_settings,
    const std::uint64_t i_nextObstacleId = 1)
{
    // 格子のない地形ではボクセル構築を省き、幅がある場合だけ全タイルを初期化する。
    if (!i_grid) { return nullptr; }
    return std::make_unique<voxelization::DynamicVoxelTiles>(i_terrain, *i_grid,
                                                            i_settings.raster, i_settings.voxel,
                                                            std::vector<Bounds>{}, i_nextObstacleId);
}

/**
 * @brief 構築結果の世代が更新可能か検証する。
 * @param i_revision 現在の世代。
 */
void ValidateRevision(const std::uint64_t i_revision)
{
    // 更新のたびに世代を増やすため、最大値では次の世代を表現できない。
    if (i_revision == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::length_error("Navmesh の更新世代が上限に達しました。");
    }
}
}

struct Navmesh::Impl
{
    input::TerrainInput m_instInput;   // 元メッシュと変換済み地形。
    NavmeshSettings m_stSettings;      // 適用済みの構築設定。
    std::optional<TileGrid> m_instGrid; // 生成できた場合のタイル格子。
    std::unique_ptr<voxelization::DynamicVoxelTiles> m_instVoxels; // 各タイルのボクセル結果。
    std::uint64_t m_uNextObstacleId = 1; // 次に発行する障害物の識別子。
    std::uint64_t m_uRevision = 0;       // 表示更新の判定に使う構築結果の世代。
    NavmeshTimings m_stTimings;          // 更新操作ごとの直近の処理時間。
};

/**
 * @brief 内部実装を受け取り Navmesh を構築する。
 * @param i_impl 地形、設定、タイル結果を保持する実装。
 */
Navmesh::Navmesh(std::unique_ptr<Impl> i_impl) noexcept : m_instImpl(std::move(i_impl)) {}

/**
 * @brief 地形と構築結果を所有する内部実装を破棄する。
 */
Navmesh::~Navmesh() = default;

/**
 * @brief Navmesh の内部実装の所有権を移す。
 * @param i_other 所有権を移す元のオブジェクト。
 */
Navmesh::Navmesh(Navmesh&& i_other) noexcept = default;

/**
 * @brief 現在の内部実装を解放し、別オブジェクトの所有権を引き継ぐ。
 * @param i_other 所有権を移す元のオブジェクト。
 * @return 更新後の自身への参照。
 */
Navmesh& Navmesh::operator=(Navmesh&& i_other) noexcept = default;

/**
 * @brief 地形、タイル格子、ボクセルとポリゴンを準備する。
 * @param i_model ステージの CPU モデル。
 * @param i_world 地形 Entity のワールド変換。
 * @param i_settings タイル、ボクセルとキャラクターの設定。
 * @return 現在の実装段階まで準備した結果。ポリゴン間の接続は含まない。
 * @throws std::invalid_argument 地形または設定が不正な場合。
 * @throws std::length_error 構築結果や設定値が上限を超える場合。
 * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
 * @details XZ のいずれかの幅がゼロの場合は TerrainOnly とし、格子を生成しない。
 */
Navmesh CreateNavmesh(const loader::ModelData& i_model, const general::math::Matrix4& i_world,
                      const NavmeshSettings& i_settings)
{
    // 地形の幅に依存する処理より先に設定を検証し、空の格子でも不正設定を通さない。
    ValidateSettings(i_settings);
    auto impl = std::make_unique<Navmesh::Impl>();
    impl->m_stSettings = i_settings;
    // 描画と同じ変換で地形を確定し、その結果に基づいて格子とボクセルタイルを作る。
    impl->m_instInput.SetTerrain(i_model, i_world);
    impl->m_instGrid = MakeGrid(*impl->m_instInput.GetTerrain(), i_settings);
    impl->m_instVoxels = MakeVoxelTiles(*impl->m_instInput.GetTerrain(), impl->m_instGrid, i_settings);
    return Navmesh(std::move(impl));
}

/**
 * @brief 地形から現在の作成処理をワールド変換に合わせてやり直す。
 * @param i_world 地形 Entity の新しいワールド変換。
 * @throws std::invalid_argument 変換後の地形やタイル設定が不正な場合。
 * @throws std::length_error ボクセルなどが上限を超える場合。
 * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
 * @details 障害物はワールド座標と識別子を維持する。障害物がある状態で
 * 格子を作れない幅への変更は拒否し、失敗時は全ての結果を保持する。
 */
void Navmesh::SetWorldTransform(const general::math::Matrix4& i_world)
{
    auto timing = TimingScope(m_instImpl->m_stTimings.transform);
    // 既存入力を複製して新しい変換結果を作り、公開中の地形には触れずに候補を準備する。
    auto input = m_instImpl->m_instInput;
    input.SetWorldTransform(i_world);
    auto grid = MakeGrid(*input.GetTerrain(), m_instImpl->m_stSettings);
    // 障害物のある状態では、格子をなくす変換により更新対象を失うため拒否する。
    if (!grid && m_instImpl->m_instVoxels && !m_instImpl->m_instVoxels->Obstacles().empty())
    {
        throw std::invalid_argument("障害物があるため、幅のない地形には変換できません。");
    }
    ValidateRevision(m_instImpl->m_uRevision);

    // 格子が残る場合は障害物を保って再構築し、初回相当なら次の障害物 ID を渡して構築する。
    auto voxels = grid && m_instImpl->m_instVoxels ?
        m_instImpl->m_instVoxels->Rebuild(*input.GetTerrain(), *grid) :
        MakeVoxelTiles(*input.GetTerrain(), grid, m_instImpl->m_stSettings, m_instImpl->m_uNextObstacleId);
    static_assert(std::is_nothrow_move_assignable_v<input::TerrainInput>);
    static_assert(std::is_nothrow_move_assignable_v<std::optional<TileGrid>>);

    // 候補の構築が完了した後、例外を送出しないムーブ代入で一括反映して世代を進める。
    m_instImpl->m_instInput = std::move(input);
    m_instImpl->m_instGrid = std::move(grid);
    m_instImpl->m_instVoxels = std::move(voxels);
    ++m_instImpl->m_uRevision;
    timing.Succeed();
}

/**
 * @brief 現在の地形と障害物を保持し、新しい設定で全体を再生成する。
 * @param i_settings 再生成に適用する設定。
 * @details 失敗時は設定と構築結果を保持し、失敗時間だけを記録する。
 */
void Navmesh::Rebuild(const NavmeshSettings& i_settings)
{
    auto timing = TimingScope(m_instImpl->m_stTimings.rebuild);
    // 検証と全候補の構築を先に行い、不正な設定で公開中の結果を失わない。
    ValidateSettings(i_settings);
    ValidateRevision(m_instImpl->m_uRevision);
    auto grid = MakeGrid(*m_instImpl->m_instInput.GetTerrain(), i_settings);
    auto voxels = grid && m_instImpl->m_instVoxels ?
        m_instImpl->m_instVoxels->Rebuild(*m_instImpl->m_instInput.GetTerrain(), *grid,
                                         i_settings.raster, i_settings.voxel) :
        MakeVoxelTiles(*m_instImpl->m_instInput.GetTerrain(), grid, i_settings, m_instImpl->m_uNextObstacleId);

    // 構築が成功した場合だけ設定と格子を交換し、表示側に変更を知らせる。
    m_instImpl->m_stSettings = i_settings;
    m_instImpl->m_instGrid = std::move(grid);
    m_instImpl->m_instVoxels = std::move(voxels);
    ++m_instImpl->m_uRevision;
    timing.Succeed();
}

/**
 * @brief 領域分割方式を変更し、既存タイルの領域、輪郭とポリゴンを再構築する。
 * @param i_method 適用する分割方式。
 * @throws std::invalid_argument 分割方式または構築結果が不正な場合。
 * @throws std::length_error 構築の上限を超える場合。
 * @details 障害物と識別子を保持する。失敗時は設定と構築結果を変更しない。
 */
void Navmesh::SetPartitionMethod(const region::PartitionMethod i_method)
{
    auto timing = TimingScope(m_instImpl->m_stTimings.partition);
    // 無効な方式を拒否し、変更がない場合は再構築と世代更新を行わない。
    region::ValidatePartitionMethod(i_method);
    if (m_instImpl->m_stSettings.voxel.partitionMethod == i_method) { timing.Cancel(); return; }
    ValidateRevision(m_instImpl->m_uRevision);
    // タイル側で全候補を用意してから設定を更新するため、失敗時はこの設定も従来値のまま。
    if (m_instImpl->m_instVoxels) { m_instImpl->m_instVoxels->SetPartitionMethod(i_method); }
    m_instImpl->m_stSettings.voxel.partitionMethod = i_method;
    ++m_instImpl->m_uRevision;
    timing.Succeed();
}

/**
 * @brief 作成処理で準備済みの段階を返す。
 * @return 地形のみ、またはボクセル結果までの準備状態。
 */
NavmeshStage Navmesh::Stage() const noexcept
{
    return m_instImpl->m_instVoxels ? NavmeshStage::VoxelFieldsReady : NavmeshStage::TerrainOnly;
}

/**
 * @brief 適用済みの設定を返す。
 * @return 作成時に指定した設定。
 */
const NavmeshSettings& Navmesh::Settings() const noexcept { return m_instImpl->m_stSettings; }

/**
 * @brief 更新操作ごとの直近の処理時間を返す。
 * @return 計測結果。
 */
const NavmeshTimings& Navmesh::Timings() const noexcept { return m_instImpl->m_stTimings; }

/**
 * @brief デバッグ表示などで利用する地形範囲を返す。
 * @return ワールド空間の地形範囲。
 * @details 参照は変換更新またはオブジェクトの破棄まで有効。
 */
const Bounds& Navmesh::TerrainBounds() const noexcept { return m_instImpl->m_instInput.GetTerrain()->bounds; }

/**
 * @brief 作成済みのタイル格子を参照する。
 * @return XZ の幅が不足する場合は nullptr。
 * @details 参照は変換更新またはオブジェクトの破棄まで有効。
 */
const TileGrid* Navmesh::Grid() const noexcept
{
    return m_instImpl->m_instGrid ? &*m_instImpl->m_instGrid : nullptr;
}

/**
 * @brief タイルごとの歩行面と距離場を読み取り専用で参照する。
 * @return XZ の幅が不足する場合は nullptr。
 * @details 参照は変換更新またはオブジェクトの破棄まで有効。
 */
const voxelization::DynamicVoxelTiles* Navmesh::VoxelTiles() const noexcept
{
    return m_instImpl->m_instVoxels.get();
}

/**
 * @brief 地形の三角形をワールド座標で参照する。
 * @return 地形入力。
 * @details 参照は変換更新またはオブジェクトの破棄まで有効。
 */
const Terrain& Navmesh::TerrainData() const noexcept { return *m_instImpl->m_instInput.GetTerrain(); }

/**
 * @brief 箱型障害物を追加して影響タイルを再構築する。
 * @param i_bounds ワールド空間の障害物。
 * @return 個別削除に使える識別子。
 * @throws std::logic_error ボクセル結果がない場合。
 * @throws std::invalid_argument 障害物が不正な場合。
 * @throws std::length_error 構築の上限を超える場合。
 * @details 失敗時は障害物と全タイルを保持する。
 */
std::uint64_t Navmesh::AddObstacle(const Bounds& i_bounds)
{
    auto timing = TimingScope(m_instImpl->m_stTimings.obstacleAdd);
    // ボクセルタイルの存在と世代上限を先に確認し、成功した更新だけを世代へ反映する。
    if (!m_instImpl->m_instVoxels) { throw std::logic_error("地形ボクセルがないため障害物を追加できません。"); }
    ValidateRevision(m_instImpl->m_uRevision);
    const auto id = m_instImpl->m_instVoxels->AddObstacle(i_bounds);
    m_instImpl->m_uNextObstacleId = id + 1;
    ++m_instImpl->m_uRevision;
    timing.Succeed();
    return id;
}

/**
 * @brief 指定した障害物を削除して影響タイルを再構築する。
 * @param i_id 追加時の識別子。
 * @throws std::logic_error ボクセル結果がない場合。
 * @throws std::invalid_argument 識別子が存在しない場合。
 * @details 失敗時は障害物と全タイルを保持する。
 */
void Navmesh::RemoveObstacle(const std::uint64_t i_id)
{
    auto timing = TimingScope(m_instImpl->m_stTimings.obstacleRemove);
    // 削除をタイル側へ委譲し、成功した場合に限って表示更新用の世代を進める。
    if (!m_instImpl->m_instVoxels) { throw std::logic_error("地形ボクセルがないため障害物を削除できません。"); }
    ValidateRevision(m_instImpl->m_uRevision);
    m_instImpl->m_instVoxels->RemoveObstacle(i_id);
    ++m_instImpl->m_uRevision;
    timing.Succeed();
}

/**
 * @brief 全障害物を削除して影響タイルを再構築する。
 * @throws std::logic_error ボクセル結果がない場合。
 * @details 失敗時は障害物と全タイルを保持する。
 */
void Navmesh::ClearObstacles()
{
    auto timing = TimingScope(m_instImpl->m_stTimings.obstacleClear);
    // ボクセル結果がない状態は呼び出し誤りとして通知し、空一覧なら更新を省略する。
    if (!m_instImpl->m_instVoxels) { throw std::logic_error("地形ボクセルがないため障害物を削除できません。"); }
    if (m_instImpl->m_instVoxels->Obstacles().empty()) { timing.Cancel(); return; }
    ValidateRevision(m_instImpl->m_uRevision);
    m_instImpl->m_instVoxels->ClearObstacles();
    ++m_instImpl->m_uRevision;
    timing.Succeed();
}

/**
 * @brief 地形や障害物の更新を識別する世代を返す。
 * @return 初期構築後の更新ごとに変わる値。
 */
std::uint64_t Navmesh::Revision() const noexcept { return m_instImpl->m_uRevision; }
}
