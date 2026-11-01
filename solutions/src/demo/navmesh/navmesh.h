/**
 * @file navmesh.h
 * @brief Navmesh の作成設定と、段階的に構築される結果を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_NAVMESH_H
#define NAVMESH_DEMO_DEMO_NAVMESH_NAVMESH_H

#include "terrain/terrain.h"
#include "voxelization/tiles/dynamic_tiles.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace demo::navmesh
{
/**
 * @brief タイル、ボクセル、キャラクター寸法と余白の構築設定。
 */
struct NavmeshSettings
{
    std::size_t tileColumns = 4; // X 方向のタイル数。
    std::size_t tileRows = 4;    // Z 方向のタイル数。
    voxelization::RasterSettings raster;  // セル幅、高さと最大傾斜角。
    voxelization::TileBuildSettings voxel; // キャラクター寸法と境界余白。
};

/**
 * @brief 更新処理の直近の経過時間と成否。
 */
struct UpdateTiming
{
    double milliseconds = 0.0; // 更新処理にかかった実時間。
    bool succeeded = false;    // 更新が正常終了したか。
};

/**
 * @brief 操作ごとの直近の計測結果。未実行の操作は空。
 */
struct NavmeshTimings
{
    std::optional<UpdateTiming> rebuild;          // 設定による全体再生成。
    std::optional<UpdateTiming> transform;        // 地形の変換更新。
    std::optional<UpdateTiming> partition;        // 領域分割方式の更新。
    std::optional<UpdateTiming> obstacleAdd;      // 障害物の追加。
    std::optional<UpdateTiming> obstacleRemove;   // 障害物の個別削除。
    std::optional<UpdateTiming> obstacleClear;    // 障害物の全削除。
};

/**
 * @brief 現在の作成処理で準備された段階。
 * @details ボクセル結果がある場合、ポリゴン間の接続も生成される。
 */
enum class NavmeshStage
{
    TerrainOnly,   // XZ の幅が不足しており、地形のみ準備済み。
    TileGridReady, // 従来の格子のみの段階。現在の構築処理は返さない。
    VoxelFieldsReady // 全タイルの歩行面、距離場とポリゴンを準備済み。
};

/**
 * @brief Navmesh 作成処理の結果。地形、格子とタイルごとのボクセル結果を保持する。
 * @details ポリゴン間の接続を保持するが、経路探索は実装していない。
 * コピーは不可で、ムーブによって所有権を移動できる。ムーブ元は破棄または再代入のみ可能。
 */
class Navmesh final
{
public:
    ~Navmesh();
    Navmesh(const Navmesh&) = delete;
    Navmesh& operator=(const Navmesh&) = delete;
    Navmesh(Navmesh&& i_other) noexcept;
    Navmesh& operator=(Navmesh&& i_other) noexcept;

    /**
     * @brief 地形から現在の作成処理をワールド変換に合わせてやり直す。
     * @param i_world 地形 Entity の新しいワールド変換。
     * @throws std::invalid_argument 変換後の地形やタイル設定が不正な場合。
     * @throws std::length_error ボクセルなどが上限を超える場合。
     * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
     * @details 障害物はワールド座標と識別子を維持する。障害物がある状態で
     * 格子を作れない幅への変更は拒否し、失敗時は全ての結果を保持する。
     */
    void SetWorldTransform(const general::math::Matrix4& i_world);

    /**
     * @brief 現在の地形と障害物を保持し、新しい設定で全体を再生成する。
     * @param i_settings 再生成に適用する設定。
     * @details 障害物の位置と識別子を保持する。失敗時は設定と構築結果を変更しない。
     * 計測結果は失敗時にも更新される。
     */
    void Rebuild(const NavmeshSettings& i_settings);

    /**
     * @brief 領域分割方式を変更し、既存タイルの領域、輪郭とポリゴンを再構築する。
     * @param i_method 適用する分割方式。
     * @throws std::invalid_argument 分割方式または構築結果が不正な場合。
     * @throws std::length_error 構築の上限を超える場合。
     * @details 障害物と識別子を保持する。失敗時は設定と構築結果を変更しない。
     */
    void SetPartitionMethod(region::PartitionMethod i_method);

    /**
     * @brief 作成処理で準備済みの段階を返す。
     * @return 地形のみ、またはボクセル結果までの準備状態。
     */
    NavmeshStage Stage() const noexcept;

    /**
     * @brief 適用済みの設定を返す。
     * @return 作成時に指定した設定。
     */
    const NavmeshSettings& Settings() const noexcept;

    /**
     * @brief 各更新操作の直近の計測結果を返す。
     * @return 処理時間と成否。未実行または何も変わらない操作は記録しない。
     */
    const NavmeshTimings& Timings() const noexcept;

    /**
     * @brief デバッグ表示などで利用する地形範囲を返す。
     * @return ワールド空間の地形範囲。
     * @details 参照は変換更新またはオブジェクトの破棄まで有効。
     */
    const Bounds& TerrainBounds() const noexcept;

    /**
     * @brief 作成済みのタイル格子を参照する。
     * @return XZ の幅が不足する場合は nullptr。
     * @details 参照は変換更新またはオブジェクトの破棄まで有効。
     */
    const TileGrid* Grid() const noexcept;

    /**
     * @brief タイルごとの歩行面と距離場を読み取り専用で参照する。
     * @return XZ の幅が不足する場合は nullptr。
     * @details 参照は変換更新またはオブジェクトの破棄まで有効。
     * 対象タイルの再構築でも、そのタイル内の参照は無効になる。
     * 分割方式の変更では、各タイルの領域・輪郭・ポリゴンへの参照が無効になる。
     */
    const voxelization::DynamicVoxelTiles* VoxelTiles() const noexcept;

    /**
     * @brief 地形の三角形をワールド座標で参照する。
     * @return 地形入力。
     * @details 参照は変換更新またはオブジェクトの破棄まで有効。
     */
    const Terrain& TerrainData() const noexcept;

    /**
     * @brief 箱型障害物を追加して影響タイルを再構築する。
     * @param i_bounds ワールド空間の障害物。
     * @return 個別削除に使える識別子。
     * @throws std::logic_error ボクセル結果がない場合。
     * @throws std::invalid_argument 障害物が不正な場合。
     * @throws std::length_error 構築の上限を超える場合。
     * @details 失敗時は障害物と全タイルを保持する。
     */
    std::uint64_t AddObstacle(const Bounds& i_bounds);

    /**
     * @brief 指定した障害物を削除して影響タイルを再構築する。
     * @param i_id 追加時の識別子。
     * @throws std::logic_error ボクセル結果がない場合。
     * @throws std::invalid_argument 識別子が存在しない場合。
     * @details 失敗時は障害物と全タイルを保持する。
     */
    void RemoveObstacle(std::uint64_t i_id);

    /**
     * @brief 全障害物を削除して影響タイルを再構築する。
     * @throws std::logic_error ボクセル結果がない場合。
     * @details 失敗時は障害物と全タイルを保持する。
     */
    void ClearObstacles();

    /**
     * @brief 地形や障害物の更新を識別する世代を返す。
     * @return 初期構築後の更新ごとに変わる値。
     */
    std::uint64_t Revision() const noexcept;

private:
    struct Impl;                       // 内部地形と構築結果の保持形式。
    explicit Navmesh(std::unique_ptr<Impl> i_impl) noexcept;
    friend Navmesh CreateNavmesh(const loader::ModelData&, const general::math::Matrix4&,
                                 const NavmeshSettings&);
    std::unique_ptr<Impl> m_instImpl;  // Navmesh の構築結果。
};

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
                      const NavmeshSettings& i_settings = {});
}
#endif
