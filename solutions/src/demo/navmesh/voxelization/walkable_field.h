/**
 * @file walkable_field.h
 * @brief 占有区間から頭上空間と段差を満たす歩行面を作る。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_WALKABLE_FIELD_H
#define NAVMESH_DEMO_DEMO_NAVMESH_VOXELIZATION_WALKABLE_FIELD_H

#include "heightfield.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace demo::navmesh::voxelization
{
/**
 * @brief 歩行面と隣接移動の判定に使用するキャラクター寸法。
 */
struct WalkableSettings
{
    float agentHeight = 2.0F; // 頭上に必要な高さ。
    float maxClimb = 0.5F;   // 上下に移動できる段差の最大値。
};

/**
 * @brief キャラクターの寸法と高さ方向のセル数を検証する。
 * @param i_settings 高さと許容段差。
 * @param i_cellHeight 高さ方向のセル幅。
 * @throws std::invalid_argument 寸法が不正な場合。
 * @throws std::length_error セル数が上限を超える場合。
 */
void ValidateWalkableSettings(const WalkableSettings& i_settings, float i_cellHeight);

/**
 * @brief 隣接セルの方向。
 */
enum class Direction : std::size_t
{
    West,       // X 軸の負方向。
    East,       // X 軸の正方向。
    North,      // Z 軸の負方向。
    South,      // Z 軸の正方向。
    NorthWest,  // X 軸の負方向かつ Z 軸の負方向。
    NorthEast,  // X 軸の正方向かつ Z 軸の負方向。
    SouthWest,  // X 軸の負方向かつ Z 軸の正方向。
    SouthEast,  // X 軸の正方向かつ Z 軸の正方向。
    Count       // 移動方向の数。方向別配列の要素数に使用する。
};

/**
 * @brief キャラクターの高さを確保できる占有区間の上面。
 */
struct WalkableSpan
{
    std::size_t x = 0;                  // X 方向のセル添字。
    std::size_t z = 0;                  // Z 方向のセル添字。
    std::size_t sourceIndex = 0;        // 元のセル内の占有区間の添字。
    std::uint32_t floor = 0;            // 床面の高さセル数。
    std::optional<std::uint32_t> ceiling; // 天井の高さセル数。空は上限なし。
    std::array<std::vector<std::size_t>, static_cast<std::size_t>(Direction::Count)> neighbors;
                                        // 各方向に移動できる歩行面の添字。
};

/**
 * @brief 歩行条件を満たす面と、その間の隣接情報。
 */
struct WalkableField
{
    std::size_t width = 0;                // X 方向のセル数。
    std::size_t depth = 0;                // Z 方向のセル数。
    std::uint32_t requiredHeight = 0;     //キャラクターの高さを確保するために必要な高さ方向のセル数。
    std::uint32_t allowedClimb = 0;       // 切り下げた段差セル数。
    std::vector<std::size_t> offsets;     // 行優先セルごとの面配列の開始位置。末尾は面数。
    std::vector<WalkableSpan> spans;      // セルごと、高さ順の歩行面。
};

/**
 * @brief 傾斜と頭上空間を満たす面を抽出し、段差を満たす隣接面を接続する。
 * @param i_field 占有区間を高さ順に持つ高さ場。
 * @param i_settings キャラクターの高さと最大段差。
 * @return 歩行面と各方向の隣接面。生成された添字は戻り値の spans を指す。
 * @throws std::invalid_argument 高さ場または寸法が不正な場合。
 * @throws std::length_error 面数、接続数または寸法が上限を超える場合。
 * @details 高さはセル単位で切り上げ、段差は切り下げる。
 * 斜め方向は両方の直交方向の経路が存在する場合のみ接続する。
 */
WalkableField BuildWalkableField(const Heightfield& i_field, const WalkableSettings& i_settings);
}
#endif
