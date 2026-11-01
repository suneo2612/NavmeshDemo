/**
 * @file contour_lines.cpp
 * @brief タイルの輪郭辺をデバッグ表示用の線分に変換する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "contour_lines.h"
#include "grid_position.h"
#include <stdexcept>

namespace demo::navmesh::debug
{
namespace
{
constexpr auto kHeightOffset = 0.02;

}

/**
 * @brief 全タイルの外周と穴を描画用の線分に変換する。
 * @param i_navmesh 構築済みのタイルと地形範囲。
 * @return terrain に外周、tiles に穴と退化輪郭の線分。
 * @throws std::invalid_argument タイルがない場合。
 * @throws std::overflow_error ワールド座標を表現できない場合。
 * @details 線は各辺の歩行面の高さから少し上に描画する。
 */
BoundsLines BuildContourLines(const Navmesh& i_navmesh)
{
    const auto* tiles = i_navmesh.VoxelTiles();
    if (!tiles) { throw std::invalid_argument("表示できる地形ボクセルがありません。"); }
    auto result = BoundsLines{};
    // 符号付き面積で外周と穴を表示先に分け、各境界辺をセル角のワールド座標へ変換する。
    for (const auto& tile : tiles->Tiles())
    {
        for (const auto& contour : tile.contours)
        {
            auto& output = contour.signedAreaTwice > 0 ? result.terrain : result.tiles;
            for (const auto& edge : contour.edges)
            {
                const auto& span = tile.retained.spans.at(edge.span);
                const auto x = tile.region.work.beginX + span.x;
                const auto z = tile.region.work.beginZ + span.z;
                const auto nw = GridPosition(i_navmesh, x, z, span.floor, kHeightOffset);
                const auto ne = GridPosition(i_navmesh, x + 1, z, span.floor, kHeightOffset);
                const auto sw = GridPosition(i_navmesh, x, z + 1, span.floor, kHeightOffset);
                const auto se = GridPosition(i_navmesh, x + 1, z + 1, span.floor, kHeightOffset);
                using voxelization::Direction;
                // 輪郭方向に応じたセル辺の両端だけを追加し、領域内部の辺は描画しない。
                switch (edge.direction)
                {
                case Direction::North: output.push_back(nw); output.push_back(ne); break;
                case Direction::East:  output.push_back(ne); output.push_back(se); break;
                case Direction::South: output.push_back(se); output.push_back(sw); break;
                case Direction::West:  output.push_back(sw); output.push_back(nw); break;
                default: throw std::invalid_argument("輪郭辺の方向が不正です。");
                }
            }
        }
    }
    return result;
}
}
