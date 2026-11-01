/**
 * @file walkable_validation.cpp
 * @brief 歩行面と八方向の接続の検証を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "walkable_validation.h"
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::voxelization
{

namespace
{
// Directionの各方向に対応するX方向のセルオフセット（左=-1、右=+1、斜め方向はX成分）。
constexpr auto kDirectionX  =   std::array<int, 8>{-1, 1, 0, 0, -1, 1, -1, 1};
// Directionの各方向に対応するZ方向のセルオフセット（前=-1、後ろ=+1、斜め方向はZ成分）。
constexpr auto kDirectionZ  =   std::array<int, 8>{0, 0, -1, 1, -1, -1, 1, 1};
// Directionの各方向に対応する逆方向の添字（例：左↔右、左前↔右後ろ）。
constexpr auto kOpposite    =   std::array<std::size_t, 8>{1, 0, 3, 2, 7, 6, 5, 4};

/**
 * @brief セル寸法と歩行面配列の区切りが整合しているか確認する。
 * @param i_field 確認対象。
 * @return 寸法と区切りが整合している場合は true。
 */
bool HasValidCellLayout(const WalkableField& i_field)
{
    // 幅または奥行きが 0 の場合は計算できないため不正とみなす
    if (i_field.width == 0 || i_field.depth == 0){ return false; }

    // セル数に 1 を加える計算がオーバーフローしないよう、乗算前に確認する
    constexpr auto maxCellCount = std::numeric_limits<std::size_t>::max() - 1;
    if (i_field.width > maxCellCount / i_field.depth)
    {
        return false;
    }

    // オフセット配列の構成がセル数とスパン数に対応しているか確認する。
    const auto cellCount = i_field.width * i_field.depth;
    return  i_field.offsets.size()  ==  cellCount + 1 &&        // オフセット配列のサイズがセル数+1であることを確認
            i_field.offsets.front() ==  0 &&                    // オフセット配列の先頭が 0 であることを確認
            i_field.offsets.back()  ==  i_field.spans.size();   // オフセット配列の末尾がスパン配列のサイズと一致することを確認
}

/**
 * @brief 軸方向の隣接セルが移動量とセル範囲に合っているか確認する。
 * @param i_coordinate  元のセルの座標。
 * @param i_target      移動先セルの座標。
 * @param i_extent      軸方向のセル数。
 * @param i_offset      軸方向の移動量。
 * @return 移動量どおりの隣接セルである場合は true。
 */
bool MatchesNeighborCoordinate(const std::size_t i_coordinate,
                               const std::size_t i_target,
                               const std::size_t i_extent,
                               const int i_offset)
{
    // 方向については『kDirectionX』および『kDirectionZ』に従う
    if (i_offset < 0)
    {
        return i_coordinate > 0 && i_target == i_coordinate - 1;
    }
    if (i_offset > 0)
    {
        return i_coordinate + 1 < i_extent && i_target == i_coordinate + 1;
    }
    return i_target == i_coordinate;
}
}

/**
 * @brief 歩行面の格納順、添字、方向と双方向接続を検証する。
 * @param i_field 確認対象。
 * @throws std::invalid_argument データの整合性が壊れている場合。
 */
void ValidateWalkableField(const WalkableField& i_field)
{
    // セル寸法とオフセット配列の構成、および必要な高さが設定されていることを確認する。
    if (!HasValidCellLayout(i_field) || i_field.requiredHeight == 0)
    {
        throw std::invalid_argument("歩行面のセル構成が不正です。");
    }

    // 各セルのオフセット範囲とスパンの整合性を確認する。
    for (auto cell = std::size_t{}; cell < i_field.offsets.size() - 1; ++cell)
    {
        // スパン配列の開始位置と終了位置が反転、または範囲外の場合は不正とみなす。
        const auto first    =   i_field.offsets[cell];
        const auto last     =   i_field.offsets[cell + 1];
        if (first > last || last > i_field.spans.size())
        {
            throw std::invalid_argument("歩行面のセル区間が不正です。");
        }

        // 各スパンの座標が対応するセルと一致することを確認する。
        for (auto index = first; index < last; ++index)
        {
            const auto& span = i_field.spans[index];

            // スパンの座標がセルの座標と一致することを確認する。
            if (span.x != cell % i_field.width || span.z != cell / i_field.width)
            {
                throw std::invalid_argument("歩行面の座標が不正です。");
            }

            // 天井がある場合は床より上にあり、必要な高さを確保していることを確認する。
            if (span.ceiling &&
                (*span.ceiling < span.floor ||
                static_cast<std::uint64_t>(*span.ceiling) - span.floor < i_field.requiredHeight))
            {
                throw std::invalid_argument("歩行面の高さが不正です（床と天井が反転しているか、必要な高さを確保できていません）。");
            }

            // 同じセル内で床面が高さ順に並んでいることを確認する。（低いほど添え字が小さいはず）
            if (index > first && i_field.spans[index - 1].floor >= span.floor)
            {
                throw std::invalid_argument("歩行面の座標または高さが不正です。");
            }
        }
    }

    // 接続の方向・添字と逆方向の対応を確認し、壊れたグラフを距離計算に渡さない。
    for (auto index = std::size_t{}; index < i_field.spans.size(); ++index)
    {
        // 各スパンの接続を8方向について確認する
        const auto& span = i_field.spans[index];
        for (auto direction = std::size_t{}; direction < kOpposite.size(); ++direction)
        {
            for (const auto target : span.neighbors[direction])
            {
                if (target >= i_field.spans.size())
                {
                    throw std::invalid_argument("歩行面の接続先が範囲外です。");
                }
                const auto& other = i_field.spans[target];
                
                // XZ平面に隣接しているか
                const auto xMatches = MatchesNeighborCoordinate(span.x, other.x, i_field.width, kDirectionX[direction]);
                const auto zMatches = MatchesNeighborCoordinate(span.z, other.z, i_field.depth, kDirectionZ[direction]);

                // 移動先から元の面への逆方向接続も存在することを確認する。
                const auto& reverse = other.neighbors[kOpposite[direction]];
                if (!xMatches || !zMatches || std::find(reverse.begin(), reverse.end(), index) == reverse.end())
                {
                    throw std::invalid_argument("歩行面の方向または逆方向の接続が不正です。");
                }
            }
        }
    }
}
}
