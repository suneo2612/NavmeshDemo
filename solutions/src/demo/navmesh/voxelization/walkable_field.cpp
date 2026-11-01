/**
 * @file walkable_field.cpp
 * @brief 頭上空間と段差に基づく歩行面・隣接関係を生成する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "walkable_field.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh::voxelization
{
namespace
{
// 入力データが極端に大きい場合に、面や接続の生成でメモリを使い切らないための上限。
constexpr auto kMaxWalkableSpans    =   std::size_t{1000000};
constexpr auto kMaxLinks            =   std::size_t{8000000};
// ceiling が未設定の歩行面は、上方向に障害物がないものとして扱う。
constexpr auto kUnboundedCeiling    =   std::numeric_limits<std::uint64_t>::max();

/**
 * @brief 寸法を高さ場の刻みに合わせて安全側に変換する。
 * @param i_value 実空間の寸法。
 * @param i_cellHeight 高さ方向のセル幅。
 * @param i_roundUp 切り上げる場合は true。
 * @return 高さセル数。
 * @details キャラクターの必要な高さは不足しないように切り上げ、許容段差は
 * 実際の上限を超えないように切り下げる。double で除算してから丸め、
 * uint32_t に収まらない値や有限でない値を結果に変換する前に拒否する。
 */
std::uint32_t ToCells(const float i_value, const float i_cellHeight, const bool i_roundUp)
{
    // 実寸をセル幅で割り、指定された安全側の丸め方法を適用する。
    const auto cells    =   static_cast<double>(i_value) / i_cellHeight;        // 高さをセル数に変換
    const auto rounded  =   i_roundUp ? std::ceil(cells) : std::floor(cells);       
    if (!std::isfinite(rounded) || rounded > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::length_error("キャラクター寸法がボクセル高さの上限を超えました。");
    }
    return static_cast<std::uint32_t>(rounded);     // 丸めたセル数を返す
}

/**
 * @brief 高さ場の寸法と列配列の大きさが一致することを検証する。
 * @param i_field 入力高さ場。
 * @throws std::invalid_argument 寸法が 0、乗算がオーバーフローする、または列数が不一致の場合。
 */
void ValidateHeightfieldDimensions(const Heightfield& i_field)
{
    if (i_field.width == 0 || i_field.depth == 0)
    {
        throw std::invalid_argument("高さ場またはキャラクター寸法が不正です。");
    }

    // 幅と奥行きの積を求める前に、size_t の上限を超えないことを確認する。
    if (i_field.width > std::numeric_limits<std::size_t>::max() / i_field.depth)
    {
        throw std::invalid_argument("高さ場またはキャラクター寸法が不正です。");
    }

    // 列配列には、行優先のセル数と同じ数の要素が必要。
    const auto cellCount = i_field.width * i_field.depth;
    if (i_field.columns.size() != cellCount)
    {
        throw std::invalid_argument("高さ場またはキャラクター寸法が不正です。");
    }
}

/**
 * @brief 高さ場の全列で占有区間の順序と範囲を検証する。
 * @param i_field 入力高さ場。
 * @throws std::invalid_argument 空の区間、または重複・未整列の区間がある場合。
 */
void ValidateHeightfieldSpans(const Heightfield& i_field)
{
    // 各列の区間は下から上へ並ぶ必要があり、隣り合う区間が重なったり接したりしてはならない。
    for (const auto& column : i_field.columns)
    {
        auto previous   =   std::uint32_t{};
        auto first      =   true;
        for (const auto& span : column)
        {
            // 区間が空でなく、高さ順に並び、前の区間と重複・接触していないことを確認する。
            if (span.minimum >= span.maximum || (!first && previous >= span.minimum))
            {
                throw std::invalid_argument("高さ場の占有区間が不正または未統合です。");
            }
            previous = span.maximum;
            first = false;
        }
    }
}

/**
 * @brief 高さ場とキャラクター寸法の整合性を検証する。
 * @param i_field 入力高さ場。
 * @param i_settings キャラクター寸法。
 * @details 寸法と区間構造をそれぞれ検証し、最後にキャラクター寸法を高さ場の
 * セル高に対して検証する。生成処理が入力の構造を前提にできるよう、ここで
 * 不正な高さ場を拒否する。
 */
void Validate(const Heightfield& i_field, const WalkableSettings& i_settings)
{
    // 幅・奥行きが正の値で、セル数の積がオーバーフローせず、列数と一致することを確認する。
    ValidateHeightfieldDimensions(i_field);
    // 各列の占有区間が空でなく、高さ順に並び、重複や接触がないことを確認する。
    ValidateHeightfieldSpans(i_field);
    // セル高とキャラクター寸法が正の有限値で、セル数へ安全に変換できることを確認する。
    ValidateWalkableSettings(i_settings, i_field.settings.cellHeight);
}

/**
 * @brief 上部空間と段差の条件を両方の面について判定する。
 * @param i_a 開始面。
 * @param i_b 隣接面。
 * @param i_height 必要な高さセル数。
 * @param i_climb 許容する段差セル数。
 * @return 両方向に移動できる場合は true。
 * @details 床の高さの差が許容段差以内であり、かつ高い方の床から必要な高さ分の
 * 空間を、両方の歩行面に共通する頭上範囲で確保できる場合に接続可能とする。
 */
bool CanConnect(const WalkableSpan& i_a,
                const WalkableSpan& i_b,
                const std::uint32_t i_height,
                const std::uint32_t i_climb)
{
    // 床の高低差を求め、必要な高さの加算は 64 bit で行って整数の桁あふれを避ける。
    const auto lower    =   std::min(i_a.floor, i_b.floor);
    const auto higher   =   std::max(i_a.floor, i_b.floor);
    const auto ceilingA =   i_a.ceiling ? static_cast<std::uint64_t>(*i_a.ceiling) : kUnboundedCeiling;
    const auto ceilingB =   i_b.ceiling ? static_cast<std::uint64_t>(*i_b.ceiling) : kUnboundedCeiling;

    // どちらか低い天井を共通の上限として、段差と通過高さの両方を判定する。
    return  higher - lower <= i_climb &&    // 段差が許容範囲内か
            std::min(ceilingA, ceilingB) >= static_cast<std::uint64_t>(higher) + i_height;
}

/**
 * @brief 隣接面の添字を上限を確認して追加する。
 * @param io_span 追加する歩行面。
 * @param i_direction 移動方向。
 * @param i_target 移動先の歩行面の添字。
 * @param io_count 全方向の接続数。
 * @details 接続は移動元と移動先の両方に登録されるため、この関数は登録する
 * 片方向分を数える。上限を超える場合は部分的な結果を返さず例外を送出する。
 */
void AddLink(WalkableSpan& io_span,
             const Direction i_direction,
             const std::size_t i_target,
             std::size_t& io_count)
{
    if (io_count >= kMaxLinks) { throw std::length_error("歩行面の接続数が上限を超えました。"); }

    // neighbors の方向ごとの配列には、spans 配列上の移動先添字を保持する。
    io_span.neighbors[static_cast<std::size_t>(i_direction)].push_back(i_target);
    ++io_count;
}

/**
 * @brief 隣接する二つのセルについて、条件を満たす全ての階層を双方向に接続する。
 * @param io_result 更新対象。
 * @param i_first 開始セルの線形添字。
 * @param i_second 隣接セルの線形添字。
 * @param i_direction 開始セルからの方向。
 * @param i_opposite 隣接セルからの方向。
 * @param io_count 全方向の接続数。
 * @details セル内に複数の階層がある場合も考慮し、二列に属する全ての歩行面の
 * 組み合わせを調べる。条件を満たす組だけを、互いの逆方向にも登録する。
 */
void ConnectColumns(WalkableField& io_result,
                    const std::size_t i_first,
                    const std::size_t i_second,
                    const Direction i_direction,
                    const Direction i_opposite,
                    std::size_t& io_count)
{
    // offsets は各セルに属する spans の開始・終了位置を示すため、二重ループで各面を列挙できる。
    for (auto a = io_result.offsets[i_first]; a < io_result.offsets[i_first + 1]; ++a)
    {
        for (auto b = io_result.offsets[i_second]; b < io_result.offsets[i_second + 1]; ++b)
        {
            // 条件を満たす場合にのみ接続を追加する。
            if (CanConnect(io_result.spans[a], io_result.spans[b], io_result.requiredHeight, io_result.allowedClimb))
            {
                // 条件を満たす場合に、互いの逆方向にも接続を追加する。
                AddLink(io_result.spans[a], i_direction, b, io_count);
                AddLink(io_result.spans[b], i_opposite, a, io_count);
            }
        }
    }
}

/**
 * @brief 指定した順序で二方向に移動し、目的の歩行面へ到達できるかを調べる。
 * @param i_result 構築中の歩行面と、その面どうしの隣接情報。
 * @param i_from 経路の開始となる歩行面の添字。
 * @param i_to 経路の到達先となる歩行面の添字。
 * @param i_first 開始面から中間面へ進む方向。
 * @param i_second 中間面から到達先へ進む方向。
 * @return 指定した二段階の接続が存在する場合は true。
 * @details 開始面から i_first 方向に接続している歩行面を中間候補として調べ、
 * その同じ面から i_second 方向に到達先へ接続している候補があれば true を返す。
 * 接続情報は先に段差と頭上空間を判定して作成済みのため、ここでは隣接リストを
 * 確認する。対角移動の安全確認では、二方向の順序を入れ替えた経路も呼び出し側で調べる。
 */
bool HasPath(const WalkableField& i_result,
             const std::size_t i_from,
             const std::size_t i_to,
             const Direction i_first,
             const Direction i_second)
{
    // 開始面から第一方向に進める面を、中間地点の候補として一つずつ取り出す。
    for (const auto intermediate : i_result.spans[i_from].neighbors[static_cast<std::size_t>(i_first)])
    {
        // その中間面の第二方向の接続先に、目的の歩行面が含まれているか調べる。
        const auto& targets = i_result.spans[intermediate].neighbors[static_cast<std::size_t>(i_second)];
        if (std::find(targets.begin(), targets.end(), i_to) != targets.end()) { return true; }
    }
    // どの中間面を経由しても目的地に接続していない場合、この順序の経路は存在しない。
    return false;
}

/**
 * @brief 角抜けがない対角方向の接続を双方向に追加する。
 * @param io_result 更新対象。
 * @param i_from 開始セルの線形添字。
 * @param i_to 対角セルの線形添字。
 * @param i_x X 方向の移動。
 * @param i_z Z 方向の移動。
 * @param i_direction 対角方向。
 * @param i_opposite 逆の対角方向。
 * @param io_count 全方向の接続数。
 * @details 対角線上の二面が段差・頭上空間の条件を満たすだけでは接続しない。
 * X→Z と Z→X の両経路で直交方向の移動が可能な場合に限り、角をすり抜けない
 * 対角接続として両方向に登録する。
 */
void ConnectDiagonal(WalkableField& io_result,
                     const std::size_t i_from,
                     const std::size_t i_to,
                     const Direction i_x,
                     const Direction i_z,
                     const Direction i_direction,
                     const Direction i_opposite,
                     std::size_t& io_count)
{
    // offsets が示す範囲から、開始セルに属する全ての歩行面を順に取り出す。
    for (auto a = io_result.offsets[i_from]; a < io_result.offsets[i_from + 1]; ++a)
    {
        // 対角セルにも複数の高さの面があり得るため、面の組み合わせごとに接続可否を調べる。
        for (auto b = io_result.offsets[i_to]; b < io_result.offsets[i_to + 1]; ++b)
        {
            // 二面間の段差と頭上空間を確認し、X→Z と Z→X の両経路がある場合だけ接続する。
            if (CanConnect(io_result.spans[a], io_result.spans[b], io_result.requiredHeight, io_result.allowedClimb) &&
                HasPath(io_result, a, b, i_x, i_z) &&
                HasPath(io_result, a, b, i_z, i_x))
            {
                // 対角方向とその逆方向の隣接リストに、互いの歩行面の添字を登録する。
                AddLink(io_result.spans[a], i_direction, b, io_count);
                AddLink(io_result.spans[b], i_opposite, a, io_count);
            }
        }
    }
}
}

/**
 * @brief キャラクターの寸法と高さ方向のセル数を検証する。
 * @param i_settings 高さと許容段差。
 * @param i_cellHeight 高さ方向のセル幅。
 * @throws std::invalid_argument 寸法が不正な場合。
 * @throws std::length_error セル数が上限を超える場合。
 */
void ValidateWalkableSettings(const WalkableSettings& i_settings, const float i_cellHeight)
{
    // セル高とキャラクター寸法を先に検査し、ゼロ除算や負の寸法での量子化を防ぐ。
    if (!std::isfinite(i_cellHeight) || i_cellHeight <= 0 ||
        !std::isfinite(i_settings.agentHeight) || i_settings.agentHeight <= 0 ||
        !std::isfinite(i_settings.maxClimb) || i_settings.maxClimb < 0)
    {
        throw std::invalid_argument("高さ場またはキャラクター寸法が不正です。");
    }
    // 寸法が uint32_t のセル数に変換可能かも事前に確認する。
    static_cast<void>(ToCells(i_settings.agentHeight, i_cellHeight, true));
    static_cast<void>(ToCells(i_settings.maxClimb, i_cellHeight, false));
}

/**
 * @brief 高さ場から歩行面と面どうしの接続グラフを構築する。
 * @param i_field 高さ順の占有区間を持つ入力高さ場。
 * @param i_settings キャラクターの高さと許容段差。
 * @return セルごとの歩行面と、方向別の隣接面添字。
 * @details 歩行できる面を抽出した後、隣り合う面を四方向、続いて対角方向に接続する。
 * 対角接続は、間の直交方向の経路が両方とも通れる場合に限る。
 */
WalkableField BuildWalkableField(const Heightfield& i_field, const WalkableSettings& i_settings)
{
    Validate(i_field, i_settings);

    // 出力のセル数を入力と揃え、キャラクター寸法をセル数に直す。
    // 必要な高さは切り上げ、許容段差は切り下げる。
    auto result             =   WalkableField{};
    result.width            =   i_field.width;
    result.depth            =   i_field.depth;
    result.requiredHeight   =   ToCells(i_settings.agentHeight, i_field.settings.cellHeight, true);
    result.allowedClimb     =   ToCells(i_settings.maxClimb, i_field.settings.cellHeight, false);

    // 各セルの開始位置と最後の終了位置を記録するため、セル数 + 1 個分を確保する。
    result.offsets.reserve(i_field.columns.size() + 1);

    // 地面を上から見た各セル領域を順に調べ、その真上にある歩行面をまとめて格納する。
    for (auto columnIndex = std::size_t{}; columnIndex < i_field.columns.size(); ++columnIndex)
    {
        // このセル領域に属する歩行面の開始位置を記録する。歩行面がないセルも区切りを残す。
        result.offsets.push_back(result.spans.size());

        // セル内の占有区間を下から順に調べ、上面を歩行面にできるか確認する。
        const auto& column = i_field.columns[columnIndex];
        for (auto index = std::size_t{}; index < column.size(); ++index)
        {
            const auto& solid = column[index];

            // 区間の上端を床とし、次の区間の下端を天井とする。次の区間がなければ天井はない。
            const auto ceiling = index + 1 < column.size() ? std::optional<std::uint32_t>{column[index + 1].minimum}
                                                           : std::nullopt;
            // 傾斜条件を満たし、天井がある場合は必要な高さも確保できる面だけを残す。
            if (!solid.walkableTop || 
                (ceiling && *ceiling - solid.maximum < result.requiredHeight)) 
            {
                continue;
            }
            
            // 面数の上限を超えるデータは生成しない。
            if (result.spans.size() >= kMaxWalkableSpans)
            {
                throw std::length_error("歩行面の数が上限を超えました。");
            }
            
            // セル座標、床と天井の高さ、元の占有区間番号を歩行面に保存する。
            result.spans.push_back({columnIndex % result.width,
                                    columnIndex / result.width,
                                    index, solid.maximum, ceiling, {}});
        }
    }
    // 最後のセル領域の終端を記録し、各セル領域の真上にある歩行面を後から区別できるようにする。
    result.offsets.push_back(result.spans.size());

    // 接続数を数える。双方向の接続はそれぞれを一件として数える。
    auto links = std::size_t{};

    // 東と南の隣接セルを調べる。逆方向も同時に登録するため、同じ組を二度調べずに済む。
    for (auto z = std::size_t{}; z < result.depth; ++z)
    {
        for (auto x = std::size_t{}; x < result.width; ++x)
        {
            // セル座標 (x, z) を行優先の添字に変換する。
            const auto cell = z * result.width + x;
            // 東隣がある場合だけ接続を調べる。
            if (x + 1 < result.width)
            {
                ConnectColumns(result, cell, cell + 1, Direction::East, Direction::West, links);
            }
            // 南隣がある場合だけ接続を調べる。
            if (z + 1 < result.depth)
            {
                ConnectColumns(result, cell, cell + result.width, Direction::South, Direction::North, links);
            }
        }
    }
    // 2×2のセル範囲ごとに対角の組を調べる。通り抜けを防ぐ条件は ConnectDiagonal で確認する。
    for (auto z = std::size_t{}; z + 1 < result.depth; ++z)
    {
        for (auto x = std::size_t{}; x + 1 < result.width; ++x)
        {
            const auto cell = z * result.width + x;
            // 左上と右下、右上と左下の二つの組を調べる。
            ConnectDiagonal(result, cell, cell + result.width + 1, Direction::East, Direction::South,
                            Direction::SouthEast, Direction::NorthWest, links);
            ConnectDiagonal(result, cell + 1, cell + result.width, Direction::West, Direction::South,
                            Direction::SouthWest, Direction::NorthEast, links);
        }
    }
    // 面と隣接関係の構築が完了した結果を返す。
    return result;
}
}
