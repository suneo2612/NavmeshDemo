/**
 * @file terrain.cpp
 * @brief 地形三角形の変換とタイル空間の分割を実装する。
 * @date 2026-10-31
 * @par 最終更新日 2026-10-31
 */
#include "terrain.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace demo::navmesh
{
namespace
{
constexpr auto kMaxTilesPerAxis = std::size_t{1024};

/**
 * @brief ノード行列を位置に適用し、描画と同じ順序でワールド変換する。
 * @param i_position メッシュ内の位置。
 * @param i_node 列優先のノード変換。
 * @param i_world Entity のワールド変換。
 * @return ワールド空間の位置。
 */
general::vec::Vector3 TransformPosition(const std::array<float, 3>&     i_position,
                                        const std::array<float, 16>&    i_node,
                                        const general::math::Matrix4&   i_world)
{
    auto transformed = std::array<float, 3>{};

    // ノード変換後の各軸を double で計算し、float で表現できる範囲か確認する。
    for (auto axis = std::size_t{}; axis < transformed.size(); ++axis)
    {
        const auto value    =   static_cast<double>(i_node[axis]) * i_position[0] +
                                static_cast<double>(i_node[4 + axis]) * i_position[1] +
                                static_cast<double>(i_node[8 + axis]) * i_position[2] + i_node[12 + axis];
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        {
            throw std::overflow_error("地形頂点の変換結果が表現可能な範囲を超えました。");
        }
        transformed[axis] = static_cast<float>(value);
    }

    // ノード変換後の位置に Entity のワールド変換を適用する。
    return i_world.TransformPoint({transformed[0], transformed[1], transformed[2]});
}

/**
 * @brief 地形頂点の変換に使えるノード行列かを検証する。
 * @param i_transform 列優先で格納されたノード変換行列。
 * @return 全要素が有限で、射影成分を持たないアフィン変換の場合は true。
 */
bool IsValidNodeTransform(const std::array<float, 16>& i_transform)
{
    const auto hasFiniteValues = std::all_of(
        i_transform.begin(), 
        i_transform.end(),
        [](const auto i_value) 
        {
            return std::isfinite(i_value);
        }
    );
    
    // 同次座標の最終行がアフィン変換の形式で、射影成分を持たないことを確認する。
    const auto isAffine = i_transform[3] == 0.0F && i_transform[7] == 0.0F &&
        i_transform[11] == 0.0F && i_transform[15] == 1.0F;
    return hasFiniteValues && isAffine;
}

/**
 * @brief 座標範囲を等分したときの、指定番号の境界座標を求める。
 * @param i_min 範囲の最小座標。
 * @param i_max 範囲の最大座標。
 * @param i_index 求める境界の番号（0からi_countまで）。
 * @param i_count 範囲の分割数。
 * @return i_index番目の境界座標。番号0とi_countでは範囲の端点を返す。
 * @details 内部の境界座標は両端の座標から線形補間して求める。
 */
float Boundary(const float i_min, const float i_max, const std::size_t i_index, const std::size_t i_count)
{
    if (i_index == 0) { return i_min; }                     // 最小端では入力された座標をそのまま使う。
    if (i_index == i_count) { return i_max; }               // 最大端では入力された座標をそのまま使う。

    // 内部境界は double で線形補間し、計算途中の精度低下を抑えてから float に戻す。
    return static_cast<float>(static_cast<double>(i_min) +
           (static_cast<double>(i_max) - i_min) * static_cast<double>(i_index) / static_cast<double>(i_count));
}

/**
 * @brief 地形を分割したタイル格子で、指定軸の検索範囲と重なるタイルの添字区間を二分探索で求める。
 * @param i_min         地形タイル格子の、この軸方向の最小座標。
 * @param i_max         地形タイル格子の、この軸方向の最大座標。
 * @param i_count       この軸方向のタイル数。
 * @param i_queryMin    検索対象範囲の、この軸方向の最小座標。
 * @param i_queryMax    検索対象範囲の、この軸方向の最大座標。
 * @return 重なるタイルの添字区間（開始添字以上、終了添字未満）。
 */
std::array<std::size_t, 2> OverlapRange(const float         i_min,
                                        const float         i_max,
                                        const std::size_t   i_count,
                                        const double        i_queryMin,
                                        const double        i_queryMax)
{
    auto first = std::size_t{};
    auto last = i_count;

    // [first, last)から、検索範囲の左端に接する最初のタイル添字を探す。
    while (first < last)
    {
        const auto middle   =   first + (last - first) / 2;

        // middle番目のタイルの右端が検索範囲の左端より手前なら、そのタイルは重ならない。
        if (static_cast<double>(Boundary(i_min, i_max, middle + 1, i_count)) < i_queryMin)
        {
            // 重ならないタイルを候補から外し、次の添字から右側を探す。
            first = middle + 1;
        }
        else
        {
            // 接触または重なる可能性があるため、middleを含む左側を引き続き探す。
            last = middle;
        }
    }

    // ここで確定したbeginを開始添字として、検索範囲の右端を越える位置を探す。
    const auto begin    =   first;
    last                =   i_count;
    while (first < last)
    {
        const auto middle   =   first + (last - first) / 2;

        // middle番目のタイルの左端が検索範囲の右端以下なら、そのタイルは接触または重なる。
        if (static_cast<double>(Boundary(i_min, i_max, middle, i_count)) <= i_queryMax)
        {
            // このタイルを含めるため、終了添字の候補を右へ進める。
            first = middle + 1;
        }
        else
        {
            // このタイルより右は重ならないため、終了添字の候補をmiddleまで戻す。
            last = middle;
        }
    }

    // begin以上first未満が、検索範囲と接触または重なるタイルの添字区間になる。
    return {begin, first};
}
}

/**
 * @brief 入力モデルを描画と同じノード・ワールド変換でワールド空間に移す。
 * @param i_model 入力モデル。
 * @param i_world フィールド Entity のワールド変換。
 * @return 三角形とその範囲。
 * @throws std::invalid_argument 三角形がない、インデックスが不正、または入力が非有限の場合。
 * @throws std::overflow_error 変換結果が表現可能な範囲を超えた場合。
 */
Terrain MakeTerrain(const loader::ModelData& i_model, const general::math::Matrix4& i_world)
{
    auto result     =   Terrain{};      // 変換済み三角形と地形全体の範囲を保持する。
    auto hasVertex  =   false;          // 使用頂点を使って範囲を初期化済みかを表す。

    // メッシュごとに変換行列と三角形のインデックスを検証し、頂点をワールド空間へ変換する。
    for (const auto& mesh : i_model.meshes)
    {
        // ノード変換は位置のアフィン変換として使うため、非有限値と射影変換を拒否する。
        if (!IsValidNodeTransform(mesh.node_transform))
        {
            throw std::invalid_argument("地形のノード変換が不正です。");
        }

        // インデックス数が三の倍数でないメッシュは拒否する。
        if (mesh.indices.size() % 3 != 0)
        {
            throw std::invalid_argument("地形のインデックスは三角形単位で指定してください。");
        }

        // インデックスを三つずつ読み、三角形と使用頂点の全体範囲を同時に構築する。
        for (auto index = std::size_t{}; index < mesh.indices.size(); index += 3)
        {
            // 変換後の三頂点を格納し、この三角形を組み立てる。
            auto triangle = std::array<general::vec::Vector3, 3>{};
            for (auto corner = std::size_t{}; corner < triangle.size(); ++corner)
            {
                // 頂点の参照先が有効で、位置の各成分が有限値であることを確認する。
                const auto vertexIndex = mesh.indices[index + corner];
                if (vertexIndex >= mesh.positions.size()) { throw std::invalid_argument("地形の頂点インデックスが範囲外です。"); }
                const auto& position = mesh.positions[vertexIndex];
                if (!std::all_of(position.begin(), position.end(), [](const auto i_value) { return std::isfinite(i_value); }))
                {
                    throw std::invalid_argument("地形の頂点は有限値で指定してください。");
                }

                // メッシュのノード変換とワールド変換を順に適用し、ワールド座標に移す。
                const auto point    =   TransformPosition(position, mesh.node_transform, i_world);
                triangle[corner]    =   point;

                // 使用される頂点だけを範囲に含め、未参照頂点の影響を避ける。
                if (!hasVertex)
                {
                    result.bounds = {point, point};
                    hasVertex = true;
                }
                else
                {
                    // 各軸の最小値と最大値を広げ、地形全体のAABBを更新する。
                    for (auto axis = std::size_t{}; axis < 3; ++axis)
                    {
                        result.bounds.minimum[axis] = std::min(result.bounds.minimum[axis], point[axis]);
                        result.bounds.maximum[axis] = std::max(result.bounds.maximum[axis], point[axis]);
                    }
                }
            }
            result.triangles.push_back(triangle);
        }
    }
    // Navmesh の入力として扱えるよう、空のモデルや三角形を持たないモデルを拒否する。
    if (!hasVertex) { throw std::invalid_argument("地形に三角形がありません。"); }
    return result;
}

/**
 * @brief 格子を作り、範囲とタイル数を検証する。
 * @param i_bounds 地形全体の範囲。
 * @param i_columns X 方向のタイル数。
 * @param i_rows Z 方向のタイル数。
 * @throws std::invalid_argument 範囲またはタイル数が不正な場合。
 */
TileGrid::TileGrid(const Bounds& i_bounds,
                   const std::size_t i_columns,
                   const std::size_t i_rows)
    : m_stBounds(i_bounds), m_instColumns(i_columns), m_instRows(i_rows)
{
    // 地形範囲の各軸が有限で、最小値が最大値を超えないことを確認する。
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        // 地形全体のAABBについて、各軸の最小値と最大値を検証する。
        if (!std::isfinite(i_bounds.minimum[axis]) ||
            !std::isfinite(i_bounds.maximum[axis]) ||
            i_bounds.minimum[axis] > i_bounds.maximum[axis])
        {
            throw std::invalid_argument("地形の範囲が不正です。");
        }
    }

    // X/Y/Z各軸の幅が正で、X/Z方向のタイル数が許容範囲内であることを確認する。
    if (i_bounds.minimum.x == i_bounds.maximum.x ||     // X軸のサイズがゼロ
        i_bounds.minimum.z == i_bounds.maximum.z ||     // Z軸のサイズがゼロ
        i_columns == 0 ||                               // 列数がゼロ
        i_rows == 0 ||                                  // 行数がゼロ
        i_columns > kMaxTilesPerAxis ||                 // 列数が最大値を超える
        i_rows > kMaxTilesPerAxis)                      // 行数が最大値を超える
    {
        throw std::invalid_argument("タイルの範囲または分割数が不正です。");
    }

    // float 座標で隣接する境界が同じ値に丸められる格子は使用できない。
    for (auto index = std::size_t{}; index < i_columns; ++index)
    {
        // X方向で隣り合う境界が異なり、各タイルに正の幅があることを確認する。
        if (Boundary(i_bounds.minimum.x, i_bounds.maximum.x, index, i_columns) >=
            Boundary(i_bounds.minimum.x, i_bounds.maximum.x, index + 1, i_columns))
        {
            throw std::invalid_argument("X 方向のタイル幅が小さすぎます。");
        }
    }
    for (auto index = std::size_t{}; index < i_rows; ++index)
    {
        // Z方向で隣り合う境界が異なり、各タイルに正の幅があることを確認する。
        if (Boundary(i_bounds.minimum.z, i_bounds.maximum.z, index, i_rows) >=
            Boundary(i_bounds.minimum.z, i_bounds.maximum.z, index + 1, i_rows))
        {
            throw std::invalid_argument("Z 方向のタイル幅が小さすぎます。");
        }
    }
}

/**
 * @brief 指定したタイルの範囲を返す。
 * @param i_column X 方向の添字。
 * @param i_row Z 方向の添字。
 * @return 地形の Y 範囲を含むタイルの範囲。
 * @throws std::out_of_range 添字が格子の範囲外の場合。
 */
Bounds TileGrid::TileBounds(const std::size_t i_column, const std::size_t i_row) const
{
    // 不正な添字を境界計算に使わず、呼び出し側に明示して知らせる。
    if (i_column >= m_instColumns || i_row >= m_instRows)
    {
        throw std::out_of_range("タイルの添字が範囲外です。");
    }

    // XZ は隣接タイルと共有する境界から求め、Y は地形全体の範囲を使う。
    return {{Boundary(m_stBounds.minimum.x, m_stBounds.maximum.x, i_column, m_instColumns),
             m_stBounds.minimum.y,
             Boundary(m_stBounds.minimum.z, m_stBounds.maximum.z, i_row, m_instRows)},
            {Boundary(m_stBounds.minimum.x, m_stBounds.maximum.x, i_column + 1, m_instColumns),
             m_stBounds.maximum.y,
             Boundary(m_stBounds.minimum.z, m_stBounds.maximum.z, i_row + 1, m_instRows)}};
}

/**
 * @brief 指定範囲に重なるタイルを Z 行・X 列の順に返す。
 * @param i_bounds 重なりを調べるワールド空間の範囲。
 * @param i_margin 境界処理を考慮して XZ に広げる距離。
 * @return 範囲と境界を含めて接触するタイルの座標。
 * @throws std::invalid_argument 範囲または余白が不正な場合。
 */
std::vector<TileCoordinate> TileGrid::OverlappingTiles(const Bounds& i_bounds, const float i_margin) const
{
    // 不正な範囲を空の結果として扱うと、必要な再構築が省略されるため拒否する。
    if (!std::isfinite(i_margin) || i_margin < 0.0F)
    {
        throw std::invalid_argument("タイル検索の余白は有限の非負数で指定してください。");
    }
    for (auto axis = std::size_t{}; axis < 3; ++axis)
    {
        if (!std::isfinite(i_bounds.minimum[axis]) || !std::isfinite(i_bounds.maximum[axis]) ||
            i_bounds.minimum[axis] > i_bounds.maximum[axis])
        {
            throw std::invalid_argument("タイル検索の範囲が不正です。");
        }
    }

    // Y 方向で地形と交差しない範囲は、X/Z の探索を行わず空の結果を返す。
    auto result = std::vector<TileCoordinate>{};
    if (i_bounds.maximum.y < m_stBounds.minimum.y || i_bounds.minimum.y > m_stBounds.maximum.y)
    {
        return result;
    }
    
    // X軸方向のタイルの重なりを境界処理も考慮した範囲で求める
    const auto columns = OverlapRange(
        m_stBounds.minimum.x,
        m_stBounds.maximum.x,
        m_instColumns,
        static_cast<double>(i_bounds.minimum.x) - i_margin,
        static_cast<double>(i_bounds.maximum.x) + i_margin);

    // Z軸方向のタイルの重なりを境界処理も考慮した範囲で求める
    const auto rows = OverlapRange(
        m_stBounds.minimum.z,
        m_stBounds.maximum.z,
        m_instRows,
        static_cast<double>(i_bounds.minimum.z) - i_margin,
        static_cast<double>(i_bounds.maximum.z) + i_margin);

    // 最終的な結果を格納するメモリ領域だけ先行して確保しておく
    result.reserve((columns[1] - columns[0]) * (rows[1] - rows[0]));
    
    // 探索区間内のタイルだけを行優先で追加し、結果の順序を安定させる。
    for (auto row = rows[0]; row < rows[1]; ++row)
    {
        for (auto column = columns[0]; column < columns[1]; ++column)
        {
            result.push_back({column, row});
        }
    }
    return result;
}
}
