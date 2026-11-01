/**
 * @file application.h
 * @brief デモアプリケーションの実行 API を定義する。
 * @date 2026-10-31
 * @par 最終更新日 2026-11-01
 */
#pragma once
#ifndef NAVMESH_DEMO_DEMO_APPLICATION_APPLICATION_H
#define NAVMESH_DEMO_DEMO_APPLICATION_APPLICATION_H

namespace demo::application
{
/**
 * @brief ウィンドウと描画環境を初期化してアプリケーションを実行する。
 * @return アプリケーションの終了コード。
 * @throws std::runtime_error 初期化に失敗した場合。
 */
int Run();
}
#endif
