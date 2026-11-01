/**
 * @file main.cpp
 * @brief コマンドライン引数を処理してアプリケーションを起動する。
 * @date 2026-10-28
 * @par 最終更新日 2026-11-01
 */
#include "demo/application/application.h"
#include <iostream>
#include <stdexcept>
#include <string_view>

int main(const int i_argc, char** i_argv)
{
    try
    {
        // ヘルプと不正な引数は描画環境を初期化する前に処理する。
        if (i_argc == 2 && std::string_view(i_argv[1]) == "--help")
        {
            std::cout << "Usage: navmesh_demo [--help]\n"
                         "Requires an OpenGL 4.3 GPU and a desktop session.\n";
            return 0;
        }
        if (i_argc != 1) { throw std::runtime_error("引数が不正です。--help を参照してください。"); }
        return demo::application::Run();
    }
    catch (const std::exception& i_error)
    {
        std::cerr << "エラー: " << i_error.what() << '\n';
        return 1;
    }
}
