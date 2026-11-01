# navmesh_demo

C++20 / OpenGL 4.3 による、静的 FBX モデルの表示とタイル型 Navmesh 生成のデモです。
モデルから歩行可能な領域を生成し、ボクセル、領域、ポリゴンや障害物の影響を
ImGui のデバッグ画面で確認できます。

**開発中:** Navmesh のポリゴンと接続情報は生成しますが、経路探索は未実装です。

## はじめに

1. [必要環境](#必要環境)を準備し、[ビルド](#ビルドと-visual-studio-ソリューション)します。
2. `solutions\model\stage` に静的 FBX ファイルを置き、アプリケーションを起動します。
3. **Stage** ウィンドウの **Refresh** を押し、**Model** からステージを選択します。
4. **デバッグ機能** で「地形ボクセル+安全距離確認」を有効にし、
   「Navmesh ポリゴン」などの表示を切り替えます。

カメラの移動方法は[ステージ表示と操作](#ステージ表示と操作)、
生成設定や障害物操作の詳細は[タイル型 Navmesh](docs/navmesh.md)を参照してください。

## 必要環境

- Windows 10 / 11 (x64)
- Visual Studio 2026 または 2022
  - 「C++ によるデスクトップ開発」
  - MSVC x64/x86 ビルドツール、Windows SDK
  - 「Windows 用 C++ CMake ツール」
- CMake 3.25 以上（VS 2026 のソリューション生成には 4.2 以上）
- OpenGL 4.3 以上に対応する GPU とメーカー製ドライバー
- 初回構成時のインターネット接続（依存ライブラリ取得用）
- Git（Visual Studio 同梱版でも可）、GitHub アカウント（公開時のみ）

## ビルドと Visual Studio ソリューション

PowerShell でプロジェクトのルートから実行します。
スクリプトは PATH 上の CMake、または Visual Studio 同梱 CMake を使用します。

```powershell
.\scripts\build.ps1 -Preset vs2026 -Configuration Debug
.\scripts\build.ps1 -Preset vs2026 -Configuration Release
```

PowerShell の実行ポリシーによりスクリプトが拒否された場合は、組織のポリシーに従い、
下記の CMake コマンドを Developer PowerShell for VS から直接実行してください。
VS 2022 では `vs2026` を `vs2022` に置き換えます。

```powershell
cmake --preset vs2026
cmake --build --preset vs2026-debug --parallel
```

CMake が PATH にない通常の PowerShell では、Visual Studio 2026 同梱版を
次のように直接指定できます。実行ポリシーの変更は不要です。

```powershell
$tools = 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin'
& "$tools\cmake.exe" --preset vs2026
& "$tools\cmake.exe" --build --preset vs2026-debug --parallel
```

生成される `solutions\vs2026\navmesh_demo.slnx` を Visual Studio 2026 で開きます。
VS 2022 では `solutions\vs2022\navmesh_demo.sln` になります。
`navmesh_demo` がスタートアッププロジェクトです。Debug / Release と x64 を選択して F5 で実行できます。
生成ファイルは Git 管理せず、CMake から再生成します。

```powershell
.\solutions\vs2026\Debug\navmesh_demo.exe
.\solutions\vs2026\Release\navmesh_demo.exe
```

## 構成

| 要素 | 実装 |
|---|---|
| 言語 / ビルド | C++20 / CMake Presets |
| ウィンドウ | GLFW 3.4 |
| 描画 | OpenGL 4.3 Core / 静的モデル、方向光、環境光 |
| OpenGL 関数ロード | GLEW 2.2.0（glew-cmake 配布、静的リンク） |
| デバッグ UI | Dear ImGui v1.91.9b / GLFW・OpenGL3 バックエンド |
| モデル読み込み | ufbx 0.23.1 / stb_image |
| バージョン管理 | Git / GitHub 対応 |

依存関係は FetchContent で `solutions\<preset>\_deps` に取得し、バージョンと
SHA-256 を固定しています。Python、vcpkg、手動のライブラリ配置は不要です。
依存元のライセンスは取得先の LICENSE / COPYING を参照してください。

- `solutions\src\main.cpp`: コマンドライン引数の処理とアプリケーションの起動
- `solutions\src\demo\application`: 初期化、ステージ選択、ImGui
- `solutions\src\demo\navmesh`: Navmesh の生成とデバッグ表示
- `solutions\src\demo\ecs` / `solutions\src\demo\component`: Entity、Component、Transform
- `solutions\src\render`: カメラ、共有 GPU モデル資源、描画
- `solutions\model\stage`: ステージとして選択する FBX の配置先
- `shaders\model.vert` / `shaders\model.frag`: モデルの変換と照明
- `shaders\navmesh_bounds.*` / `shaders\navmesh_voxel.*`: デバッグ表示用シェーダー
- `cmake\shader_sources.hpp.in`: GLSL の実行ファイルへの埋め込み

シェーダー変更時は再ビルドしてください。CMake が変更を検出して埋め込みを再生成します。
実行時の作業ディレクトリに依存せず、シェーダーファイルのコピーも不要です。

## 詳細資料

- [タイル型 Navmesh](docs/navmesh.md): 生成設定、デバッグ表示、障害物操作と実装の詳細
- [開発者向け情報](docs/development.md): FBX 読み込み、3D 姿勢、ECS の利用方法

## ステージ表示と操作

`solutions\model\stage` 以下に FBX を置き、起動後に **Model** から選択します。
子フォルダーも再帰的に探索し、一覧にはステージフォルダーからの相対パスを表示します。
ディレクトリのシンボリックリンクはたどりません。起動時は未選択です。追加したファイルは **Refresh**
で一覧に反映します。フォルダーが存在しない場合は作成してください。
パスは CMake 構成時のソースディレクトリに基づくため、実行時の作業ディレクトリには
依存しません。リポジトリを移動した場合は再構成・再ビルドしてください。

- **右ドラッグ**: カメラの視線を回転
- **W / S**: 視線方向に前進 / 後退
- **A / D**: カメラに対して左 / 右へ移動
- **Lighting**: 方向光と環境光の有効 / 無効
- **Light direction**: 光の進行方向。ゼロや非有限値は適用しません。
- **Esc**: 終了

カメラの初期位置は `(0, 10, 10)`、初期視線は負の Z 方向です。
原点を見たい場合は、右ドラッグで下方向に視線を動かしてください。

フィールド Entity の Transform は原点・回転なし・等倍で固定です。一般のモデル Entity は
`ModelComponent` と `LocalTransform` を組み合わせ、同期済みのワールド行列で描画できます。
複数 Entity が同じ `render::Model` を共有できます。GPU 資源は OpenGL コンテキストを
所有するスレッドで作成・使用・解放し、コンテキストより先に破棄してください。
モデル読み込みや GPU 登録が失敗した場合、エラーを表示し、従来のモデルを維持します。

三角形ごとのマテリアル、基本色・テクスチャ、粗さ・金属度を使用します。透過、
法線マップ、投影される影、アニメーションは描画しません。ゼロスケールなどで
法線変換ができない場合は環境光のみで表示します。
ローダーは法線マップも読み込むため、描画に使わない場合でも FBX が参照する画像は
読み込み可能である必要があります。

## Git / GitHub

ローカルは `git init -b main` で初期化します。ビルド成果物・依存の展開先・IDE 個人設定は除外します。
コミットや GitHub リポジトリの作成・送信は自動では行いません。
Git が PATH にない場合、Visual Studio の「Git」メニューまたは同梱の `git.exe` を使用できます。

GitHub 側に空のリポジトリを用意した後、認証済みの環境で実行してください。
公開範囲、リモート URL、ライセンスはプロジェクトの方針に応じて決定してください。

```powershell
git add .
git commit -m "Initial C++20 OpenGL compute demo"
git remote add origin https://github.com/YOUR_ACCOUNT/navmesh_demo.git
git push -u origin main
```

GitHub 認証トークンやローカル専用のパス・資格情報はソースに記録しないでください。
