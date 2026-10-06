# 取り込み元

- 元: https://github.com/mjmjm0101/zmk-input-processor-scroll-inertia
- コミット: `f7dadefee453d555fe066d13a3de3bb60739b45e`（2026-06-21）
- ライセンス: MIT（`LICENSE`、著作権表示はそのまま）
- 取り込み理由: 実行中に設定を変えるため（D15）。このコミットは改変前の複製で、変更は次のコミット以降に分けて入れる。
- 変更点の確認: `git log -p -- firmware/zmk-input-processor-scroll-inertia`

## Toucan Studio での変更（最小限）

- 設定構造体を `static const` からRAMに置き、`enabled` を追加。
- `include/zmk_scroll_inertia/runtime.h` の `scroll_inertia_apply_runtime()` で、`enabled`・`start`・`decay-*`（3つ同じ値）・`scale`/`scale-div` を実行中に変える。
- `scroll_inertia_reset()` で状態を捨てる（レイヤー別設定の切替時に使う）。
- `enabled=0` のときは慣性を始めず、入力はそのまま通す。慣性中に無効化したら止める。
- 付属のホスト用テスト（`tests/`）は変更後も成功（124/124）。
