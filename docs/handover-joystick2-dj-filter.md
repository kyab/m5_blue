# 引き継ぎ: Joystick2 DJ フィルター（PR #32）

別チャットからこのブランチ／作業を続けるためのメモ。最終更新: 2026-08-10。

## 参照先

| 項目 | 値 |
|------|--------|
| ブランチ | `cursor/joystick2-dj-filter-80d2`（`origin` 追従） |
| ベース | `main` |
| PR | https://github.com/kyab/m5_blue/pull/32 |
| tip コミット | `13af7f7` — `tune: map Joystick2 Y full-scale to ±4096 for \|v\|=1` |
| 作業ツリー | クリーン（EMA / Q 実験はリバート済み・未コミット） |
| 既定 env | `m5stack-core2` |

## ゴール（完了 vs 未解決）

**完了**

- PORT.A の Joystick2 で DJ フィルター操作（Z 押し + Y）。上=LPF / 中=bypass / 下=HPF。
- U005 は `#ifdef` で残存。I2C は Joystick2=`Wire`@32/33、Module Audio=`Wire1`@21/22（`Wire1` に Joystick2 を載せない）。

**未解決**

- `|v| > 0.5` 付近で「チュワ」音。Y EMA と Q 下げは試して効果なし→リバート済み。次は DSP/トポロジ側を疑う。

## エージェント向けメモ

- チャット: 日本語。コメント: 英語。PR: 英語。
- 依頼なくコミットしない。Going-Zero 参照は GitHub から fetch（`AGENTS.md`）。
