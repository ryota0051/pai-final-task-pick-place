// ゲームパッド入力の解釈（ブラウザ UI 用。node --test test/js でテスト）
//
// - ブラウザが「標準の配置（mapping === 'standard'）」と判定したゲームパッドだけを使う。
//   それ以外（ハンドル・ジョイスティック・トリガーを軸として出す機器など）は軸の意味が違うので無視する。
// - 最初に見えたときの軸の値を「静止時の値」とし、そこからの変化だけを操作として扱う。
//   静止時に軸が ±1 を返す機器で、操作していないのに腕が動き続ける問題への対策（2026-10-12）。
(function (root) {
  class GamepadReader {
    constructor(deadzone) {
      this.deadzone = deadzone;
      this.rest = null;      // 静止時の軸の値
      this.id = null;
      this.prev = {};
      this.status = 'なし';
    }

    // gp: navigator.getGamepads() の 1 台。戻り値: {vx, vy, vz, vp, open, close} または null（使わない）
    read(gp) {
      if (!gp) {
        this.status = 'なし';
        return null;
      }
      if (gp.mapping !== 'standard') {
        this.status = `無視（標準の配置ではない: ${gp.id}）`;
        return null;
      }
      if (this.id !== gp.id || !this.rest) {
        this.id = gp.id;
        this.rest = [...gp.axes];
        this.prev = {};
      }
      this.status = `使用中（${gp.id}）`;
      const ax = (i) => {
        const v = (gp.axes[i] ?? 0) - (this.rest[i] ?? 0);
        return Math.abs(v) < this.deadzone ? 0 : v;
      };
      const neg = (v) => (v === 0 ? 0 : -v);   // -0 を作らない
      const btn = (i) => !!gp.buttons[i]?.pressed;
      const a = btn(0), b = btn(1);
      const out = {
        vx: neg(ax(1)), vy: neg(ax(0)), vz: neg(ax(3)),
        vp: (btn(5) ? 1 : 0) - (btn(4) ? 1 : 0),
        close: a && !this.prev.a,
        open: b && !this.prev.b,
      };
      this.prev = { a, b };
      return out;
    }
  }

  if (typeof module !== 'undefined' && module.exports) module.exports = { GamepadReader };
  else root.GamepadReader = GamepadReader;
})(this);
