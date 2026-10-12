// キー割り当ての解釈（ブラウザ UI 用。node --test test/js/keys.test.js でテスト）
// config.json の keys は「動作名 → キー」。キーは 1 つ（文字列）でも複数（配列。例: ["w", "ArrowUp"]）でもよい。
// キー名は KeyboardEvent.key（"w", "ArrowUp", " " など）で、大文字小文字は区別しない。
(function (root) {
  class KeyBindings {
    constructor(keys) {
      this.byKey = new Map();
      for (const [action, k] of Object.entries(keys)) {
        for (const key of Array.isArray(k) ? k : [k]) this.byKey.set(key.toLowerCase(), action);
      }
    }

    actionOf(key) {
      return this.byKey.get(key.toLowerCase()) ?? null;
    }

    isBound(key) {
      return this.actionOf(key) !== null;
    }

    // held: 押している動作名の集合
    axis(held, pos, neg) {
      return (held.has(pos) ? 1 : 0) - (held.has(neg) ? 1 : 0);
    }
  }

  if (typeof module !== 'undefined' && module.exports) module.exports = { KeyBindings };
  else root.KeyBindings = KeyBindings;
})(this);
