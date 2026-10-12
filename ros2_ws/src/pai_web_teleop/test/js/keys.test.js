// node --test test/js/keys.test.js  （ブラウザ UI のキー割り当て）
const test = require('node:test');
const assert = require('node:assert');
const { KeyBindings } = require('../../web/keys.js');

const kb = new KeyBindings({ 'x+': ['w', 'ArrowUp'], 'x-': ['s', 'ArrowDown'], 'y+': 'a', record: ' ' });

test('1 つの動作に複数のキー（矢印キーを含む）を割り当てられる', () => {
  assert.strictEqual(kb.actionOf('w'), 'x+');
  assert.strictEqual(kb.actionOf('ArrowUp'), 'x+');
  assert.strictEqual(kb.actionOf('W'), 'x+');          // 大文字小文字は区別しない
  assert.strictEqual(kb.actionOf('a'), 'y+');          // 文字列 1 つでもよい
  assert.strictEqual(kb.actionOf('q'), null);
});

test('押している動作から軸の値を作る', () => {
  const held = new Set(['x+']);
  assert.strictEqual(kb.axis(held, 'x+', 'x-'), 1);
  held.add('x-');
  assert.strictEqual(kb.axis(held, 'x+', 'x-'), 0);
});

test('割り当てたキーはページのスクロールなどを止める対象', () => {
  assert.strictEqual(kb.isBound('ArrowDown'), true);
  assert.strictEqual(kb.isBound('ArrowLeft'), false);
});
