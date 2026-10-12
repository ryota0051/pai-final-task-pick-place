// node --test test/js/gamepad.test.js  （ブラウザ UI のゲームパッド入力の解釈）
const test = require('node:test');
const assert = require('node:assert');
const { GamepadReader } = require('../../web/gamepad.js');

const pad = (axes, buttons = [], mapping = 'standard', id = 'pad') =>
  ({ id, mapping, axes, buttons: buttons.map((p) => ({ pressed: p })) });

test('静止時に端の値を返す軸は、最初の値を基準にして 0 とみなす', () => {
  const r = new GamepadReader(0.15, true);
  const v = r.read(pad([0, 0, 0, 1]));       // 右スティック Y が静止で +1（ドリフト・トリガー型）
  assert.deepStrictEqual([v.vx, v.vy, v.vz], [0, 0, 0]);
  const v2 = r.read(pad([0, 0, 0, 1]));
  assert.strictEqual(v2.vz, 0);
});

test('基準からの変化は操作として扱う（右スティックを上へ倒すと z+）', () => {
  const r = new GamepadReader(0.15, true);
  r.read(pad([0, 0, 0, 0]));
  const v = r.read(pad([0, 0, 0, -0.8]));
  assert.ok(Math.abs(v.vz - 0.8) < 1e-9);
});

test('標準の配置でない機器は無視する', () => {
  const r = new GamepadReader(0.15, true);
  const v = r.read(pad([0, 0, 0, 1], [], '', 'joystick'));
  assert.strictEqual(v, null);
  assert.match(r.status, /無視/);
});

test('デッドゾーン内は 0、ボタンは押した瞬間だけ', () => {
  const r = new GamepadReader(0.15, true);
  r.read(pad([0, 0, 0, 0], [false, false]));
  const v = r.read(pad([0.1, 0, 0, 0], [true, false]));
  assert.strictEqual(v.vy, 0);
  assert.strictEqual(v.close, true);
  const v2 = r.read(pad([0, 0, 0, 0], [true, false]));
  assert.strictEqual(v2.close, false);
});

test('ゲームパッドがなければ null', () => {
  assert.strictEqual(new GamepadReader(0.15).read(null), null);
});

test('無効（既定）のときは検出しても使わない', () => {
  const r = new GamepadReader(0.15);            // 既定は無効
  assert.strictEqual(r.read(pad([0, 0, 0, 0.5])), null);
  assert.match(r.status, /無効/);
});

test('有効にした瞬間の値を静止時の値として取り直す', () => {
  const r = new GamepadReader(0.15);
  r.read(pad([0, 0, 0, 0.5]));
  r.setEnabled(true);
  const v = r.read(pad([0, 0, 0, 0.55]));         // 有効化直後の値が基準
  assert.strictEqual(v.vz, 0);
  const v2 = r.read(pad([0, 0, 0, -0.3]));        // 基準 0.55 から -0.85
  assert.ok(Math.abs(v2.vz - 0.85) < 1e-9);
});
