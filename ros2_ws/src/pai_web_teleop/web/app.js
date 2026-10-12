/* CRANE+ V2 Web Teleop: roslibjs で rosbridge に接続し、手先速度/目標・グリッパ・録画を操作する */
(async () => {
  const cfg = await (await fetch('config.json', { cache: 'no-store' })).json();   // 設定の変更をすぐ反映
  const params = new URLSearchParams(location.search);
  const wsUrl = params.get('ws') || `ws://${location.hostname || 'localhost'}:${cfg.rosbridge_port}`;
  const $ = (id) => document.getElementById(id);
  const log = (msg) => { $('log').textContent = `${new Date().toLocaleTimeString()} ${msg}`; };

  const ros = new ROSLIB.Ros({ url: wsUrl });
  ros.on('connection', () => { $('conn').textContent = 'connected'; $('conn').className = 'badge on'; });
  ros.on('close', () => { $('conn').textContent = 'disconnected'; $('conn').className = 'badge off'; setTimeout(() => ros.connect(wsUrl), 2000); });
  ros.on('error', () => {});

  const topic = (name, type) => new ROSLIB.Topic({ ros, name, messageType: type });
  const velPub = topic(cfg.topics.ee_velocity, 'geometry_msgs/Twist');
  const targetPub = topic(cfg.topics.ee_target, 'geometry_msgs/PoseStamped');
  const gripperPub = topic(cfg.topics.gripper, 'std_msgs/Float64');
  const callTrigger = (name) => new Promise((resolve) => {
    const srv = new ROSLIB.Service({ ros, name, serviceType: 'std_srvs/Trigger' });
    srv.callService({}, (r) => { log(`${name}: ${r.message}`); resolve(r); },
      (e) => { log(`${name}: ${e}`); resolve(null); });
  });
  const callSetBool = (name, data) => {
    const srv = new ROSLIB.Service({ ros, name, serviceType: 'std_srvs/SetBool' });
    srv.callService({ data }, (r) => log(`${name}: ${r.message}`), (e) => log(`${name}: ${e}`));
  };

  // ---- 手先目標スライダー（ドラッグ中は絶対目標を送る。ee_state で同期）----
  const axes = ['x', 'y', 'z', 'pitch'];
  const sliders = {};
  let dragging = false;
  for (const a of axes) {
    const [lo, hi] = cfg.slider_ranges[a];
    const row = document.createElement('div');
    row.className = 'slider';
    row.innerHTML = `<label>${a}</label><input type="range" min="${lo}" max="${hi}" step="0.001"><span>-</span>`;
    $('sliders').appendChild(row);
    const input = row.querySelector('input');
    sliders[a] = { input, label: row.querySelector('span') };
    input.addEventListener('pointerdown', () => { dragging = true; });
    window.addEventListener('pointerup', () => { dragging = false; });
    input.addEventListener('input', sendTarget);
  }
  function sendTarget() {
    const v = Object.fromEntries(axes.map((a) => [a, parseFloat(sliders[a].input.value)]));
    targetPub.publish({
      header: { frame_id: 'crane_plus_base' },
      pose: { position: { x: v.x, y: v.y, z: v.z },
        orientation: { x: 0, y: Math.sin(v.pitch / 2), z: 0, w: Math.cos(v.pitch / 2) } },
    });
  }
  topic(cfg.topics.ee_state, 'geometry_msgs/PoseStamped').subscribe((m) => {
    const p = m.pose.position, o = m.pose.orientation;
    const vals = { x: p.x, y: p.y, z: p.z, pitch: 2 * Math.atan2(o.y, o.w) };
    for (const a of axes) {
      if (!dragging) sliders[a].input.value = vals[a];
      sliders[a].label.textContent = vals[a].toFixed(3);
    }
  });

  // ---- キーボード / ゲームパッドによる速度指令 ----
  const held = new Set();
  const keyOf = (name) => cfg.keys[name];
  window.addEventListener('keydown', (e) => {
    const k = e.key.toLowerCase();
    if (e.target.tagName === 'INPUT') return;
    if (k === keyOf('record')) { e.preventDefault(); if (!e.repeat) toggleRecord(); return; }
    if (k === keyOf('open')) setGripper(0);
    else if (k === keyOf('close')) setGripper(1);
    else if (k === keyOf('home')) callTrigger(cfg.services.home);
    held.add(k);
  });
  window.addEventListener('keyup', (e) => held.delete(e.key.toLowerCase()));
  window.addEventListener('blur', () => held.clear());

  const axis = (pos, neg) => (held.has(keyOf(pos)) ? 1 : 0) - (held.has(keyOf(neg)) ? 1 : 0);
  // ゲームパッドは標準の配置のものだけを使い、静止時の値からの変化を操作とする（web/gamepad.js）
  const gpReader = new GamepadReader(cfg.gamepad_deadzone, cfg.gamepad_enabled);
  $('gamepad-enable').checked = cfg.gamepad_enabled;
  $('gamepad-enable').onchange = (e) => gpReader.setEnabled(e.target.checked);
  setInterval(() => {
    let vx = axis('x+', 'x-'), vy = axis('y+', 'y-'), vz = axis('z+', 'z-'), vp = axis('pitch+', 'pitch-');
    const gp = navigator.getGamepads ? [...navigator.getGamepads()].find((g) => g) : null;
    const g = gpReader.read(gp);
    $('gamepad').textContent = gpReader.status;
    if (g) {
      vx += g.vx; vy += g.vy; vz += g.vz; vp += g.vp;
      if (g.close) setGripper(1);
      if (g.open) setGripper(0);
    }
    if (vx || vy || vz || vp) {
      velPub.publish({
        linear: { x: vx * cfg.linear_speed, y: vy * cfg.linear_speed, z: vz * cfg.linear_speed },
        angular: { x: 0, y: vp * cfg.pitch_speed, z: 0 },
      });
    }
  }, 1000 / cfg.command_rate_hz);

  function setGripper(v) { gripperPub.publish({ data: v }); }
  $('btn-home').onclick = () => callTrigger(cfg.services.home);
  $('btn-open').onclick = () => setGripper(0);
  $('btn-close').onclick = () => setGripper(1);

  // ---- 録画・タスク ----
  let recording = false;
  async function toggleRecord() {
    await callTrigger(recording ? cfg.services.record_stop : cfg.services.record_start);
  }
  $('btn-record').onclick = toggleRecord;
  $('btn-discard').onclick = () => callTrigger(cfg.services.discard_last);
  $('btn-reset').onclick = async () => {
    await callTrigger(cfg.services.task_reset);
    await callTrigger(cfg.services.home);
    setGripper(0);
  };

  const subJson = (name, cb) => topic(name, 'std_msgs/String').subscribe((m) => cb(JSON.parse(m.data)));
  subJson(cfg.topics.teleop_status, (s) => {
    $('ik').textContent = s.ik_ok ? (s.pitch_relaxed ? 'OK（pitch 緩和中）' : 'OK') : '解なし（直前姿勢を維持）';
    $('ik').className = s.ik_ok ? 'ok' : 'ng';
    $('joints').textContent = s.joints.map((v) => v.toFixed(2)).join(', ');
    $('gripper').textContent = s.gripper >= 0.5 ? '閉' : '開';
    $('teleop-enabled').textContent = s.enabled ? '有効' : '無効（方策実行中など）';
  });
  subJson(cfg.topics.recorder_status, (s) => {
    recording = s.recording;
    $('rec-state').textContent = s.recording ? `録画中 ${s.episode} (${s.elapsed}s)` : '停止';
    $('rec-next').textContent = s.next_episode;
    $('rec-last').textContent = s.last_episode ?? '-';
    $('btn-record').textContent = s.recording ? '録画停止 (Space)' : '録画開始 (Space)';
    $('btn-record').className = s.recording ? 'recording' : 'primary';
  });
  topic(cfg.topics.object_pose, 'geometry_msgs/PoseStamped').subscribe((m) => {
    const p = m.pose.position;
    $('obj').textContent = `${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)}`;
  });
  subJson(cfg.topics.task_status, (s) => {
    $('task').textContent = s.success ? '成功' : (s.grasped ? '把持中' : '未達');
    $('task').className = s.success ? 'ok' : '';
  });

  // ---- 方策 ----
  $('btn-policy-on').onclick = () => { callSetBool(cfg.services.teleop_enable, false); callSetBool(cfg.services.policy_enable, true); };
  $('btn-policy-off').onclick = () => { callSetBool(cfg.services.policy_enable, false); callSetBool(cfg.services.teleop_enable, true); };
  subJson(cfg.topics.policy_status, (s) => {
    const pred = s.predicted ? s.predicted.map((v) => v.toFixed(2)).join(', ') : '-';
    $('policy').textContent = `${s.enabled ? '実行中' : '停止'} / ${s.latency_ms ?? '-'} ms / pred: ${pred}`;
  });
})();
