'use strict';

// ---------------------------------------------------------------------------
// AGV 仓库调度中心 — Web 操作面板（Nav2 版）
// 通过 rosbridge 与系统通信：
//   订阅 /fleet/state    (std_msgs/String, JSON)        -> 渲染地图 + 车队表
//   订阅 /<ns>/plan      (nav_msgs/Path)                 -> 叠加 Nav2 全局规划路径
//   订阅 /rosout         (rcl_interfaces/msg/Log)        -> 异常日志
//   发布 /fleet/add_task (std_msgs/String, JSON)         -> 下发运输任务（取->卸）
//   发布 /<ns>/goal_pose (geometry_msgs/PoseStamped)     -> 点击地图手动导航（仅 IDLE 车）
// ---------------------------------------------------------------------------

// ---- 仓库静态几何 (与 worlds/warehouse.world 一致) ----
const WAREHOUSE = {
  half: 7.0,                                   // 内墙半边长 (14x14m)
  shelfXs: [-2, -1, 0, 1, 2],
  shelfYs: [4.5, 1.5, -1.5, -4.5],
  shelfSize: { x: 0.9, y: 0.4 },               // 开口朝 -Y，本体中心在 (x, y-0.2)
  pickup: { cx: 5.5, cy: 5.5, sx: 2.5, sy: 2.5 },
  charger: { cx: -5.5, cy: -5.5, sx: 2.0, sy: 2.0 },
};

const AGV_COLORS = ['#3da9fc', '#ffa733', '#3ddc84', '#c87cff', '#ff7ca8'];

// ---- 画布坐标变换 (世界 m <-> 像素，y 轴翻转) ----
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const SIZE = canvas.width;
const PAD = 24;
const SPAN = SIZE - 2 * PAD;
const VIEW = 7.6;                              // 视野半边长 (m)
const toX = (wx) => PAD + ((wx + VIEW) / (2 * VIEW)) * SPAN;
const toY = (wy) => PAD + ((VIEW - wy) / (2 * VIEW)) * SPAN;
const toL = (m) => (m / (2 * VIEW)) * SPAN;
// 像素 -> 世界（toX/toY 的逆，供地图点击用）
const fromX = (px) => ((px - PAD) / SPAN) * (2 * VIEW) - VIEW;
const fromY = (py) => VIEW - ((py - PAD) / SPAN) * (2 * VIEW);

function fillRectC(cx, cy, w, h, color) {
  ctx.fillStyle = color;
  ctx.fillRect(toX(cx - w / 2), toY(cy + h / 2), toL(w), toL(h));
}
function strokeRectC(cx, cy, w, h, color, dash) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.setLineDash(dash || []);
  ctx.strokeRect(toX(cx - w / 2), toY(cy + h / 2), toL(w), toL(h));
  ctx.setLineDash([]);
}

let lastZones = null;
let latestState = null;
const pathByNs = {};        // ns -> [{x,y}, ...]  最新 Nav2 全局规划
const trailByNs = {};       // ns -> [[x,y], ...]  最近经过点（尾迹）
const TRAIL_MAX = 50;
let nsIndex = {};           // ns -> 颜色下标（与车队表一致）
let selectedNs = null;      // 手动导航选中的车

function drawStatic(zones) {
  ctx.clearRect(0, 0, SIZE, SIZE);

  // 地面
  fillRectC(0, 0, 2 * WAREHOUSE.half, 2 * WAREHOUSE.half, '#11161d');
  // 外墙
  strokeRectC(0, 0, 2 * WAREHOUSE.half, 2 * WAREHOUSE.half, '#46566b');

  // 货架 (20 个)
  for (const y of WAREHOUSE.shelfYs) {
    for (const x of WAREHOUSE.shelfXs) {
      fillRectC(x, y - WAREHOUSE.shelfSize.y / 2, WAREHOUSE.shelfSize.x, WAREHOUSE.shelfSize.y, '#8a6d3b');
    }
  }

  // 取货点 / 充电站（实体托盘/充电柜的位置示意）
  const p = WAREHOUSE.pickup, c = WAREHOUSE.charger;
  fillRectC(p.cx, p.cy, p.sx, p.sy, 'rgba(61,220,132,.18)');
  strokeRectC(p.cx, p.cy, p.sx, p.sy, '#3ddc84');
  fillRectC(c.cx, c.cy, c.sx, c.sy, 'rgba(255,167,51,.18)');
  strokeRectC(c.cx, c.cy, c.sx, c.sy, '#ffa733');

  // 调度区域 (来自 /fleet/state)：语义中心 + 可达接近点(gx,gy)
  if (zones) {
    ctx.font = '11px sans-serif';
    const owners = (latestState && latestState.zone_owner) || {};
    for (const [name, z] of Object.entries(zones)) {
      const isCharger = name.startsWith('charger');
      // 被某台车预约（防撞独占）时，用该车颜色高亮描边
      const owner = owners[name];
      const ownerColor = owner ? AGV_COLORS[(nsIndex[owner] || 0) % AGV_COLORS.length] : null;
      strokeRectC(z.cx, z.cy, z.sx, z.sy, ownerColor || (isCharger ? '#ffa733' : '#7e93b0'), [4, 3]);
      ctx.fillStyle = ownerColor || '#9fb0c4';
      ctx.fillText(name + (owner ? ` ⇠${owner}` : ''), toX(z.cx - z.sx / 2) + 2, toY(z.cy + z.sy / 2) - 3);

      // 接近点：仅当与中心不同（即中心落在障碍内、AGV 实际停靠点在旁边）
      const gx = (z.gx === undefined) ? z.cx : z.gx;
      const gy = (z.gy === undefined) ? z.cy : z.gy;
      if (Math.abs(gx - z.cx) > 1e-3 || Math.abs(gy - z.cy) > 1e-3) {
        // 中心 -> 接近点 连线
        ctx.strokeStyle = 'rgba(159,176,196,.55)';
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(toX(z.cx), toY(z.cy));
        ctx.lineTo(toX(gx), toY(gy));
        ctx.stroke();
        ctx.setLineDash([]);
        // 接近点标记（空心菱形）
        const ax = toX(gx), ay = toY(gy), s = 4;
        ctx.strokeStyle = '#cdd8e6';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(ax, ay - s); ctx.lineTo(ax + s, ay);
        ctx.lineTo(ax, ay + s); ctx.lineTo(ax - s, ay);
        ctx.closePath();
        ctx.stroke();
      }
    }
  }
}

function batteryColor(b) {
  if (b > 0.5) return '#3ddc84';
  if (b > 0.2) return '#ffa733';
  return '#ff5d5d';
}

// ---- Nav2 全局规划路径 ----
function drawPaths() {
  for (const [ns, pts] of Object.entries(pathByNs)) {
    if (!pts || pts.length < 2) continue;
    const color = AGV_COLORS[(nsIndex[ns] || 0) % AGV_COLORS.length];
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.55;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(toX(pts[0].x), toY(pts[0].y));
    for (let i = 1; i < pts.length; i++) ctx.lineTo(toX(pts[i].x), toY(pts[i].y));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }
}

// ---- 尾迹 ----
function drawTrails() {
  for (const [ns, pts] of Object.entries(trailByNs)) {
    if (!pts || pts.length < 2) continue;
    const color = AGV_COLORS[(nsIndex[ns] || 0) % AGV_COLORS.length];
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.25;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(toX(pts[0][0]), toY(pts[0][1]));
    for (let i = 1; i < pts.length; i++) ctx.lineTo(toX(pts[i][0]), toY(pts[i][1]));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
}

function drawAgvs(agvs) {
  agvs.forEach((a) => {
    if (a.x === null || a.y === null) return;
    const color = AGV_COLORS[(nsIndex[a.ns] || 0) % AGV_COLORS.length];
    const px = toX(a.x), py = toY(a.y), r = 9;

    // 选中高亮环
    if (a.ns === selectedNs) {
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(px, py, r + 8, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 朝向三角
    const hx = Math.cos(-a.yaw), hy = Math.sin(-a.yaw);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(px + hx * (r + 9), py + hy * (r + 9));
    ctx.lineTo(px - hy * 5, py + hx * 5);
    ctx.lineTo(px + hy * 5, py - hx * 5);
    ctx.closePath();
    ctx.fill();

    // 载货时外圈高亮
    if (a.carrying) {
      ctx.strokeStyle = '#ffd166';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(px, py, r + 4, 0, Math.PI * 2);
      ctx.stroke();
    }

    // 本体
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fill();

    // 电量小环
    ctx.strokeStyle = batteryColor(a.battery);
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(px, py, r + 1.5, -Math.PI / 2, -Math.PI / 2 + Math.PI * 2 * Math.max(0, Math.min(1, a.battery)));
    ctx.stroke();

    // 标签
    ctx.fillStyle = '#e7eef6';
    ctx.font = 'bold 11px sans-serif';
    ctx.fillText(a.ns, px + r + 4, py + 4);
  });
}

function drawCollisions() {
  if (!latestState || !latestState.collisions) return;
  const byNs = {};
  latestState.agvs.forEach((a) => { byNs[a.ns] = a; });
  for (const c of latestState.collisions) {
    const A = byNs[c.a], B = byNs[c.b];
    if (!A || !B || A.x === null || B.x === null) continue;
    ctx.strokeStyle = '#ff3b3b';
    ctx.lineWidth = 3;
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(toX(A.x), toY(A.y));
    ctx.lineTo(toX(B.x), toY(B.y));
    ctx.stroke();
    for (const P of [A, B]) {
      ctx.beginPath();
      ctx.arc(toX(P.x), toY(P.y), 15, 0, Math.PI * 2);
      ctx.stroke();
    }
  }
}

function render() {
  drawStatic(lastZones);
  drawPaths();
  drawTrails();
  if (latestState) drawAgvs(latestState.agvs);
  drawCollisions();
}

function updateCollisionAlarm(state) {
  const cols = state.collisions || [];
  const badge = document.getElementById('collisionBadge');
  const alarm = document.getElementById('collisionAlarm');
  if (cols.length) {
    badge.style.display = '';
    badge.textContent = `⚠ 碰撞 ${cols.length}`;
    alarm.style.display = '';
    alarm.textContent = '⚠ 碰撞告警：'
      + cols.map((c) => `${c.a}↔${c.b} (${c.d}m)`).join('，')
      + `　|　累计 ${state.collision_count || 0} 次`;
  } else {
    badge.style.display = 'none';
    alarm.style.display = 'none';
  }
}

// ---- 车队表 + 队列 ----
function renderFleet(state) {
  const body = document.getElementById('fleetBody');
  document.getElementById('agvCount').textContent = `(${state.agvs.length} 台)`;
  document.getElementById('queueCount').textContent = state.queued_tasks;

  const yielding = state.yielding || [];
  body.innerHTML = state.agvs.map((a) => {
    const color = AGV_COLORS[(nsIndex[a.ns] || 0) % AGV_COLORS.length];
    const bpct = Math.round((a.battery || 0) * 100);
    const pos = (a.x === null) ? '—' : `${a.x.toFixed(1)}, ${a.y.toFixed(1)}`;
    const sel = a.ns === selectedNs ? ' class="sel"' : '';
    const yieldTag = yielding.includes(a.ns) ? ' <span class="badge yield">让行</span>' : '';
    const navTag = (a.nav_ready === false) ? ' <span class="badge navdown">导航未就绪</span>' : '';
    return `<tr data-ns="${a.ns}"${sel}>
      <td><span style="color:${color};font-weight:700">●</span> ${a.ns}${a.task ? ` <small class="muted">[${a.task}]</small>` : ''}</td>
      <td><span class="badge ${a.state}">${a.state}</span>${yieldTag}${navTag}</td>
      <td><span class="batt"><span class="bar"><i style="width:${bpct}%;background:${batteryColor(a.battery)}"></i></span>${bpct}%</span></td>
      <td>${a.carrying ? '📦' : '—'}</td>
      <td class="muted">${pos}</td>
    </tr>`;
  }).join('');

  // 行点击 -> 选中该车用于手动导航
  body.querySelectorAll('tr[data-ns]').forEach((tr) => {
    tr.addEventListener('click', () => selectRobot(tr.getAttribute('data-ns')));
  });
}

// ---- 任务分配情况 ----
const STATE_LABEL = {
  IDLE: '空闲', TO_PICKUP: '前往取货', LOADING: '装货中',
  TO_DROPOFF: '前往卸货', UNLOADING: '卸货中',
  TO_CHARGER: '前往充电', CHARGING: '充电中',
};

function renderTasks(state) {
  const tasks = state.tasks || [];
  const idle = state.idle_agvs || [];

  // 是否有空闲小车
  const idleEl = document.getElementById('idleInfo');
  idleEl.textContent = idle.length
    ? `· 空闲车 ${idle.length}：${idle.join(', ')}`
    : '· 无空闲车';
  idleEl.className = idle.length ? 'ok-text' : 'muted';

  const body = document.getElementById('taskBody');
  if (!tasks.length) {
    body.innerHTML = '<tr><td colspan="4" class="muted">暂无任务…</td></tr>';
    return;
  }
  // 已分配在前，排队在后
  const ordered = tasks.slice().sort((a, b) => (a.status === b.status ? 0 : a.status === 'assigned' ? -1 : 1));
  body.innerHTML = ordered.map((t) => {
    const agvCell = t.agv
      ? `<span style="color:${AGV_COLORS[(nsIndex[t.agv] || 0) % AGV_COLORS.length]};font-weight:700">●</span> ${t.agv}`
      : '<span class="muted">未分配</span>';
    const stateCell = t.status === 'assigned'
      ? `<span class="badge ${t.agv_state}">${STATE_LABEL[t.agv_state] || t.agv_state}</span>`
      : '<span class="badge queued">排队中</span>';
    return `<tr>
      <td><b>${escapeHtml(t.id)}</b></td>
      <td>${agvCell}</td>
      <td>${stateCell}</td>
      <td class="muted">${escapeHtml(t.pickup || '?')} → ${escapeHtml(t.dropoff || '?')}</td>
    </tr>`;
  }).join('');
}

// ---- 区域下拉框 (仅在 zones 变化时重建) ----
function populateZones(zones) {
  const names = Object.keys(zones);
  const key = names.join(',');
  if (key === populateZones._key) return;
  populateZones._key = key;

  const pickup = names.filter((n) => !n.startsWith('charger'));
  const fill = (sel, list) => {
    const cur = sel.value;
    sel.innerHTML = list.map((n) => `<option value="${n}">${n}</option>`).join('');
    if (list.includes(cur)) sel.value = cur;
  };
  fill(document.getElementById('pickupSel'), pickup);
  fill(document.getElementById('dropoffSel'), pickup);
  if (pickup.length > 1) document.getElementById('dropoffSel').selectedIndex = 1;
}

// ---- 异常日志 (/rosout) ----
const LOG_LEVELS = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL' };
const logs = [];                 // 最近的日志 (最多 100 条，新的在前)
const LOG_MAX = 100;

function onLog(msg) {
  const level = msg.level;
  if (level < 20) return;        // 丢弃 DEBUG
  const sec = msg.stamp ? (msg.stamp.sec || msg.stamp.secs || 0) : 0;
  logs.unshift({
    level,
    name: msg.name || '',
    msg: (msg.msg || '').trim(),
    t: sec ? new Date(sec * 1000).toLocaleTimeString('zh-CN', { hour12: false }) : '',
  });
  if (logs.length > LOG_MAX) logs.length = LOG_MAX;
  renderLogs();
}

function renderLogs() {
  const showInfo = document.getElementById('showInfo').checked;
  const minLevel = showInfo ? 20 : 30;     // 默认只看 WARN 及以上 (异常)
  const list = document.getElementById('logList');
  const rows = logs.filter((l) => l.level >= minLevel);
  if (!rows.length) {
    list.innerHTML = '<li class="muted">暂无' + (showInfo ? '日志' : '异常') + '…</li>';
    return;
  }
  list.innerHTML = rows.map((l) => {
    const lv = LOG_LEVELS[l.level] || String(l.level);
    return `<li class="${lv}"><span class="lt">${l.t}</span>`
      + `<span class="lv">${lv}</span>`
      + `<span class="ln">[${l.name}]</span> ${escapeHtml(l.msg)}</li>`;
  }).join('');
}

function escapeHtml(s) {
  return s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
}

// ---------------------------------------------------------------------------
// ROS / rosbridge 连接
// ---------------------------------------------------------------------------
let ros = null;
let addTaskTopic = null;
const goalPubByNs = {};       // ns -> ROSLIB.Topic(/<ns>/goal_pose)
const pathSubByNs = {};       // ns -> ROSLIB.Topic(/<ns>/plan)

const statusEl = document.getElementById('status');
const wsInput = document.getElementById('wsUrl');
const addBtn = document.getElementById('addTaskBtn');
const manualHint = document.getElementById('manualHint');

wsInput.value = `ws://${location.hostname || 'localhost'}:9090`;

function setStatus(on, text) {
  statusEl.className = 'status ' + (on ? 'on' : 'off');
  statusEl.textContent = text;
  addBtn.disabled = !on;
}

// 为每台车按需建立 /plan 订阅 与 /goal_pose 发布
function ensureRobotTopics(agvs) {
  agvs.forEach((a, i) => {
    if (!(a.ns in nsIndex)) nsIndex[a.ns] = i;     // 颜色下标固定
    if (ros && !pathSubByNs[a.ns]) {
      const t = new ROSLIB.Topic({ ros, name: `/${a.ns}/plan`, messageType: 'nav_msgs/Path' });
      t.subscribe((p) => {
        pathByNs[a.ns] = (p.poses || []).map((ps) => ({
          x: ps.pose.position.x, y: ps.pose.position.y,
        }));
      });
      pathSubByNs[a.ns] = t;
    }
    if (ros && !goalPubByNs[a.ns]) {
      goalPubByNs[a.ns] = new ROSLIB.Topic({
        ros, name: `/${a.ns}/goal_pose`, messageType: 'geometry_msgs/PoseStamped',
      });
    }
  });
}

function connect() {
  if (ros) { try { ros.close(); } catch (e) { /* ignore */ } }
  setStatus(false, '连接中…');
  // 重连时清空旧订阅缓存，避免引用已关闭的 ros 实例
  for (const k of Object.keys(pathSubByNs)) delete pathSubByNs[k];
  for (const k of Object.keys(goalPubByNs)) delete goalPubByNs[k];
  ros = new ROSLIB.Ros({ url: wsInput.value.trim() });

  ros.on('connection', () => {
    setStatus(true, '已连接');
    const stateTopic = new ROSLIB.Topic({ ros, name: '/fleet/state', messageType: 'std_msgs/String' });
    stateTopic.subscribe((msg) => {
      let state;
      try { state = JSON.parse(msg.data); } catch (e) { return; }
      latestState = state;
      ensureRobotTopics(state.agvs);
      // 累积尾迹
      state.agvs.forEach((a) => {
        if (a.x === null || a.y === null) return;
        const t = trailByNs[a.ns] || (trailByNs[a.ns] = []);
        const last = t[t.length - 1];
        if (!last || Math.hypot(a.x - last[0], a.y - last[1]) > 0.05) {
          t.push([a.x, a.y]);
          if (t.length > TRAIL_MAX) t.shift();
        }
      });
      if (state.zones) { lastZones = state.zones; populateZones(state.zones); }
      renderFleet(state);
      renderTasks(state);
      updateCollisionAlarm(state);
      render();
    });
    addTaskTopic = new ROSLIB.Topic({ ros, name: '/fleet/add_task', messageType: 'std_msgs/String' });

    // 异常日志：订阅 /rosout (rcl_interfaces/msg/Log)
    const logTopic = new ROSLIB.Topic({ ros, name: '/rosout', messageType: 'rcl_interfaces/msg/Log' });
    logTopic.subscribe(onLog);
  });

  ros.on('error', () => setStatus(false, '连接错误'));
  ros.on('close', () => {
    setStatus(false, '已断开 (2s 后重试)');
    setTimeout(connect, 2000);
  });
}

function submitTask() {
  if (!addTaskTopic) return;
  const pickup = document.getElementById('pickupSel').value;
  const dropoff = document.getElementById('dropoffSel').value;
  const msgEl = document.getElementById('formMsg');
  if (!pickup || !dropoff) { msgEl.className = 'form-msg err'; msgEl.textContent = '请选择取货区和卸货区'; return; }
  if (pickup === dropoff) { msgEl.className = 'form-msg err'; msgEl.textContent = '取货区与卸货区不能相同'; return; }
  addTaskTopic.publish(new ROSLIB.Message({ data: JSON.stringify({ pickup, dropoff }) }));
  msgEl.className = 'form-msg ok';
  msgEl.textContent = `已下发任务：${pickup} → ${dropoff}`;
}

// ---- 手动导航：选中车 + 点击地图发目标 ----
function agvState(ns) {
  if (!latestState) return null;
  const a = latestState.agvs.find((x) => x.ns === ns);
  return a ? a.state : null;
}

function selectRobot(ns) {
  selectedNs = (selectedNs === ns) ? null : ns;     // 再次点击取消
  updateManualHint();
  if (latestState) { renderFleet(latestState); render(); }
}

function updateManualHint() {
  if (!selectedNs) {
    manualHint.className = 'manual-hint';
    manualHint.textContent = '点击下方车队表中的某台车，再点击地图即可手动导航。';
    canvas.classList.remove('aiming');
    return;
  }
  const st = agvState(selectedNs);
  canvas.classList.add('aiming');
  if (st && st !== 'IDLE') {
    manualHint.className = 'manual-hint warn';
    manualHint.textContent = `已选 ${selectedNs}（当前 ${st}，执行任务中）。手动导航建议选 IDLE 车，否则会与调度目标冲突。再次点击该车取消。`;
  } else {
    manualHint.className = 'manual-hint ok';
    manualHint.textContent = `已选 ${selectedNs}（IDLE）。点击地图任意位置发送导航目标；再次点击该车取消。`;
  }
}

function onMapClick(ev) {
  if (!selectedNs) return;
  const pub = goalPubByNs[selectedNs];
  if (!pub) return;
  const st = agvState(selectedNs);
  if (st && st !== 'IDLE') {
    manualHint.className = 'manual-hint err';
    manualHint.textContent = `${selectedNs} 正在执行任务（${st}），已忽略手动目标。请选 IDLE 车。`;
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
  const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
  const wx = fromX(px), wy = fromY(py);
  // 限制在仓库内墙范围
  const lim = WAREHOUSE.half - 0.4;
  if (Math.abs(wx) > lim || Math.abs(wy) > lim) {
    manualHint.className = 'manual-hint err';
    manualHint.textContent = '目标超出仓库范围，已忽略。';
    return;
  }
  pub.publish(new ROSLIB.Message({
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    pose: { position: { x: wx, y: wy, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
  }));
  manualHint.className = 'manual-hint ok';
  manualHint.textContent = `已向 ${selectedNs} 发送目标 (${wx.toFixed(1)}, ${wy.toFixed(1)})。`;
}

document.getElementById('connectBtn').addEventListener('click', connect);
addBtn.addEventListener('click', submitTask);
document.getElementById('showInfo').addEventListener('change', renderLogs);
document.getElementById('clearLog').addEventListener('click', () => { logs.length = 0; renderLogs(); });
canvas.addEventListener('click', onMapClick);

updateManualHint();
drawStatic(null);   // 先画静态仓库
connect();          // 自动连接
