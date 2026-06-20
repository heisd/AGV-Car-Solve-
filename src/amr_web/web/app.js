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
// ---- 仓库静态几何 (与 worlds/*.world 一致，根据运行场景动态加载) ----
let currentWorldName = 'warehouse';
let VIEW = 7.6;                                // 视野半边长 (m)

const WORLD_LAYOUTS = {
  'warehouse': {
    half: 7.0,
    view: 7.6,
    shelfXs: [-2, -1, 0, 1, 2],
    shelfYs: [4.5, 1.5, -1.5, -4.5],
    shelfSize: { x: 0.9, y: 0.4 },               // 开口朝 -Y，本体中心在 (x, y-0.2)
    pickup: { cx: 5.5, cy: 5.5, sx: 2.5, sy: 2.5 },
    charger: { cx: -5.5, cy: -5.5, sx: 2.0, sy: 2.0 },
    partitions: [],
    decorations: []
  },
  'complex_warehouse': {
    half: 8.0,
    view: 8.6,
    shelves: [
      // NW shelves
      { cx: -4, cy: 5, sx: 0.9, sy: 0.4 }, { cx: -3, cy: 5, sx: 0.9, sy: 0.4 }, { cx: -2, cy: 5, sx: 0.9, sy: 0.4 },
      { cx: -4, cy: 3, sx: 0.9, sy: 0.4 }, { cx: -3, cy: 3, sx: 0.9, sy: 0.4 }, { cx: -2, cy: 3, sx: 0.9, sy: 0.4 },
      // NE shelves
      { cx: 2, cy: 5, sx: 0.9, sy: 0.4 }, { cx: 3, cy: 5, sx: 0.9, sy: 0.4 }, { cx: 4, cy: 5, sx: 0.9, sy: 0.4 },
      { cx: 2, cy: 3, sx: 0.9, sy: 0.4 }, { cx: 3, cy: 3, sx: 0.9, sy: 0.4 }, { cx: 4, cy: 3, sx: 0.9, sy: 0.4 },
      // SW shelves
      { cx: -4, cy: -3, sx: 0.9, sy: 0.4 }, { cx: -3, cy: -3, sx: 0.9, sy: 0.4 }, { cx: -2, cy: -3, sx: 0.9, sy: 0.4 },
      { cx: -4, cy: -5, sx: 0.9, sy: 0.4 }, { cx: -3, cy: -5, sx: 0.9, sy: 0.4 }, { cx: -2, cy: -5, sx: 0.9, sy: 0.4 },
      // SE shelves
      { cx: 2, cy: -3, sx: 0.9, sy: 0.4 }, { cx: 3, cy: -3, sx: 0.9, sy: 0.4 }, { cx: 4, cy: -3, sx: 0.9, sy: 0.4 },
      { cx: 2, cy: -5, sx: 0.9, sy: 0.4 }, { cx: 3, cy: -5, sx: 0.9, sy: 0.4 }, { cx: 4, cy: -5, sx: 0.9, sy: 0.4 }
    ],
    partitions: [
      { cx: 0, cy: 4, sx: 0.2, sy: 4.0, color: '#46566b' },
      { cx: 0, cy: -4, sx: 0.2, sy: 4.0, color: '#46566b' }
    ],
    decorations: [
      { cx: -6, cy: 0.5, sx: 0.8, sy: 2.0, color: '#2a3b4c', type: 'cabinet' },
      { cx: 6, cy: -0.5, sx: 0.8, sy: 2.0, color: '#2a3b4c', type: 'cabinet' },
      { cx: 0, cy: 0, sx: 1.2, sy: 0.8, color: '#3a4b5c', type: 'cart' }
    ]
  },
  'warehouse_complex': {
    half: 9.0,
    view: 9.6,
    shelves: [
      // NW inner
      { cx: -2.5, cy: 2.0, sx: 0.9, sy: 0.4 }, { cx: -2.5, cy: 3.2, sx: 0.9, sy: 0.4 }, { cx: -2.5, cy: 4.4, sx: 0.9, sy: 0.4 }, { cx: -2.5, cy: 5.6, sx: 0.9, sy: 0.4 },
      // NE inner
      { cx: 2.5, cy: 2.0, sx: 0.9, sy: 0.4 }, { cx: 2.5, cy: 3.2, sx: 0.9, sy: 0.4 }, { cx: 2.5, cy: 4.4, sx: 0.9, sy: 0.4 }, { cx: 2.5, cy: 5.6, sx: 0.9, sy: 0.4 },
      // SW inner
      { cx: -2.5, cy: -2.0, sx: 0.9, sy: 0.4 }, { cx: -2.5, cy: -3.2, sx: 0.9, sy: 0.4 }, { cx: -2.5, cy: -4.4, sx: 0.9, sy: 0.4 }, { cx: -2.5, cy: -5.6, sx: 0.9, sy: 0.4 },
      // SE inner
      { cx: 2.5, cy: -2.0, sx: 0.9, sy: 0.4 }, { cx: 2.5, cy: -3.2, sx: 0.9, sy: 0.4 }, { cx: 2.5, cy: -4.4, sx: 0.9, sy: 0.4 }, { cx: 2.5, cy: -5.6, sx: 0.9, sy: 0.4 }
    ],
    partitions: [
      { cx: 0, cy: 0, sx: 0.2, sy: 8.0, color: '#46566b' },
      { cx: 0, cy: 4.0, sx: 2.0, sy: 0.2, color: '#46566b' },
      { cx: 0, cy: -4.0, sx: 2.0, sy: 0.2, color: '#46566b' }
    ],
    decorations: [
      // Cabinets
      { cx: -8.0, cy: 2.5, sx: 0.6, sy: 2.0, color: '#2a3b4c', type: 'cabinet' },
      { cx: 8.0, cy: -2.5, sx: 0.6, sy: 2.0, color: '#2a3b4c', type: 'cabinet' },
      // Carts
      { cx: -8.0, cy: 0.0, sx: 1.0, sy: 0.7, color: '#3a4b5c', type: 'cart' },
      { cx: 8.0, cy: 0.0, sx: 1.0, sy: 0.7, color: '#3a4b5c', type: 'cart' },
      // Wooden Cases
      { cx: -8.0, cy: -1.5, sx: 0.8, sy: 1.6, color: '#8b7355', type: 'box' },
      { cx: 8.0, cy: 1.5, sx: 0.8, sy: 1.6, color: '#8b7355', type: 'box' }
    ]
  },
  'map': {
    half: 8.0,
    view: 8.6,
    shelves: [],
    partitions: [],
    decorations: []
  }
};

let WAREHOUSE = WORLD_LAYOUTS[currentWorldName];

const CORRIDOR_SEGMENTS = {
  'corridor_east':      { x_min: 4.5,  x_max: 6.5,  y_min: -6.5, y_max: 6.5 },
  'corridor_west':      { x_min: -6.5, x_max: -4.5, y_min: -6.5, y_max: 6.5 },
  'corridor_north':     { x_min: -6.5, x_max: 6.5,  y_min: 4.5,  y_max: 6.5 },
  'corridor_south':     { x_min: -6.5, x_max: 6.5,  y_min: -6.5, y_max: -4.5 },
  'corridor_center_ns': { x_min: -1.0, x_max: 1.0,  y_min: -6.5, y_max: 6.5 },
};

const WAIT_POINTS = {
  'corridor_east_south':    { x: 5.5,  y: -6.0 },
  'corridor_east_north':    { x: 5.5,  y: 6.0 },
  'corridor_west_south':    { x: -5.5, y: -6.0 },
  'corridor_west_north':    { x: -5.5, y: 6.0 },
  'corridor_north_east':    { x: 6.0,  y: 5.5 },
  'corridor_north_west':    { x: -6.0, y: 5.5 },
  'corridor_south_east':    { x: 6.0,  y: -5.5 },
  'corridor_south_west':    { x: -6.0, y: -5.5 },
  'corridor_center_south':  { x: 0.0,  y: -6.0 },
  'corridor_center_north':  { x: 0.0,  y: 6.0 },
};

const AGV_COLORS = ['#3da9fc', '#ffa733', '#3ddc84', '#c87cff', '#ff7ca8'];

// ---- 画布坐标变换 (世界 m <-> 像素，y 轴翻转) ----
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const SIZE = canvas.width;
const PAD = 24;
const SPAN = SIZE - 2 * PAD;
// 缩放/平移状态：base* 为 VIEW 基础变换，to* 再叠加滚轮缩放 zoom 与拖动平移 pan
let zoom = 1, panX = 0, panY = 0;
const baseX = (wx) => PAD + ((wx + VIEW) / (2 * VIEW)) * SPAN;
const baseY = (wy) => PAD + ((VIEW - wy) / (2 * VIEW)) * SPAN;
const baseL = (m) => (m / (2 * VIEW)) * SPAN;
const toX = (wx) => baseX(wx) * zoom + panX;
const toY = (wy) => baseY(wy) * zoom + panY;
const toL = (m) => baseL(m) * zoom;
// 像素 -> 世界（toX/toY 的逆，供地图点击用，需先去除 zoom/pan）
const fromX = (px) => ((((px - panX) / zoom) - PAD) / SPAN) * (2 * VIEW) - VIEW;
const fromY = (py) => VIEW - (((((py - panY) / zoom) - PAD) / SPAN) * (2 * VIEW));

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

// ---- 真实占据栅格地图 (/<ns>/map, nav_msgs/OccupancyGrid) ----
let mapBitmap = null;       // 离屏 canvas：渲染好的占据栅格（地图静态，只建一次）
let mapMeta = null;         // {resolution, width, height, originX, originY}
let mapViewLocked = false;  // 真实 /map 已定标 VIEW 后置 true，避免 world_name 切换覆盖缩放
let showRealMap = true;     // true=显示真实 /map 栅格，false=硬编码示意图
let mapSub = null;          // 当前 /map 的 ROSLIB.Topic
let mapSubNs = null;        // 已订阅 map 的命名空间
let mapRetryTimer = null;   // 未收到地图前的重订阅定时器（应对 nav2 错峰晚于浏览器订阅）
let mapRetries = 0;         // 已重订阅次数
let mapUseCbor = true;      // 先用 CBOR；多次失败后降级为无压缩（应对环境不支持 CBOR）
let mapUnavailable = false; // 重试用尽仍无地图 -> 判定 /map 不可用，永久回退硬编码示意图
const MAP_RETRY_MS = 4000;
const MAP_MAX_RETRIES = 12;     // ~48s 仍拿不到 /map 即放弃，回退示意图
const MAP_CBOR_ATTEMPTS = 3;    // 前 3 次用 CBOR，之后降级为无压缩

const pathByNs = {};        // ns -> [{x,y}, ...]  最新 Nav2 全局规划
const trailByNs = {};       // ns -> [[x,y], ...]  最近经过点（尾迹）
const TRAIL_MAX = 50;
let nsIndex = {};           // ns -> 颜色下标（与车队表一致）
let selectedNs = null;      // 手动导航选中的车

// nav_msgs/OccupancyGrid -> 离屏 canvas（一次性，地图静态）。
// data 行优先、row0 在底部(原点处，+Y 向上)；图像 y0 在顶部，故按行翻转。
function onMapMsg(msg) {
  const info = msg.info;
  const w = info.width, h = info.height;
  if (!w || !h) return;
  const data = msg.data;          // Int8: -1 未知 / 0 空闲 / 100 占据
  const off = document.createElement('canvas');
  off.width = w; off.height = h;
  const octx = off.getContext('2d');
  const img = octx.createImageData(w, h);
  for (let row = 0; row < h; row++) {
    const srcBase = row * w;
    const dstBase = (h - 1 - row) * w;        // 行翻转
    for (let col = 0; col < w; col++) {
      const v = data[srcBase + col];
      const di = (dstBase + col) * 4;
      let r, g, b, a;
      if (v < 0) {              // 未知 -> 透明（露出深色画布）
        r = g = b = 0; a = 0;
      } else if (v >= 65) {     // 占据（occupied_thresh 0.65）-> 墙体色
        r = 91; g = 107; b = 130; a = 255;     // #5b6b82
      } else {                  // 空闲 -> 地面色
        r = 17; g = 22; b = 29; a = 255;       // #11161d
      }
      img.data[di] = r; img.data[di + 1] = g; img.data[di + 2] = b; img.data[di + 3] = a;
    }
  }
  octx.putImageData(img, 0, 0);
  mapBitmap = off;
  mapMeta = {
    resolution: info.resolution,
    width: w, height: h,
    originX: info.origin.position.x,
    originY: info.origin.position.y,
  };
  // 自动适配视野到真实地图范围：地图以世界原点为中心，取较大半边长 + 4% 余量。
  // 这样占据栅格、/fleet/state 分区、AGV 位置在任意尺寸地图（±7 仓库 / ±25 大仓）上都对齐显示。
  const halfX = Math.max(Math.abs(mapMeta.originX), Math.abs(mapMeta.originX + w * info.resolution));
  const halfY = Math.max(Math.abs(mapMeta.originY), Math.abs(mapMeta.originY + h * info.resolution));
  VIEW = Math.max(halfX, halfY, 1) * 1.04;
  mapViewLocked = true;
  // 收到地图：停止重订阅、清除不可用标记
  if (mapRetryTimer) { clearInterval(mapRetryTimer); mapRetryTimer = null; }
  mapUnavailable = false;
  updateMapStatus();
  render();
}

// 把占据栅格离屏位图按世界坐标贴到主画布（左下角=origin，Y 翻转已在离屏处理）。
function drawMap() {
  if (!mapBitmap || !mapMeta) return;
  const m = mapMeta;
  const dx = toX(m.originX);                              // 左边界世界 x
  const dy = toY(m.originY + m.height * m.resolution);    // 上边界世界 y
  const dw = toL(m.width * m.resolution);
  const dh = toL(m.height * m.resolution);
  const prev = ctx.imageSmoothingEnabled;
  ctx.imageSmoothingEnabled = false;                      // 栅格保持像素感，不模糊
  ctx.drawImage(mapBitmap, dx, dy, dw, dh);
  ctx.imageSmoothingEnabled = prev;
}

function drawStatic(zones) {
  ctx.clearRect(0, 0, SIZE, SIZE);

  // 底图：有真实占据栅格且开关开启时用 /map，否则回退到硬编码示意图。
  // 栅格已含墙体/货架等物理障碍，故此时跳过硬编码的地面/外墙/货架/隔断/装饰，避免重复。
  const useMap = showRealMap && mapBitmap;
  if (useMap) {
    drawMap();
  } else {
    // 仅在已知有硬编码物理布局的场景下才画地表、外墙与货架，避免在大仓等自定义地图场景中因尺度与回退布局不一致产生视觉错位。
    const isHardcodedWorld = ['warehouse', 'complex_warehouse', 'warehouse_complex'].includes(currentWorldName);
    if (isHardcodedWorld) {
      // 地面
      fillRectC(0, 0, 2 * WAREHOUSE.half, 2 * WAREHOUSE.half, '#11161d');
      // 外墙
      strokeRectC(0, 0, 2 * WAREHOUSE.half, 2 * WAREHOUSE.half, '#46566b');

      // 货架
      if (WAREHOUSE.shelves) {
        for (const s of WAREHOUSE.shelves) {
          fillRectC(s.cx, s.cy, s.sx, s.sy, '#8a6d3b');
        }
      } else if (WAREHOUSE.shelfYs && WAREHOUSE.shelfXs) {
        for (const y of WAREHOUSE.shelfYs) {
          for (const x of WAREHOUSE.shelfXs) {
            fillRectC(x, y - WAREHOUSE.shelfSize.y / 2, WAREHOUSE.shelfSize.x, WAREHOUSE.shelfSize.y, '#8a6d3b');
          }
        }
      }

      // 隔断墙
      if (WAREHOUSE.partitions) {
        for (const p of WAREHOUSE.partitions) {
          fillRectC(p.cx, p.cy, p.sx, p.sy, p.color || '#46566b');
        }
      }

      // 辅助装饰物/柜子
      if (WAREHOUSE.decorations) {
        for (const d of WAREHOUSE.decorations) {
          fillRectC(d.cx, d.cy, d.sx, d.sy, d.color);
          strokeRectC(d.cx, d.cy, d.sx, d.sy, 'rgba(255,255,255,0.15)');
        }
      }
    } else {
      // 对于无硬编码布局的世界（例如大仓场景），不画示意图的地面/外墙/货架，直接填充深色背景底图以供标定层显示
      ctx.fillStyle = '#11161d';
      ctx.fillRect(0, 0, SIZE, SIZE);
    }
  }

  // 取货点 / 充电站：仅在无 /fleet/state 分区数据时，回退到布局内置示意（旧版 ±7 仓库）。
  // 有真实分区时跳过，改由下方按类型着色绘制（针对不同世界自动呈现正确的取货/卸货/充电点）。
  if (!zones && WAREHOUSE.pickup) {
    const p = WAREHOUSE.pickup;
    fillRectC(p.cx, p.cy, p.sx, p.sy, 'rgba(61,220,132,.18)');
    strokeRectC(p.cx, p.cy, p.sx, p.sy, '#3ddc84');
  }
  if (!zones && WAREHOUSE.charger) {
    const c = WAREHOUSE.charger;
    fillRectC(c.cx, c.cy, c.sx, c.sy, 'rgba(255,167,51,.18)');
    strokeRectC(c.cx, c.cy, c.sx, c.sy, '#ffa733');
  }

  // 调度区域 (来自 /fleet/state)：按类型着色 + 语义中心 + 可达接近点(gx,gy)
  // 取货=绿 / 卸货=蓝 / 充电=橙 / 其他=灰；被某台车预约时用车色高亮描边。
  if (zones) {
    ctx.font = '11px sans-serif';
    const owners = (latestState && latestState.zone_owner) || {};
    for (const [name, z] of Object.entries(zones)) {
      const type = name.startsWith('charger') ? 'charger'
        : name.startsWith('pickup') ? 'pickup'
        : name.startsWith('dropoff') ? 'dropoff' : 'other';
      const typeColor = type === 'charger' ? '#ffa733'
        : type === 'pickup' ? '#3ddc84'
        : type === 'dropoff' ? '#3da9fc' : '#7e93b0';
      const owner = owners[name];
      const ownerColor = owner ? AGV_COLORS[(nsIndex[owner] || 0) % AGV_COLORS.length] : null;
      fillRectC(z.cx, z.cy, z.sx, z.sy, hexToRgba(typeColor, 0.14));
      strokeRectC(z.cx, z.cy, z.sx, z.sy, ownerColor || typeColor, owner ? [] : [4, 3]);
      ctx.fillStyle = ownerColor || typeColor;
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

// ---- 走廊段与路权绘制 ----
// 内置走廊/等待点是为 ±7 旧仓库设计的；当 /fleet/state 分区尺度远超此范围
// （如 amr_vision ±25 大仓），说明与当前地图不匹配，隐藏以免误导。
// 后续若需按地图显示走廊，应由后端发布与该地图匹配的走廊几何。
function hardcodedOverlaysMatchMap() {
  if (!lastZones) return true;
  let maxAbs = 0;
  for (const z of Object.values(lastZones)) {
    maxAbs = Math.max(maxAbs, Math.abs(z.cx) + (z.sx || 0) / 2, Math.abs(z.cy) + (z.sy || 0) / 2);
  }
  return maxAbs <= 10;   // 分区≲±7 的世界判定匹配；分区达 ±20 则不匹配
}

function drawCorridors() {
  if (!latestState) return;
  // 优先用后端发布的走廊几何（按地图自带划分）；无则回退硬编码（尺度不符时隐藏）
  const segs = latestState.corridor_segments || (hardcodedOverlaysMatchMap() ? CORRIDOR_SEGMENTS : null);
  if (!segs) return;
  const owners = latestState.segment_owner || {};
  const queues = latestState.segment_queue || {};

  ctx.save();
  ctx.font = '10px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';

  for (const [name, seg] of Object.entries(segs)) {
    const cx = (seg.x_min + seg.x_max) / 2;
    const cy = (seg.y_min + seg.y_max) / 2;
    const w = seg.x_max - seg.x_min;
    const h = seg.y_max - seg.y_min;

    const owner = owners[name];
    const queue = queues[name] || [];

    // 基础颜色 (空闲)
    let fillColor = 'rgba(126, 138, 153, 0.05)';
    let borderColor = 'rgba(126, 138, 153, 0.2)';
    let isReserved = false;

    if (owner) {
      isReserved = true;
      const index = nsIndex[owner] !== undefined ? nsIndex[owner] : 0;
      const ownerColor = AGV_COLORS[index % AGV_COLORS.length];
      fillColor = hexToRgba(ownerColor, 0.12);
      borderColor = hexToRgba(ownerColor, 0.5);
    }

    // 绘制填充
    ctx.fillStyle = fillColor;
    ctx.fillRect(toX(seg.x_min), toY(seg.y_max), toL(w), toL(h));

    // 绘制虚线/实线边框
    ctx.strokeStyle = borderColor;
    ctx.lineWidth = 1.5;
    if (isReserved) {
      ctx.setLineDash([]);
    } else {
      ctx.setLineDash([4, 4]);
    }
    ctx.strokeRect(toX(seg.x_min), toY(seg.y_max), toL(w), toL(h));
    ctx.setLineDash([]);

    // 绘制文字标识
    ctx.fillStyle = isReserved ? '#ffffff' : 'rgba(215, 224, 234, 0.4)';
    const cleanName = name.replace('corridor_', '');
    let label = cleanName;
    if (owner) {
      label += ` [${owner}]`;
    }
    if (queue.length > 0) {
      label += ` (等:${queue.join(',')})`;
    }
    ctx.fillText(label, toX(cx), toY(cy));
  }
  ctx.restore();
}

// ---- 等待点绘制 ----
function drawWaitPoints() {
  // 优先用后端发布的等待点；无则回退硬编码（尺度不符时隐藏）
  const wps = (latestState && latestState.wait_points) || (hardcodedOverlaysMatchMap() ? WAIT_POINTS : null);
  if (!wps) return;
  ctx.save();
  for (const [name, wp] of Object.entries(wps)) {
    const px = toX(wp.x);
    const py = toY(wp.y);
    const r = 4;

    // 绘制圆圈
    ctx.strokeStyle = '#ffa733';
    ctx.fillStyle = '#1a212b';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // 内部实心小点
    ctx.fillStyle = '#ffa733';
    ctx.beginPath();
    ctx.arc(px, py, 1.5, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

// 辅助函数：将 #hex 颜色转换为 rgba
function hexToRgba(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

// ---- 待命点绘制 ----
function drawHomePoints() {
  if (!latestState) return;
  latestState.agvs.forEach((a) => {
    if (a.home_x === undefined || a.home_x === null) return;
    const color = AGV_COLORS[(nsIndex[a.ns] || 0) % AGV_COLORS.length];
    const px = toX(a.home_x);
    const py = toY(a.home_y);
    const r = 5;

    ctx.save();
    // 绘制圆圈
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([2, 2]);
    ctx.beginPath();
    ctx.arc(px, py, r + 2, 0, Math.PI * 2);
    ctx.stroke();

    // 内部实心圆点
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(px, py, 2, 0, Math.PI * 2);
    ctx.fill();

    // 待命点标签
    ctx.fillStyle = 'rgba(215, 224, 234, 0.5)';
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`${a.ns}·待命`, px, py - 10);
    ctx.restore();
  });
}

function render() {
  drawStatic(lastZones);
  drawCorridors();
  drawWaitPoints();
  drawHomePoints();
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
    const homePos = (a.home_x === undefined || a.home_x === null) ? '—' : `${a.home_x.toFixed(1)}, ${a.home_y.toFixed(1)}`;
    const sel = a.ns === selectedNs ? ' class="sel"' : '';
    const yieldTag = yielding.includes(a.ns) ? ' <span class="badge yield">让行</span>' : '';
    const navTag = (state.require_nav_ready && a.nav_ready === false) ? ' <span class="badge navdown">导航未就绪</span>' : '';
    return `<tr data-ns="${a.ns}"${sel}>
      <td><span style="color:${color};font-weight:700">●</span> ${a.ns}${a.task ? ` <small class="muted">[${a.task}]</small>` : ''}</td>
      <td><span class="badge ${a.state}">${a.state}</span>${yieldTag}${navTag}</td>
      <td><span class="batt"><span class="bar"><i style="width:${bpct}%;background:${batteryColor(a.battery)}"></i></span>${bpct}%</span></td>
      <td>${a.carrying ? '📦' : '—'}</td>
      <td class="muted">${pos}</td>
      <td class="muted">${homePos}</td>
    </tr>`;
  }).join('');

  // 行点击 -> 选中该车用于手动导航
  body.querySelectorAll('tr[data-ns]').forEach((tr) => {
    tr.addEventListener('click', () => selectRobot(tr.getAttribute('data-ns')));
  });
}

// ---- 路权与走廊状态表格 ----
function renderRow(state) {
  const body = document.getElementById('rowBody');
  const owners = state.segment_owner || {};
  const queues = state.segment_queue || {};

  // 走廊几何优先用后端发布的（按地图自带划分），回退硬编码
  const segs = state.corridor_segments || CORRIDOR_SEGMENTS;

  // 显示路权总状态
  let activeLocks = 0;
  for (const name of Object.keys(segs)) {
    if (owners[name]) activeLocks++;
  }
  document.getElementById('rowStatus').textContent = activeLocks > 0
    ? `· 已锁 ${activeLocks} 段`
    : '· 全路段空闲';

  body.innerHTML = Object.entries(segs).map(([name, seg]) => {
    const owner = owners[name];
    const queue = queues[name] || [];

    const ownerColor = owner ? AGV_COLORS[(nsIndex[owner] !== undefined ? nsIndex[owner] : 0) % AGV_COLORS.length] : null;
    const ownerCell = owner
      ? `<span style="color:${ownerColor};font-weight:700">●</span> ${owner}`
      : '<span class="badge IDLE">空闲</span>';

    const queueCell = queue.length
      ? queue.map(q => {
          const qColor = AGV_COLORS[(nsIndex[q] !== undefined ? nsIndex[q] : 0) % AGV_COLORS.length];
          return `<span style="color:${qColor};font-weight:700">●</span> ${q}`;
        }).join(', ')
      : '<span class="muted">—</span>';

    const bounds = `x: [${seg.x_min}, ${seg.x_max}], y: [${seg.y_min}, ${seg.y_max}]`;
    const cleanName = name.replace('corridor_', '');

    return `<tr>
      <td><b>${cleanName}</b></td>
      <td>${ownerCell}</td>
      <td>${queueCell}</td>
      <td class="muted">${bounds}</td>
    </tr>`;
  }).join('');
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

// ---- 系统异常总览 ----
const ANOM_ICON = { error: '⛔', warn: '⚠', info: 'ℹ' };

function renderAnomalies(state) {
  const anoms = state.anomalies || [];
  const countEl = document.getElementById('anomalyCount');
  const list = document.getElementById('anomalyList');
  const errs = anoms.filter((a) => a.level === 'error').length;
  const warns = anoms.filter((a) => a.level === 'warn').length;
  if (!anoms.length) {
    countEl.textContent = '· 正常';
    countEl.className = 'ok-text';
    list.innerHTML = '<li class="muted">系统正常，无异常…</li>';
    return;
  }
  countEl.textContent = '· ' + (errs ? `${errs} 严重 ` : '') + (warns ? `${warns} 警告` : '');
  countEl.className = errs ? 'err-text' : 'warn-text';
  list.innerHTML = anoms.map((a) =>
    `<li class="anom ${a.level}"><span class="ai">${ANOM_ICON[a.level] || ''}</span>`
    + `<b>${escapeHtml(a.ns)}</b> <span class="atype">${escapeHtml(a.type)}</span> `
    + `<span class="muted">${escapeHtml(a.msg)}</span></li>`).join('');
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
// 相机视频流 (web_video_server MJPEG) —— 参考 LabRobot/wheeltec_dashboard 接法。
// 每台车一块 <img>，src 指向 http://<host>:<port>/stream?topic=/<ns>/camera/image_raw。
// 注意: WSL2 下 MJPEG 长连接偶发卡帧；直连 video_port 一般可用，必要时可改快照轮询。
// ---------------------------------------------------------------------------
const camPortEl = document.getElementById('camPort');
const camQualityEl = document.getElementById('camQuality');
const camReloadEl = document.getElementById('camReload');
const cameraGrid = document.getElementById('cameraGrid');
let camNsKey = '';   // 当前已建 tile 的 ns 集合签名（变化才重建，避免重启流闪断）

function videoBase() {
  const port = ((camPortEl && camPortEl.value) || '8082').trim();
  return `${location.protocol}//${location.hostname || 'localhost'}:${port}`;
}

function camStreamUrl(ns) {
  const q = Math.max(1, Math.min(100, parseInt(camQualityEl && camQualityEl.value, 10) || 60));
  // 注意: web_video_server 不会解码 %2F，topic 的斜杠必须保持原样。
  // 不能用 URLSearchParams（它会把 "/" 编码成 "%2F" -> "Invalid topic name"）。
  const topic = `/${ns}/camera/image_raw`;
  return `${videoBase()}/stream?topic=${topic}&type=mjpeg&quality=${q}&_=${Date.now()}`;
}

function startCamStream(img, ns) {
  const tile = img.closest('.cam-tile');
  const err = tile ? tile.querySelector('.cam-err') : null;
  img.onerror = () => { if (err) err.hidden = false; };
  img.onload = () => { if (err) err.hidden = true; };
  img.src = camStreamUrl(ns);
}

function renderCameras(agvs) {
  if (!cameraGrid) return;
  const list = (agvs || []).map((a) => a.ns);
  const key = list.join(',');
  if (key !== camNsKey) {       // ns 集合变化 -> 重建 tile（否则只更新徽标）
    camNsKey = key;
    if (!list.length) {
      cameraGrid.innerHTML = '<div class="muted cam-empty">等待车队相机…(需 web_video_server 在线)</div>';
    } else {
      cameraGrid.innerHTML = list.map((ns) => {
        const color = AGV_COLORS[(nsIndex[ns] || 0) % AGV_COLORS.length];
        return `<div class="cam-tile" data-ns="${ns}">
          <img class="cam-img" alt="${ns} camera" />
          <div class="cam-label"><span style="color:${color}">●</span> ${ns}
            <span class="cam-sem sem-clear" data-ns="${ns}">clear</span></div>
          <div class="cam-err" hidden>⚠ 无视频流（检查 web_video_server / 相机话题）</div>
        </div>`;
      }).join('');
      cameraGrid.querySelectorAll('.cam-img').forEach((img) => {
        startCamStream(img, img.closest('.cam-tile').getAttribute('data-ns'));
      });
    }
  }
  // 每帧更新语义徽标（person/pallet/clear，来自 fleet_manager_ai 的 YOLO 层）
  (agvs || []).forEach((a) => {
    const sem = cameraGrid.querySelector(`.cam-sem[data-ns="${a.ns}"]`);
    if (!sem) return;
    const label = a.semantic || 'clear';
    sem.textContent = a.semantic_stop ? `${label}·停车` : label;
    sem.className = 'cam-sem ' + (label === 'person' ? 'sem-person'
      : label === 'pallet' ? 'sem-pallet' : 'sem-clear');
  });
}

function reloadCameras() {
  if (!cameraGrid) return;
  cameraGrid.querySelectorAll('.cam-img').forEach((img) => {
    startCamStream(img, img.closest('.cam-tile').getAttribute('data-ns'));
  });
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
const mapStatusEl = document.getElementById('mapStatus');
const showRealMapEl = document.getElementById('showRealMap');

wsInput.value = `ws://${location.hostname || 'localhost'}:9090`;

function setStatus(on, text) {
  statusEl.className = 'status ' + (on ? 'on' : 'off');
  statusEl.textContent = text;
  addBtn.disabled = !on;
}

// 地图加载状态提示 + 回退时禁用「真实地图」开关
function updateMapStatus() {
  if (!mapStatusEl) return;
  if (mapUnavailable) {
    mapStatusEl.textContent = '· /map 不可用，已回退示意图';
    mapStatusEl.className = 'warn-text';
    showRealMapEl.disabled = true;
  } else if (!mapBitmap) {
    mapStatusEl.textContent = '· 加载真实地图…';
    mapStatusEl.className = 'muted';
    showRealMapEl.disabled = false;
  } else {
    mapStatusEl.textContent = '';
    showRealMapEl.disabled = false;
  }
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

// 订阅占据栅格地图。每车一套命名空间化 Nav2，map_server 发布的是 /<ns>/map（非全局 /map），
// 各车共用同一张 warehouse_map.yaml，取首台车的命名空间即可。
function ensureMapSub(ns) {
  if (!ros || !ns || mapSubNs === ns) return;
  mapSubNs = ns;
  mapRetries = 0;
  mapUseCbor = true;
  mapUnavailable = false;
  updateMapStatus();
  subscribeMap();
  // /map 是 latched(transient_local)：rosbridge 仅在订阅时已存在发布者才会匹配该 QoS。
  // 但 nav2 错峰启动(12–36s)，浏览器可能先订阅 -> QoS 退化成 volatile 收不到。
  // 故未拿到地图前每 4s 重订阅一次，触发 rosbridge 重新匹配 transient_local 并立刻补发 latched 帧。
  if (mapRetryTimer) clearInterval(mapRetryTimer);
  mapRetryTimer = setInterval(() => {
    if (mapBitmap) { clearInterval(mapRetryTimer); mapRetryTimer = null; return; }
    mapRetries++;
    if (mapRetries >= MAP_CBOR_ATTEMPTS) mapUseCbor = false;   // CBOR 多次无果 -> 降级无压缩
    if (mapRetries > MAP_MAX_RETRIES) {
      // 回退机制：重试用尽仍拿不到 /map（话题不存在 / QoS 不匹配 / 解码失败等）
      // -> 判定 /map 不可用，停止重试并永久回退到硬编码示意图。
      clearInterval(mapRetryTimer); mapRetryTimer = null;
      mapUnavailable = true;
      if (mapSub) { try { mapSub.unsubscribe(); } catch (e) { /* ignore */ } mapSub = null; }
      console.warn('[map] /' + mapSubNs + '/map 不可用，已回退到示意图');
      updateMapStatus();
      render();
      return;
    }
    subscribeMap();
  }, MAP_RETRY_MS);
}

function subscribeMap() {
  if (!ros || !mapSubNs) return;
  if (mapSub) { try { mapSub.unsubscribe(); } catch (e) { /* ignore */ } }
  const opts = { ros, name: `/${mapSubNs}/map`, messageType: 'nav_msgs/OccupancyGrid' };
  if (mapUseCbor) opts.compression = 'cbor';   // 栅格较大，CBOR 比 JSON 省带宽；不支持时降级无压缩
  mapSub = new ROSLIB.Topic(opts);
  mapSub.subscribe(onMapMsg);
}

function connect() {
  if (ros) { try { ros.close(); } catch (e) { /* ignore */ } }
  setStatus(false, '连接中…');
  // 重连时清空旧订阅缓存，避免引用已关闭的 ros 实例
  for (const k of Object.keys(pathSubByNs)) delete pathSubByNs[k];
  for (const k of Object.keys(goalPubByNs)) delete goalPubByNs[k];
  // 复位地图订阅（保留已缓存的 mapBitmap，重连期间地图仍可显示）
  if (mapRetryTimer) { clearInterval(mapRetryTimer); mapRetryTimer = null; }
  mapSub = null; mapSubNs = null;
  mapRetries = 0; mapUnavailable = false; updateMapStatus();
  ros = new ROSLIB.Ros({ url: wsInput.value.trim() });

  ros.on('connection', () => {
    setStatus(true, '已连接');
    const stateTopic = new ROSLIB.Topic({ ros, name: '/fleet/state', messageType: 'std_msgs/String' });
    stateTopic.subscribe((msg) => {
      let state;
      try { state = JSON.parse(msg.data); } catch (e) { return; }
      latestState = state;
      
      // 动态更新仓库世界底图及坐标范围
      if (state.world_name && currentWorldName !== state.world_name) {
        currentWorldName = state.world_name;
        if (!WORLD_LAYOUTS[currentWorldName]) {
          // 动态注册未知的场景（例如大仓 my_map 等），定义为空白模板以防止错误引用或错位回退
          WORLD_LAYOUTS[currentWorldName] = {
            half: 8.0,
            view: 8.6,
            shelves: [],
            partitions: [],
            decorations: []
          };
        }
        WAREHOUSE = WORLD_LAYOUTS[currentWorldName];
        // 真实 /map 已定标缩放时不覆盖（避免 world_name 与内置布局尺度冲突，如 amr_vision 的 ±25 大仓）
        if (!mapViewLocked) VIEW = WORLD_LAYOUTS[currentWorldName].view;
        console.log("检测到地图场景切换: " + currentWorldName + ", VIEW缩放范围: " + VIEW);
      }
      
      ensureRobotTopics(state.agvs);
      if (state.agvs && state.agvs.length) ensureMapSub(state.agvs[0].ns);
      renderCameras(state.agvs);     // 相机视频流面板（按车队动态建块 + 更新语义徽标）
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
      renderRow(state);
      renderTasks(state);
      renderAnomalies(state);
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

let _dragMoved = false;   // 刚发生过拖动平移时置 true，抑制紧随的 click 误下发目标
function onMapClick(ev) {
  if (_dragMoved) { _dragMoved = false; return; }
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
  // 限制在地图范围内：有真实 /map 时按其边界（x/y 分别，地图可能非方形），否则用内置布局范围
  let limX, limY;
  if (mapViewLocked && mapMeta) {
    limX = Math.max(Math.abs(mapMeta.originX), Math.abs(mapMeta.originX + mapMeta.width * mapMeta.resolution)) - 0.4;
    limY = Math.max(Math.abs(mapMeta.originY), Math.abs(mapMeta.originY + mapMeta.height * mapMeta.resolution)) - 0.4;
  } else {
    limX = limY = WAREHOUSE.half - 0.4;
  }
  if (Math.abs(wx) > limX || Math.abs(wy) > limY) {
    manualHint.className = 'manual-hint err';
    manualHint.textContent = '目标超出地图范围，已忽略。';
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
document.getElementById('showRealMap').addEventListener('change', (e) => {
  showRealMap = e.target.checked;
  render();
});
document.getElementById('clearLog').addEventListener('click', () => { logs.length = 0; renderLogs(); });
canvas.addEventListener('click', onMapClick);
if (camReloadEl) camReloadEl.addEventListener('click', reloadCameras);

// ---- 地图鼠标缩放(滚轮，围绕光标) + 平移(放大后拖动) ----
(function setupZoomPan() {
  let dragging = false, downX = 0, downY = 0, startPanX = 0, startPanY = 0, moved = false;
  const sc = () => canvas.width / canvas.getBoundingClientRect().width;
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const mx = (e.clientX - rect.left) * (canvas.width / rect.width);
    const my = (e.clientY - rect.top) * (canvas.height / rect.height);
    const nz = Math.min(8, Math.max(1, zoom * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
    if (nz === zoom) return;
    panX = mx - ((mx - panX) / zoom) * nz;   // 保持光标处世界点不动
    panY = my - ((my - panY) / zoom) * nz;
    zoom = nz;
    if (zoom <= 1.0001) { zoom = 1; panX = 0; panY = 0; }   // 复位避免漂移
    render();
  }, { passive: false });
  canvas.addEventListener('mousedown', (e) => {
    if (zoom <= 1) return;                   // 未放大时不平移，保留点击下发目标
    dragging = true; moved = false;
    downX = e.clientX; downY = e.clientY; startPanX = panX; startPanY = panY;
  });
  window.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    const k = sc(), dx = e.clientX - downX, dy = e.clientY - downY;
    if (Math.abs(dx) + Math.abs(dy) > 3) moved = true;
    panX = startPanX + dx * k; panY = startPanY + dy * k;
    render();
  });
  window.addEventListener('mouseup', () => {
    if (dragging && moved) _dragMoved = true;   // 抑制紧随的 click
    dragging = false;
  });
})();

updateManualHint();
updateMapStatus();  // 初始化地图状态提示
drawStatic(null);   // 先画静态仓库
connect();          // 自动连接
