import { interpretSnapshot } from "./simulator-bridge.mjs";

const $ = (selector, root = document) => root.querySelector(selector);
const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];

const dom = {
  canvas: $("#simulator-canvas"),
  canvasWrap: $("#canvas-wrap"),
  modelStatus: $("#model-status"),
  scenario: $("#scenario-select"),
  run: $("#run-toggle"),
  step: $("#step-button"),
  reset: $("#reset-button"),
  export: $("#export-button"),
  speed: $("#speed-select"),
  speedLabel: $("#speed-label"),
  timecode: $("#timecode"),
  transport: $("#transport-state-label"),
  timelineFill: $("#timeline-fill"),
  timelineCursor: $("#timeline-cursor"),
  sequence: $("#sequence-list"),
  sequenceCount: $("#sequence-count"),
  activePart: $("#active-part-label"),
  activeAction: $("#active-action-label"),
  inspectorPartId: $("#inspector-part-id"),
  inspectorPart: $("#inspector-part"),
  inspectorPartMeta: $("#inspector-part-meta"),
  partState: $("#part-state-chip"),
  healthTitle: $("#health-title"),
  healthScore: $("#health-score"),
  healthRing: $(".health-ring"),
  sigma: $("#position-sigma"),
  force: $("#tool-force"),
  forceFill: $("#force-fill"),
  forceGateLabel: $("#force-gate-label"),
  clearance: $("#clearance-value"),
  views: $("#visible-views"),
  poseX: $("#pose-x"),
  poseY: $("#pose-y"),
  poseZ: $("#pose-z"),
  rotX: $("#rot-x"),
  rotY: $("#rot-y"),
  rotZ: $("#rot-z"),
  poseHeading: $("#pose-heading"),
  sigmaX: $("#sigma-x"),
  sigmaY: $("#sigma-y"),
  sigmaZ: $("#sigma-z"),
  guardCard: $("#guard-card"),
  guardLabel: $("#guard-label"),
  guardDetail: $("#guard-detail"),
  eventStream: $("#event-stream"),
  sigmaChart: $("#sigma-chart"),
  callout: $("#active-callout"),
  toastStack: $("#toast-stack"),
  pin: $("#pin-inspector"),
};

const sequence = [
  { key: "housing", progress: 0.00, label: "HOUSING", part: "HOUSING", action: "FIXING TO DATUMS", state: "FIXTURED", guard: "DATUM RESIDUAL ≤ 10 µm" },
  { key: "shafts", progress: 0.08, label: "SHAFTS", part: "SHAFT C", action: "SEATING SHAFT SET", state: "GRASPED", guard: "AXIS ERROR ≤ 0.8°" },
  { key: "output_gear", progress: 0.18, label: "OUTPUT GEAR", part: "OUTPUT GEAR · 24T", action: "ALIGNING TO SHAFT C", state: "HELD", guard: "CENTER ERROR ≤ 12 µm" },
  { key: "idler_gear", progress: 0.36, label: "IDLER GEAR", part: "IDLER GEAR · 18T", action: "MESHING WITH OUTPUT", state: "HELD", guard: "PLACEMENT FORCE ≤ 5 mN" },
  { key: "input_gear", progress: 0.52, label: "INPUT GEAR", part: "INPUT GEAR · 12T", action: "MESHING WITH IDLER", state: "HELD", guard: "BACKLASH ≥ 10 µm" },
  { key: "rotation_test", progress: 0.65, label: "ROTATION TEST", part: "GEAR TRAIN", action: "DRIVING +90° / −90°", state: "ENGAGED", guard: "RATIO ERROR ≤ 1%" },
  { key: "cover_handoff", progress: 0.75, label: "COVER HANDOFF", part: "HOUSING COVER", action: "TRANSFERRING ARM 1 → 3", state: "DUAL HELD", guard: "BOTH GRASPS VERIFIED" },
  { key: "cover_closure", progress: 0.84, label: "COVER CLOSURE", part: "HOUSING COVER", action: "LOWERING TO LATCH", state: "HELD", guard: "INSERTION FORCE ≤ 50 mN" },
  { key: "post_test", progress: 0.92, label: "POST-COVER TEST", part: "ASSEMBLED GEARBOX", action: "FINAL FUNCTION CHECK", state: "FIXTURED", guard: "BACKLASH 10–40 µm" },
  { key: "complete", progress: 1.00, label: "ACCEPTED", part: "GEARBOX ASSEMBLY", action: "ALL GATES PASSED", state: "ACCEPTED", guard: "REPORT DIGEST SEALED" },
];

const injectedFaults = {
  collision: { at: 0.47, message: "Predicted arm separation below 0.5 mm", type: "error" },
  occlusion: { at: 0.31, message: "Observation paused: only one distinct camera view", type: "sensor" },
  "insertion-force": { at: 0.57, message: "Insertion force exceeded 50 mN gate", type: "error" },
  "fault-suite": { at: 0.39, message: "Fault suite injected recoverable pose dropout", type: "sensor" },
};

const componentCatalog = {
  1: { name: "INPUT SHAFT", meta: "Ø 0.35 × 1.55 mm" },
  2: { name: "IDLER SHAFT", meta: "Ø 0.35 × 1.55 mm" },
  3: { name: "OUTPUT SHAFT", meta: "Ø 0.35 × 1.55 mm" },
  4: { name: "OUTPUT GEAR", meta: "24 teeth · pitch Ø 2.40 mm" },
  5: { name: "IDLER GEAR", meta: "18 teeth · pitch Ø 1.80 mm" },
  6: { name: "INPUT GEAR", meta: "12 teeth · pitch Ø 1.20 mm" },
  7: { name: "HOUSING COVER", meta: "6.00 × 4.00 × 0.20 mm" },
};

const state = {
  running: false,
  terminal: false,
  completed: false,
  mode: "preview",
  scenario: dom.scenario.value,
  simulator: null,
  wasmModule: null,
  rawSnapshot: null,
  sceneDescription: null,
  sceneFrame: null,
  telemetry: null,
  poseError: null,
  componentId: 4,
  componentName: null,
  executivePhase: null,
  cycle: 3,
  time: 0.06,
  progress: 0.285,
  speed: 1,
  view: "iso",
  layers: { sensors: true, collision: true, uncertainty: false },
  camera: { yaw: -0.08, pitch: -0.15, zoom: 1, targetYaw: -0.08, targetPitch: -0.15, targetZoom: 1 },
  drag: null,
  activeIndex: 2,
  lastPhase: "output_gear",
  lastTimestamp: performance.now(),
  accumulator: 0,
  faultRaised: false,
  inspectorPinned: false,
  sigmaHistory: [5.8, 6.1, 5.9, 6.4, 6.3, 6.6, 6.2, 6.0, 6.2],
  events: [],
  dimensions: { width: 0, height: 0, dpr: 1 },
};

function formatTime(seconds) {
  const mins = Math.floor(seconds / 60).toString().padStart(2, "0");
  const secs = Math.floor(seconds % 60).toString().padStart(2, "0");
  const ms = Math.floor((seconds % 1) * 1000).toString().padStart(3, "0");
  return `${mins}:${secs}.${ms}`;
}

function clamp(value, min, max) { return Math.max(min, Math.min(max, value)); }
function lerp(a, b, t) { return a + (b - a) * t; }
function ease(t) { return t * t * (3 - 2 * t); }

function phaseForProgress(progress) {
  let result = sequence[0];
  for (const phase of sequence) {
    if (progress + 0.00001 >= phase.progress) result = phase;
  }
  return result;
}

function setModelBadge(status, detail, kind = "online") {
  dom.modelStatus.innerHTML = `
    <span class="status-dot status-dot--${kind}"></span>
    <span><small>MODEL</small><strong>${status}</strong></span>
  `;
  dom.modelStatus.title = detail;
}

function toast(message, tone = "normal") {
  const item = document.createElement("div");
  item.className = `toast ${tone === "warning" ? "warning" : ""}`;
  item.textContent = message;
  dom.toastStack.append(item);
  setTimeout(() => item.remove(), 3600);
}

function addEvent(message, type = "info", timestamp = state.time) {
  state.events.unshift({ message, type, timestamp });
  state.events = state.events.slice(0, 7);
  renderEvents();
}

function renderEvents() {
  const rows = state.events.length ? state.events : [
    { timestamp: 0.060, type: "info", message: "Visual servo acquired output_gear" },
    { timestamp: 0.040, type: "pass", message: "Shaft C axis gate passed" },
    { timestamp: 0.020, type: "sensor", message: "5 distinct camera views fused" },
    { timestamp: 0, type: "pass", message: "Scenario digest verified" },
  ];

  dom.eventStream.innerHTML = rows.map((event) => {
    const safe = event.message.replace(/[&<>]/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" })[char]);
    return `<div><time>${event.timestamp.toFixed(3).padStart(6, "0")}</time><i class="event-${event.type}"></i><span>${safe}</span></div>`;
  }).join("");
}

function updateFromWasm(snapshot) {
  state.rawSnapshot = snapshot;
  const interpreted = interpretSnapshot(snapshot, state.progress);
  state.cycle = interpreted.cycle ?? state.cycle + 1;
  state.time = interpreted.timeSeconds ?? state.time + 0.02;
  state.progress = interpreted.progress;
  state.telemetry = interpreted.telemetry;
  state.poseError = interpreted.poseError;
  state.componentId = interpreted.componentId;
  state.componentName = interpreted.componentName;
  state.executivePhase = interpreted.executivePhase;
  state.sceneFrame = interpreted.sceneFrame;
  state.terminal = interpreted.terminal || state.simulator?.terminal || false;
  state.completed = interpreted.completed || state.simulator?.completed || false;

  for (const event of interpreted.events) addEvent(event, event.includes("rejected") || event.includes("aborted") ? "error" : "info");
  if (interpreted.injectedFault) addEvent(`Injected fault: ${interpreted.injectedFault}`, "sensor");
  if (state.terminal) stopRun();
}

async function initializeSimulator({ preservePosition = false } = {}) {
  stopRun();
  state.scenario = dom.scenario.value;
  state.terminal = false;
  state.completed = false;
  state.faultRaised = false;
  state.rawSnapshot = null;
  state.sceneDescription = null;
  state.sceneFrame = null;
  state.telemetry = null;
  state.poseError = null;
  state.componentId = null;
  state.componentName = null;
  state.executivePhase = null;
  state.events = [];

  if (!preservePosition) {
    state.cycle = 0;
    state.time = 0;
    state.progress = 0.001;
    state.lastPhase = "housing";
  }

  if (state.wasmModule) {
    try {
      state.simulator = new state.wasmModule.ReferenceSimulator(state.scenario);
      state.sceneDescription = JSON.parse(state.simulator.sceneDescriptionJson());
      state.sceneFrame = JSON.parse(state.simulator.sceneFrameJson());
      state.mode = "wasm";
      setModelBadge("RUST / WASM", "Renderer is driven by the versioned Rust machine scene.");
      addEvent(`Loaded compiled scenario ${state.scenario}`, "pass", 0);
      updateUi();
      return;
    } catch (error) {
      console.warn("WASM scenario initialization failed", error);
    }
  }

  state.simulator = null;
  state.mode = "preview";
  setModelBadge("UI PREVIEW", "Deterministic visualization fallback; acceptance remains in the Rust core.", "pending");
  addEvent(`UI preview initialized for ${state.scenario}`, "info", 0);
  updateUi();
}

async function connectWasm() {
  setModelBadge("CONNECTING", "Looking for the generated WebAssembly module.", "pending");
  try {
    const module = await import("../dist/wasm/pipe_sim_wasm.js");
    await module.default();
    state.wasmModule = module;
    await initializeSimulator({ preservePosition: false });
    toast("Rust/WASM reference core connected");
  } catch (error) {
    console.info("WASM wrapper unavailable; using the UI preview model.", error);
    state.wasmModule = null;
    await initializeSimulator({ preservePosition: true });
  }
}

function stepFallback() {
  const before = phaseForProgress(state.progress);
  state.cycle += 1;
  state.time += 0.02;

  const baseRate = state.scenario === "fault-suite" ? 0.0023 : 0.002;
  state.progress = clamp(state.progress + baseRate, 0, 1);

  const fault = injectedFaults[state.scenario];
  if (fault && !state.faultRaised && state.progress >= fault.at) {
    state.faultRaised = true;
    addEvent(fault.message, fault.type);
    if (state.scenario === "collision" || state.scenario === "insertion-force") {
      state.terminal = true;
      state.completed = false;
      stopRun();
      toast("Simulation stopped at a guarded failure", "warning");
    } else {
      state.progress = Math.max(0, state.progress - 0.04);
      addEvent("Recovery branch scheduled", "info");
    }
  }

  const after = phaseForProgress(state.progress);
  if (before.key !== after.key) {
    addEvent(`Entered ${after.label.toLowerCase()} phase`, after.key === "complete" ? "pass" : "info");
    state.lastPhase = after.key;
  }

  if (state.progress >= 1) {
    state.terminal = true;
    state.completed = true;
    stopRun();
    addEvent("Gearbox acceptance report sealed", "pass");
    toast("Reference sequence completed");
  }
}

function stepSimulation() {
  if (state.terminal) return;
  if (state.mode === "wasm" && state.simulator) {
    try {
      const snapshot = JSON.parse(state.simulator.stepJson());
      updateFromWasm(snapshot);
    } catch (error) {
      stopRun();
      addEvent(`Simulator error: ${error.message ?? error}`, "error");
      toast("The reference core stopped with an error", "warning");
    }
  } else {
    stepFallback();
  }
  updateUi();
}

function startRun() {
  if (state.terminal) {
    initializeSimulator({ preservePosition: false }).then(startRun);
    return;
  }
  state.running = true;
  dom.run.classList.add("running");
  dom.run.querySelector("span").textContent = "PAUSE";
  dom.transport.closest(".transport-panel").classList.add("running");
  updateUi();
}

function stopRun() {
  state.running = false;
  dom.run.classList.remove("running");
  dom.run.querySelector("span").textContent = state.terminal ? "RESET" : "RUN";
  dom.transport.closest(".transport-panel").classList.remove("running");
  updateUi();
}

function toggleRun() {
  if (state.terminal) initializeSimulator({ preservePosition: false });
  else if (state.running) stopRun();
  else startRun();
}

function metricSnapshot() {
  const phase = phaseForProgress(state.progress);
  const gearInsertion = ["output_gear", "idler_gear", "input_gear"].includes(phase.key);
  const forceLimitMn = gearInsertion ? 5 : 50;

  if (state.mode === "wasm" && state.telemetry) {
    const { sigmaUm: sigma, forceMn: force, clearanceMm: clearance, views } = state.telemetry;
    const healthy = sigma <= 10 && force <= forceLimitMn && clearance >= 0.01 && views >= 2 && !state.terminal;
    const accepted = state.completed;
    const score = accepted ? 100 : clamp(Math.round(
      100
      - Math.max(0, sigma / 10 - 0.6) * 30
      - Math.max(0, force / forceLimitMn - 0.6) * 25
      - Math.max(0, 0.01 - clearance) * 2_000
      - Math.max(0, 2 - views) * 24,
    ), 0, 99);
    return { sigma, force, clearance, views, healthy: accepted || healthy, score, forceLimitMn };
  }

  const wave = Math.sin(state.time * 2.6 + state.activeIndex) * 0.7;
  const progressPulse = Math.sin(state.progress * Math.PI * 16) * 0.35;
  const fault = injectedFaults[state.scenario];
  const nearFault = fault && Math.abs(state.progress - fault.at) < 0.035;

  let sigma = 5.8 + wave * 0.5 + progressPulse;
  let force = 2.4 + Math.abs(Math.sin(state.time * 1.7)) * 2.1;
  let clearance = 0.204 - Math.abs(Math.sin(state.progress * Math.PI * 7)) * 0.038;
  let views = 5 + (Math.sin(state.time * 0.8) > 0.78 ? 1 : 0);

  if (nearFault && state.scenario === "occlusion") views = 1;
  if (nearFault && state.scenario === "collision") clearance = 0.006;
  if (nearFault && state.scenario === "insertion-force") force = 54.2;
  if (state.terminal && !state.completed) sigma = Math.max(sigma, 11.4);

  const healthy = sigma <= 10 && force <= forceLimitMn && clearance >= 0.01 && views >= 2;
  const score = state.completed ? 100 : clamp(Math.round(97 - sigma / 5 - Math.max(0, force - 5) * 0.25 - Math.max(0, 2 - views) * 20), 18, 99);
  return { sigma, force, clearance, views, healthy, score, forceLimitMn };
}

function updateSequence(phase) {
  const items = $$("li", dom.sequence);
  const uiIndexByKey = new Map([
    ["housing", 0], ["shafts", 1], ["output_gear", 2], ["idler_gear", 3], ["input_gear", 4],
    ["rotation_test", 5], ["cover_handoff", 6], ["cover_closure", 7], ["post_test", 8], ["complete", 9],
  ]);
  const index = uiIndexByKey.get(phase.key) ?? 0;
  items.forEach((item, itemIndex) => {
    item.classList.toggle("complete", itemIndex < index || phase.key === "complete");
    item.classList.toggle("active", itemIndex === index && phase.key !== "complete");
  });
  dom.sequenceCount.textContent = phase.key === "complete" ? "09 / 09" : `${String(Math.min(index + 1, 9)).padStart(2, "0")} / 09`;
}

function signed(value, digits) {
  const formatted = Number(value).toFixed(digits);
  return Number(value) >= 0 ? `+${formatted}` : formatted.replace("-", "−");
}

function fallbackPoseError() {
  const wobble = Math.sin(state.time * 1.5) * 0.006;
  return {
    translationMm: [wobble, -wobble * 0.6, 0.012 - state.progress * 0.01],
    rotationDeg: [0.06 + wobble * 9, -0.11 - wobble * 8, 0.4 + Math.sin(state.time) * 0.25],
  };
}

function inspectorComponent(phase) {
  const fallbackByPhase = {
    shafts: 3,
    output_gear: 4,
    idler_gear: 5,
    input_gear: 6,
    rotation_test: 6,
    cover_handoff: 7,
    cover_closure: 7,
    post_test: 7,
    complete: 7,
  };
  const id = state.componentId ?? fallbackByPhase[phase.key] ?? null;
  return { id, ...(componentCatalog[id] ?? { name: phase.part.replace(/ ·.+$/, ""), meta: "Registered assembly feature" }) };
}

function updateInspector(phase, metrics) {
  const component = inspectorComponent(phase);
  const pose = state.poseError ?? fallbackPoseError();
  const executiveState = {
    locate: "LOCATING",
    pick: "GRASPING",
    handoff: "HANDOFF",
    align: "ALIGNING",
    insert: "INSERTING",
    mesh: "MESHING",
    verify: "VERIFYING",
  }[state.executivePhase] ?? phase.state;

  dom.inspectorPartId.textContent = component.id ? `PART ${String(component.id).padStart(2, "0")}` : "CELL DATUM";
  dom.inspectorPart.textContent = component.name;
  dom.inspectorPartMeta.textContent = component.meta;
  dom.partState.textContent = executiveState;
  dom.poseHeading.textContent = state.mode === "wasm" ? "POSE ERROR · RUST CORE" : "POSE ERROR · UI PREVIEW";
  dom.poseX.textContent = signed(pose.translationMm[0], 3);
  dom.poseY.textContent = signed(pose.translationMm[1], 3);
  dom.poseZ.textContent = signed(pose.translationMm[2], 3);
  dom.rotX.textContent = signed(pose.rotationDeg[0], 2);
  dom.rotY.textContent = signed(pose.rotationDeg[1], 2);
  dom.rotZ.textContent = signed(pose.rotationDeg[2], 2);
  const sigmaMm = metrics.sigma / 1_000;
  for (const cell of [dom.sigmaX, dom.sigmaY, dom.sigmaZ]) cell.textContent = `±${sigmaMm.toFixed(3)}`;

  dom.guardLabel.textContent = phase.guard;
  let guardValue = "Executive gate evaluated by Rust core";
  if (state.mode !== "wasm") guardValue = "Preview threshold estimate";
  if (phase.guard.includes("FORCE")) guardValue = `Current ${metrics.force.toFixed(1)} mN`;
  else if (phase.guard.includes("CENTER") || phase.guard.includes("AXIS") || phase.guard.includes("DATUM")) {
    guardValue = `Current σ ${metrics.sigma.toFixed(1)} µm`;
  }
  dom.guardDetail.textContent = `${guardValue} · ${metrics.healthy ? "PASS" : "HOLD"}`;
  dom.guardCard.classList.toggle("warning", !metrics.healthy);
}

function updateUi() {
  const phase = phaseForProgress(state.progress);
  state.activeIndex = Math.max(0, sequence.findIndex((item) => item.key === phase.key));
  const metrics = metricSnapshot();
  const progressPercent = clamp(state.progress * 100, 0.4, 100);

  dom.timecode.textContent = formatTime(state.time);
  dom.transport.textContent = state.terminal
    ? `${state.completed ? "COMPLETED" : "STOPPED"} · CYCLE ${state.cycle}`
    : `${state.running ? "RUNNING" : "PAUSED"} · CYCLE ${state.cycle}`;
  dom.timelineFill.style.width = `${progressPercent}%`;
  dom.timelineCursor.style.left = `${progressPercent}%`;
  dom.speedLabel.textContent = state.speed.toFixed(1);

  dom.activePart.textContent = phase.part;
  dom.activeAction.textContent = phase.action;
  if (!state.inspectorPinned) updateInspector(phase, metrics);

  dom.healthTitle.textContent = metrics.healthy ? (state.completed ? "Accepted" : "Within guards") : "Guard hold";
  dom.healthScore.textContent = metrics.score;
  dom.healthRing.style.background = `conic-gradient(${metrics.healthy ? "var(--lime)" : "var(--orange)"} 0 ${metrics.score}%, var(--line) ${metrics.score}% 100%)`;
  dom.sigma.innerHTML = `${metrics.sigma.toFixed(1)} <small>µm</small>`;
  dom.force.innerHTML = `${metrics.force.toFixed(1)} <small>mN</small>`;
  dom.forceFill.style.width = `${clamp(metrics.force / 50 * 100, 1, 100)}%`;
  dom.forceFill.style.background = metrics.force > metrics.forceLimitMn ? "var(--red)" : metrics.force > metrics.forceLimitMn * 0.75 ? "var(--orange)" : "var(--lime)";
  dom.forceGateLabel.textContent = `${metrics.forceLimitMn} mN gate`;
  dom.clearance.innerHTML = `${metrics.clearance.toFixed(3)} <small>mm</small>`;
  dom.views.innerHTML = `${metrics.views} <small>/ 6</small>`;

  updateSequence(phase);
  state.sigmaHistory.push(metrics.sigma);
  state.sigmaHistory = state.sigmaHistory.slice(-34);
  drawSigmaChart();
}

function drawSigmaChart() {
  const canvas = dom.sigmaChart;
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (!rect.width || !rect.height) return;
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, rect.width, rect.height);

  const points = state.sigmaHistory;
  const min = 3;
  const max = 12;
  ctx.beginPath();
  points.forEach((value, index) => {
    const x = points.length === 1 ? 0 : index / (points.length - 1) * rect.width;
    const y = rect.height - (value - min) / (max - min) * rect.height;
    if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  const gradient = ctx.createLinearGradient(0, 0, rect.width, 0);
  gradient.addColorStop(0, "rgba(186,255,57,.25)");
  gradient.addColorStop(1, "#baff39");
  ctx.strokeStyle = gradient;
  ctx.lineWidth = 1.2;
  ctx.stroke();
}

function resizeCanvas() {
  const rect = dom.canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  state.dimensions = { width: rect.width, height: rect.height, dpr };
  dom.canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  dom.canvas.height = Math.max(1, Math.floor(rect.height * dpr));
}

function cameraProject(point) {
  const { width, height } = state.dimensions;
  const camera = state.camera;
  const viewPreset = state.view;
  let yaw = camera.yaw;
  let pitch = camera.pitch;
  if (viewPreset === "top") { yaw = 0; pitch = -Math.PI / 2 + 0.035; }

  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const rx = point.x * cy - point.y * sy;
  const ry = point.x * sy + point.y * cy;
  const py = ry * cp - point.z * sp;
  const depth = ry * sp + point.z * cp;
  const scale = Math.min(width / 8.5, height / 6.4) * camera.zoom;
  return {
    x: width * 0.5 + rx * scale,
    y: height * 0.5 + py * scale,
    depth,
    scale,
  };
}

function drawLine3(ctx, a, b, options = {}) {
  const pa = cameraProject(a);
  const pb = cameraProject(b);
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.strokeStyle = options.stroke ?? "rgba(127,145,136,.45)";
  ctx.lineWidth = options.width ?? 1;
  ctx.globalAlpha = options.alpha ?? 1;
  if (options.dash) ctx.setLineDash(options.dash);
  ctx.stroke();
  ctx.restore();
}

function ringPoints(x, radius, segments = 56) {
  return Array.from({ length: segments + 1 }, (_, index) => {
    const angle = index / segments * Math.PI * 2;
    return { x, y: Math.cos(angle) * radius, z: Math.sin(angle) * radius };
  });
}

function drawPolyline3(ctx, points, options = {}) {
  const projected = points.map(cameraProject);
  ctx.save();
  ctx.beginPath();
  projected.forEach((point, index) => index ? ctx.lineTo(point.x, point.y) : ctx.moveTo(point.x, point.y));
  ctx.strokeStyle = options.stroke ?? "rgba(127,145,136,.45)";
  ctx.lineWidth = options.width ?? 1;
  ctx.globalAlpha = options.alpha ?? 1;
  if (options.dash) ctx.setLineDash(options.dash);
  ctx.stroke();
  ctx.restore();
}

function drawPoint3(ctx, point, radius, fill, stroke = null) {
  const p = cameraProject(point);
  ctx.save();
  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = fill;
  ctx.fill();
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke(); }
  ctx.restore();
  return p;
}

function gearPath(ctx, x, y, radius, teeth, rotation = 0) {
  const inner = radius * 0.83;
  ctx.beginPath();
  for (let index = 0; index < teeth * 2; index++) {
    const angle = rotation + index / (teeth * 2) * Math.PI * 2;
    const r = index % 2 ? inner : radius;
    const px = x + Math.cos(angle) * r;
    const py = y + Math.sin(angle) * r;
    if (index === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  }
  ctx.closePath();
}

function drawGear2d(ctx, x, y, radius, teeth, rotation, active = false) {
  ctx.save();
  gearPath(ctx, x, y, radius, teeth, rotation);
  ctx.fillStyle = active ? "rgba(186,255,57,.13)" : "rgba(121,139,130,.07)";
  ctx.strokeStyle = active ? "rgba(186,255,57,.95)" : "rgba(136,153,145,.65)";
  ctx.lineWidth = active ? 1.6 : 1;
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(x, y, radius * .18, 0, Math.PI * 2);
  ctx.fillStyle = "#090d0c";
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function currentPhysicalScene() {
  // Operator rendering follows the estimator when one exists. Simulation
  // truth is a diagnostic fallback, never a source for browser-side motion.
  return state.sceneFrame?.estimate ?? state.sceneFrame?.truth ?? null;
}

function sceneDisplayScale() {
  const radius = Number(state.sceneDescription?.tube?.inner_radius_m);
  return Number.isFinite(radius) && radius > 0 ? 2.1 / radius : 1;
}

function scenePoint(translationM) {
  const values = Array.isArray(translationM) ? translationM.map(Number) : [];
  if (values.length !== 3 || values.some((value) => !Number.isFinite(value))) {
    return { x: 0, y: 0, z: 0 };
  }
  const scale = sceneDisplayScale();
  // Rust pipe_world uses +Z along the tube. The renderer uses +X along it.
  return { x: values[2] * scale, y: values[0] * scale, z: values[1] * scale };
}

function quaternionRotateVector(rotation, vector) {
  if (!Array.isArray(rotation) || rotation.length !== 4) return vector;
  const [x, y, z, w] = rotation.map(Number);
  const [vx, vy, vz] = vector;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ];
}

function quaternionZAngle(rotation) {
  if (!Array.isArray(rotation) || rotation.length !== 4) return 0;
  const [x, y, z, w] = rotation.map(Number);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function bodyDescriptions() {
  return new Map((state.sceneDescription?.rigid_bodies ?? []).map((body) => [Number(body.id), body]));
}

function shapeRadiusM(shape) {
  if (!shape) return 0.0001;
  if (shape.kind === "sphere") return Number(shape.radius_m) || 0.0001;
  if (shape.kind === "capsule") {
    return (Number(shape.radius_m) || 0) + (Number(shape.half_segment_m) || 0);
  }
  if (shape.kind === "gear") return Number(shape.tip_radius_m) || 0.0001;
  if (shape.kind === "box") {
    return Math.hypot(...(shape.half_extents_m ?? [0.0001, 0.0001, 0.0001]).map(Number));
  }
  return 0.0001;
}

function activeSceneBody(scene) {
  return scene.rigid_bodies.find((body) => Number(body.id) === Number(state.componentId))
    ?? scene.rigid_bodies.find((body) => body.enabled)
    ?? null;
}

function drawUnavailableScene(ctx) {
  const { width, height } = state.dimensions;
  ctx.save();
  ctx.textAlign = "center";
  ctx.fillStyle = "rgba(135,153,144,.8)";
  ctx.font = `10px ${getComputedStyle(document.documentElement).getPropertyValue("--mono")}`;
  ctx.fillText("RUST MACHINE SCENE UNAVAILABLE", width * .5, height * .49);
  ctx.fillStyle = "rgba(100,117,109,.62)";
  ctx.fillText("BUILD THE WASM CORE TO RENDER PHYSICAL STATE", width * .5, height * .54);
  ctx.restore();
  dom.callout.style.opacity = "0";
}

function drawCollider(ctx, collider) {
  const shape = collider?.shape;
  const pose = collider?.pose;
  if (!shape || !pose || shape.kind !== "capsule") return;
  const half = Number(shape.half_segment_m) || 0;
  const radius = Number(shape.radius_m) || 0;
  const axis = quaternionRotateVector(pose.rotation_xyzw, [0, 0, half]);
  const center = pose.translation_m.map(Number);
  const a = scenePoint(center.map((value, index) => value - axis[index]));
  const b = scenePoint(center.map((value, index) => value + axis[index]));
  const projected = cameraProject(scenePoint(center));
  drawLine3(ctx, a, b, {
    stroke: "rgba(255,154,71,.24)",
    width: Math.max(1, radius * sceneDisplayScale() * projected.scale * 2),
    dash: [3, 3],
  });
}

function drawRigidBody3(ctx, body, description, active) {
  if (!body.enabled && !active) return;
  const shape = description?.shape;
  const point = scenePoint(body.pose?.translation_m);
  const projected = cameraProject(point);
  const pixelScale = sceneDisplayScale() * projected.scale;
  if (shape?.kind === "gear") {
    drawGear2d(
      ctx,
      projected.x,
      projected.y,
      Math.max(3, Number(shape.tip_radius_m) * pixelScale),
      Number(shape.teeth),
      quaternionZAngle(body.pose?.rotation_xyzw),
      active,
    );
    return;
  }
  if (shape?.kind === "capsule") {
    const half = Number(shape.half_segment_m) || 0;
    const axis = quaternionRotateVector(body.pose?.rotation_xyzw, [0, 0, half]);
    const center = body.pose.translation_m.map(Number);
    const a = scenePoint(center.map((value, index) => value - axis[index]));
    const b = scenePoint(center.map((value, index) => value + axis[index]));
    drawLine3(ctx, a, b, {
      stroke: active ? "#baff39" : "rgba(173,190,181,.7)",
      width: Math.max(1, Number(shape.radius_m) * pixelScale * 2),
    });
    return;
  }
  const radius = Math.max(2, shapeRadiusM(shape) * pixelScale);
  ctx.save();
  ctx.translate(projected.x, projected.y);
  ctx.rotate(quaternionZAngle(body.pose?.rotation_xyzw));
  ctx.fillStyle = active ? "rgba(186,255,57,.12)" : "rgba(121,139,130,.05)";
  ctx.strokeStyle = active ? "rgba(186,255,57,.9)" : "rgba(136,153,145,.52)";
  if (shape?.kind === "box") {
    const extents = shape.half_extents_m.map((value) => Number(value) * pixelScale);
    ctx.fillRect(-extents[0], -extents[1], extents[0] * 2, extents[1] * 2);
    ctx.strokeRect(-extents[0], -extents[1], extents[0] * 2, extents[1] * 2);
  } else {
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

function drawTubeScene(ctx) {
  const scene = currentPhysicalScene();
  const description = state.sceneDescription;
  if (!scene || !description) {
    drawUnavailableScene(ctx);
    return;
  }

  const scale = sceneDisplayScale();
  const radius = description.tube.inner_radius_m * scale;
  const end = description.tube.working_length_m * scale * .5;
  const ringXs = Array.from({ length: 7 }, (_, index) => -end + index * end / 3);
  for (const x of ringXs) {
    const boundary = Math.abs(Math.abs(x) - end) < 1e-9;
    drawPolyline3(ctx, ringPoints(x, radius), {
      stroke: boundary ? "rgba(124,143,134,.56)" : "rgba(91,109,100,.16)",
      width: boundary ? 1.3 : .7,
    });
  }
  drawPolyline3(ctx, ringPoints(0, description.tube.central_work_radius_m * scale), {
    stroke: "rgba(186,255,57,.22)", width: 1, dash: [3, 4],
  });

  for (const arm of scene.manipulators) {
    const angle = Number(arm.carriage_theta_rad);
    const railRadius = description.rail_system.shoulder_datum_radius_m * scale;
    const a = { x: -end, y: Math.cos(angle) * railRadius, z: Math.sin(angle) * railRadius };
    const b = { x: end, y: Math.cos(angle) * railRadius, z: Math.sin(angle) * railRadius };
    drawLine3(ctx, a, b, { stroke: "rgba(150,169,159,.52)", width: 2.5 });
  }

  const targetBody = activeSceneBody(scene);
  const target = targetBody ? scenePoint(targetBody.pose.translation_m) : { x: 0, y: 0, z: 0 };
  if (state.layers.sensors) {
    for (const sensor of description.sensors.filter((item) => item.kind.includes("camera"))) {
      const camera = scenePoint(sensor.nominal_translation_m);
      drawPoint3(ctx, camera, 3.2, "#48d9ff", "rgba(72,217,255,.55)");
      drawLine3(ctx, camera, target, {
        stroke: "rgba(72,217,255,.11)", width: .7, dash: [2, 5],
      });
    }
  }

  const motionScore = (arm) => Math.abs(Number(arm.carriage_z_velocity_m_s))
    + Math.abs(Number(arm.carriage_theta_velocity_rad_s)) * .01
    + (arm.joint_velocities_rad_s ?? [])
      .reduce((sum, value) => sum + Math.abs(Number(value)), 0) * .01;
  const activeArm = scene.manipulators.reduce(
    (best, arm) => motionScore(arm) > motionScore(best) ? arm : best,
    scene.manipulators[0],
  );
  for (const arm of scene.manipulators) {
    const active = activeArm && arm.id === activeArm.id && motionScore(arm) > 1e-8;
    const color = active ? "rgba(186,255,57,.9)" : "rgba(133,151,142,.62)";
    const points = [arm.shoulder_pose, arm.elbow_pose, arm.wrist_pose, arm.tool_pose]
      .map((pose) => scenePoint(pose.translation_m));
    if (active) {
      for (let index = 0; index < points.length - 1; index++) {
        drawLine3(ctx, points[index], points[index + 1], {
          stroke: "rgba(186,255,57,.18)", width: 8,
        });
      }
    }
    for (let index = 0; index < points.length - 1; index++) {
      drawLine3(ctx, points[index], points[index + 1], {
        stroke: color, width: index === 2 ? 2.5 : 3.2,
      });
    }
    drawPoint3(ctx, scenePoint(arm.carriage_pose.translation_m), 5, "#101714", color);
    points.forEach((point, index) => drawPoint3(
      ctx,
      point,
      index === points.length - 1 ? 2.8 : 3.5,
      "#0b100e",
      color,
    ));
    for (const jaw of arm.gripper?.jaw_poses ?? []) {
      drawPoint3(ctx, scenePoint(jaw.translation_m), 2, "#baff39");
    }
    if (state.layers.collision) {
      for (const collider of arm.link_colliders ?? []) drawCollider(ctx, collider);
    }
  }

  const descriptions = bodyDescriptions();
  for (const body of scene.rigid_bodies) {
    drawRigidBody3(
      ctx,
      body,
      descriptions.get(Number(body.id)),
      Number(body.id) === Number(state.componentId),
    );
  }

  if (state.layers.uncertainty) {
    const p = cameraProject(target);
    ctx.save();
    ctx.beginPath();
    ctx.ellipse(p.x, p.y, 20, 11, -.2, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(185,140,255,.07)";
    ctx.strokeStyle = "rgba(185,140,255,.7)";
    ctx.setLineDash([3, 3]);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  const calloutPoint = cameraProject(target);
  dom.callout.style.left = `${clamp(calloutPoint.x / state.dimensions.width * 100, 12, 72)}%`;
  dom.callout.style.top = `${clamp(calloutPoint.y / state.dimensions.height * 100, 18, 76)}%`;
  dom.callout.style.opacity = "1";
}

function drawMacroScene(ctx) {
  const scene = currentPhysicalScene();
  const descriptions = bodyDescriptions();
  if (!scene || !state.sceneDescription) {
    drawUnavailableScene(ctx);
    return;
  }
  const bodies = scene.rigid_bodies
    .filter((body) => body.enabled || Number(body.id) === Number(state.componentId));
  if (!bodies.length) {
    drawUnavailableScene(ctx);
    return;
  }
  const bounds = bodies.reduce((result, body) => {
    const shape = descriptions.get(Number(body.id))?.shape;
    const radius = shapeRadiusM(shape);
    const [x, y] = body.pose.translation_m.map(Number);
    return {
      minX: Math.min(result.minX, x - radius),
      maxX: Math.max(result.maxX, x + radius),
      minY: Math.min(result.minY, y - radius),
      maxY: Math.max(result.maxY, y + radius),
    };
  }, { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity });
  const { width, height } = state.dimensions;
  const centerX = (bounds.minX + bounds.maxX) * .5;
  const centerY = (bounds.minY + bounds.maxY) * .5;
  const spanX = Math.max(bounds.maxX - bounds.minX, 7e-3);
  const spanY = Math.max(bounds.maxY - bounds.minY, 5e-3);
  const scale = Math.min(width * .82 / spanX, height * .78 / spanY) * state.camera.zoom;
  const project = (translation) => ({
    x: width * .5 + (Number(translation[0]) - centerX) * scale,
    y: height * .52 - (Number(translation[1]) - centerY) * scale,
  });

  for (const body of bodies) {
    const description = descriptions.get(Number(body.id));
    const shape = description?.shape;
    const point = project(body.pose.translation_m);
    const active = Number(body.id) === Number(state.componentId);
    if (shape?.kind === "gear") {
      drawGear2d(
        ctx,
        point.x,
        point.y,
        Math.max(4, Number(shape.tip_radius_m) * scale),
        Number(shape.teeth),
        quaternionZAngle(body.pose.rotation_xyzw),
        active,
      );
      continue;
    }
    ctx.save();
    ctx.translate(point.x, point.y);
    ctx.rotate(-quaternionZAngle(body.pose.rotation_xyzw));
    ctx.fillStyle = active ? "rgba(186,255,57,.10)" : "rgba(91,111,101,.05)";
    ctx.strokeStyle = active ? "rgba(186,255,57,.9)" : "rgba(115,136,126,.62)";
    if (shape?.kind === "box") {
      const [hx, hy] = shape.half_extents_m.map((value) => Number(value) * scale);
      ctx.fillRect(-hx, -hy, hx * 2, hy * 2);
      ctx.strokeRect(-hx, -hy, hx * 2, hy * 2);
    } else {
      const radius = Math.max(2, shapeRadiusM(shape) * scale);
      ctx.beginPath();
      ctx.arc(0, 0, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
    ctx.restore();
  }
  dom.callout.style.opacity = "0";
}

function renderCanvas() {
  const { width, height, dpr } = state.dimensions;
  if (!width || !height) return;
  const ctx = dom.canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  state.camera.yaw = lerp(state.camera.yaw, state.camera.targetYaw, .12);
  state.camera.pitch = lerp(state.camera.pitch, state.camera.targetPitch, .12);
  state.camera.zoom = lerp(state.camera.zoom, state.camera.targetZoom, .12);

  const vignette = ctx.createRadialGradient(width * .5, height * .48, 0, width * .5, height * .48, Math.max(width, height) * .62);
  vignette.addColorStop(0, "rgba(39,58,50,.12)");
  vignette.addColorStop(1, "rgba(0,0,0,.25)");
  ctx.fillStyle = vignette;
  ctx.fillRect(0, 0, width, height);

  if (state.view === "macro") drawMacroScene(ctx);
  else drawTubeScene(ctx);
}

function animationLoop(timestamp) {
  const delta = Math.min(50, timestamp - state.lastTimestamp);
  state.lastTimestamp = timestamp;

  if (state.running) {
    state.accumulator += delta * state.speed;
    const cycleMs = state.mode === "wasm" ? 20 : 22;
    let iterations = 0;
    while (state.accumulator >= cycleMs && iterations < 12) {
      stepSimulation();
      state.accumulator -= cycleMs;
      iterations += 1;
    }
  }

  renderCanvas();
  requestAnimationFrame(animationLoop);
}

function setView(view) {
  state.view = view;
  $$(".view-tab").forEach((button) => button.classList.toggle("active", button.dataset.view === view));
  if (view === "iso") {
    state.camera.targetYaw = -.08;
    state.camera.targetPitch = -.15;
    state.camera.targetZoom = 1;
  } else if (view === "top") {
    state.camera.targetYaw = 0;
    state.camera.targetPitch = -Math.PI / 2 + .035;
    state.camera.targetZoom = .95;
  } else {
    state.camera.targetZoom = 1;
  }
  $("#view-hint").style.opacity = view === "macro" ? ".35" : ".75";
}

function bindInteractions() {
  dom.run.addEventListener("click", toggleRun);
  dom.step.addEventListener("click", () => {
    if (state.terminal) initializeSimulator({ preservePosition: false });
    else { stopRun(); stepSimulation(); }
  });
  dom.reset.addEventListener("click", () => initializeSimulator({ preservePosition: false }));
  dom.scenario.addEventListener("change", () => {
    initializeSimulator({ preservePosition: false });
    toast(`Scenario changed to ${dom.scenario.value}`);
  });
  dom.speed.addEventListener("change", () => {
    state.speed = Number(dom.speed.value);
    updateUi();
  });

  $$(".view-tab").forEach((button) => button.addEventListener("click", () => setView(button.dataset.view)));
  $$(".tool-toggle").forEach((button) => button.addEventListener("click", () => {
    const layer = button.dataset.layer;
    state.layers[layer] = !state.layers[layer];
    button.classList.toggle("active", state.layers[layer]);
    button.setAttribute("aria-pressed", String(state.layers[layer]));
  }));

  $("#fit-view").addEventListener("click", () => setView(state.view));
  dom.pin.addEventListener("click", () => {
    state.inspectorPinned = !state.inspectorPinned;
    dom.pin.textContent = state.inspectorPinned ? "PINNED" : "PIN";
    dom.pin.setAttribute("aria-pressed", String(state.inspectorPinned));
    if (!state.inspectorPinned) updateUi();
    toast(state.inspectorPinned ? "Inspector pinned to active component" : "Inspector follows task state");
  });

  dom.export.addEventListener("click", () => {
    let payload;
    if (state.mode === "wasm" && state.simulator) {
      try { payload = JSON.parse(state.simulator.reportJson(true)); }
      catch { payload = state.rawSnapshot; }
    }
    payload ??= {
      schema: "the-pipe-ui-preview-v1",
      warning: "Visualization-only preview; this is not a Rust acceptance report.",
      scenario: state.scenario,
      cycle: state.cycle,
      sim_time_s: state.time,
      progress: state.progress,
      terminal: state.terminal,
      completed: state.completed,
      events: state.events,
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `the-pipe-${state.scenario}-report.json`;
    anchor.click();
    URL.revokeObjectURL(url);
    toast(state.mode === "wasm" ? "Rust report exported" : "UI preview snapshot exported");
  });

  dom.canvasWrap.addEventListener("pointerdown", (event) => {
    if (state.view === "top") return;
    dom.canvasWrap.setPointerCapture(event.pointerId);
    dom.canvasWrap.classList.add("dragging");
    state.drag = { x: event.clientX, y: event.clientY, yaw: state.camera.targetYaw, pitch: state.camera.targetPitch };
  });
  dom.canvasWrap.addEventListener("pointermove", (event) => {
    if (!state.drag) return;
    state.camera.targetYaw = state.drag.yaw + (event.clientX - state.drag.x) * .006;
    state.camera.targetPitch = clamp(state.drag.pitch + (event.clientY - state.drag.y) * .004, -.85, .65);
  });
  const endDrag = () => { state.drag = null; dom.canvasWrap.classList.remove("dragging"); };
  dom.canvasWrap.addEventListener("pointerup", endDrag);
  dom.canvasWrap.addEventListener("pointercancel", endDrag);
  dom.canvasWrap.addEventListener("wheel", (event) => {
    event.preventDefault();
    state.camera.targetZoom = clamp(state.camera.targetZoom * (event.deltaY > 0 ? .92 : 1.08), .55, 1.9);
  }, { passive: false });

  document.addEventListener("keydown", (event) => {
    if (event.target.matches("select, input, textarea")) return;
    if (event.code === "Space") { event.preventDefault(); toggleRun(); }
    else if (event.code === "ArrowRight") { event.preventDefault(); stopRun(); stepSimulation(); }
    else if (event.key.toLowerCase() === "r") initializeSimulator({ preservePosition: false });
    else if (["1", "2", "3"].includes(event.key)) setView({ "1": "iso", "2": "top", "3": "macro" }[event.key]);
  });
}

function bootstrap() {
  bindInteractions();
  new ResizeObserver(() => { resizeCanvas(); drawSigmaChart(); }).observe(dom.canvasWrap);
  resizeCanvas();
  renderEvents();
  updateUi();
  requestAnimationFrame(animationLoop);
  connectWasm();
}

bootstrap();
