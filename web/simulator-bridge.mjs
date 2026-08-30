const EXECUTIVE_PHASE_FRACTION = Object.freeze({
  locate: 0.05,
  pick: 0.22,
  handoff: 0.38,
  align: 0.5,
  insert: 0.7,
  mesh: 0.84,
  verify: 0.96,
});

const COMPONENT_PROGRESS = Object.freeze({
  4: [0.18, 0.36],
  5: [0.36, 0.52],
  // The final gear's verification phase drives the high-level rotation-test stage.
  6: [0.52, 0.75],
});

const COVER_PROGRESS = Object.freeze({
  locate: 0.755,
  pick: 0.78,
  handoff: 0.82,
  align: 0.85,
  insert: 0.89,
  mesh: 0.91,
  verify: 0.95,
});

function finiteNumber(value) {
  if (value === null || value === "" || typeof value === "boolean") return undefined;
  const number = Number(value);
  return Number.isFinite(number) ? number : undefined;
}

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function progressForSnapshot(snapshot) {
  const componentId = finiteNumber(snapshot?.component_id);
  const phase = String(snapshot?.phase ?? "").toLowerCase();
  const phaseFraction = EXECUTIVE_PHASE_FRACTION[phase] ?? 0;

  if (componentId >= 1 && componentId <= 3) {
    const shaftSpan = 0.10 / 3;
    const start = 0.08 + (componentId - 1) * shaftSpan;
    return start + shaftSpan * phaseFraction;
  }

  if (COMPONENT_PROGRESS[componentId]) {
    const [start, end] = COMPONENT_PROGRESS[componentId];
    return start + (end - start) * phaseFraction;
  }

  if (componentId === 7) return COVER_PROGRESS[phase] ?? 0.75;
  return 0.08;
}

function telemetryFromSnapshot(snapshot) {
  const optical = snapshot?.optical ?? {};
  const mechanics = snapshot?.mechanics ?? {};
  const axialForce = finiteNumber(mechanics.active_axial_force_n) ?? 0;
  const lateralForce = finiteNumber(mechanics.active_lateral_force_n) ?? 0;

  return {
    sigmaUm: (finiteNumber(optical.position_sigma_mm) ?? 0) * 1_000,
    forceMn: Math.max(Math.abs(axialForce), Math.abs(lateralForce)) * 1_000,
    clearanceMm: finiteNumber(mechanics.min_unplanned_clearance_mm) ?? 0,
    views: finiteNumber(optical.valid_camera_views) ?? 0,
  };
}

function poseErrorFromSnapshot(snapshot) {
  const translation = snapshot?.pose_error?.translation_mm;
  const rotation = snapshot?.pose_error?.rotation_deg;
  if (!Array.isArray(translation) || !Array.isArray(rotation)) return null;

  const translationMm = translation.map(finiteNumber);
  const rotationDeg = rotation.map(finiteNumber);
  if (translationMm.some((value) => value === undefined) || rotationDeg.some((value) => value === undefined)) return null;
  return { translationMm, rotationDeg };
}

export function interpretSnapshot(snapshot, previousProgress = 0) {
  const status = String(snapshot?.status ?? "running").toLowerCase();
  const terminal = status === "completed" || status === "aborted";
  const completed = status === "completed";
  const derivedProgress = completed ? 1 : progressForSnapshot(snapshot);
  const controlTimeMs = finiteNumber(snapshot?.control_time_ms);
  const cycle = finiteNumber(snapshot?.cycle);

  return {
    cycle,
    timeSeconds: controlTimeMs === undefined ? undefined : controlTimeMs / 1_000,
    componentId: finiteNumber(snapshot?.component_id),
    componentName: snapshot?.component_name ?? null,
    executivePhase: snapshot?.phase ?? null,
    status,
    terminal,
    completed,
    progress: clamp(Math.max(previousProgress, derivedProgress), 0, 1),
    telemetry: telemetryFromSnapshot(snapshot),
    poseError: poseErrorFromSnapshot(snapshot),
    events: Array.isArray(snapshot?.events) ? snapshot.events.map(String) : [],
    injectedFault: snapshot?.injected_fault ?? null,
  };
}
