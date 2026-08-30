import assert from "node:assert/strict";
import test from "node:test";

import { interpretSnapshot } from "./simulator-bridge.mjs";

function snapshot(overrides = {}) {
  return {
    cycle: 12,
    control_time_ms: 240,
    component_id: 4,
    component_name: "G3 24 tooth output gear",
    phase: "align",
    status: "running",
    optical: { position_sigma_mm: 0.0062, valid_camera_views: 5 },
    mechanics: {
      min_unplanned_clearance_mm: 0.187,
      active_axial_force_n: 0.0031,
      active_lateral_force_n: 0.0004,
    },
    pose_error: {
      translation_mm: [0.001, -0.002, 0.003],
      rotation_deg: [0.06, -0.11, 14.8],
    },
    events: ["transition:align->insert"],
    injected_fault: null,
    ...overrides,
  };
}

test("maps real simulator clock and telemetry units", () => {
  const result = interpretSnapshot(snapshot());
  assert.equal(result.cycle, 12);
  assert.equal(result.timeSeconds, 0.24);
  assert.deepEqual(result.telemetry, {
    sigmaUm: 6.2,
    forceMn: 3.1,
    clearanceMm: 0.187,
    views: 5,
  });
  assert.deepEqual(result.poseError.translationMm, [0.001, -0.002, 0.003]);
});

test("maps components and executive phases onto the high-level timeline", () => {
  assert.ok(interpretSnapshot(snapshot({ component_id: 1, phase: "locate" })).progress >= 0.08);

  const output = interpretSnapshot(snapshot({ component_id: 4, phase: "align" })).progress;
  assert.ok(output >= 0.18 && output < 0.36);

  const rotationTest = interpretSnapshot(snapshot({ component_id: 6, phase: "verify" })).progress;
  assert.ok(rotationTest >= 0.65 && rotationTest < 0.75);

  const handoff = interpretSnapshot(snapshot({ component_id: 7, phase: "handoff" })).progress;
  const closure = interpretSnapshot(snapshot({ component_id: 7, phase: "insert" })).progress;
  assert.ok(handoff >= 0.75 && handoff < 0.84);
  assert.ok(closure >= 0.84 && closure < 0.92);
});

test("keeps timeline progress monotonic across executive retries", () => {
  const result = interpretSnapshot(snapshot({ component_id: 4, phase: "locate" }), 0.34);
  assert.equal(result.progress, 0.34);
});

test("derives terminal state from the serialized status field", () => {
  const completed = interpretSnapshot(snapshot({ status: "completed", component_id: null }));
  assert.equal(completed.terminal, true);
  assert.equal(completed.completed, true);
  assert.equal(completed.progress, 1);
  assert.equal(completed.componentId, undefined);

  const aborted = interpretSnapshot(snapshot({ status: "aborted" }), 0.47);
  assert.equal(aborted.terminal, true);
  assert.equal(aborted.completed, false);
  assert.equal(aborted.progress, 0.47);
});

test("preserves simulator events and injected-fault identity", () => {
  const result = interpretSnapshot(snapshot({
    events: ["gate-rejected:vision-confidence-low"],
    injected_fault: "occlusion",
  }));
  assert.deepEqual(result.events, ["gate-rejected:vision-confidence-low"]);
  assert.equal(result.injectedFault, "occlusion");
});
