# Near-wall transfer candidate v1

Selected for a **synthetic software experiment**, not a qualified physical design.
The shared machine definition is `designs/pipe_micro/near_wall_v1/machine.json`;
operation limits are `scenarios/pipe_micro/native_resin_transfer_v1.json`.

Compare three layouts: axis station with wall shoulders is unreachable with 12 mm
reach inside a 50 mm radius; inward supports could reach but add occlusion and service
obstructions; a station at x=40 mm permits two shoulders at y=±8 mm with short tools.
Choose the last for this experiment. Neither 100 mm enclosure diameter nor 12 mm reach
becomes a universal product requirement. Both are engineering candidate values.

A shared ring carries both shoulders. It executes bounded z and θ positioning before
pickup; independent motion, passing and continuous rotation are rejected. Tendon
service travel budgets are explicit. The ring returns to the selected station before
loaded manipulation. This is not yet a designed transport mechanism or routed cable assembly.

The local mechanism is modeled as bounded translation with an assumed linear restoring
stiffness. Two monolithic compliant tools grasp separate x regions of a 500×300×200 µm
coupon. Contacts close along z, tools travel in y; the coupon's orientation is constrained
and **not estimated**. A change in orientation invalidates the operation. This is not a
six-axis arm model. Palm, fingers, pads, shoulder, flexure envelopes, fixtures, part and
service corridors share one geometry generator across runtime, optical rays and CAD.

The 4 mm beam, 40 µm thickness, 200 µm width and 1 mm displacement envelope are a proposed
coupon domain. The simple `3*t*travel/L²` strain bound and 0.02 N/m stiffness are assumed
screening quantities, not derived material performance or a validated mechanism. Pretension,
retention and pull-off bounds are exploratory synthetic loads, independent of old force
constants. The rigidly retained object follows its supporting tool within this explicitly
reduced model; dual support blocks loaded movement until donor separation.

The synthetic optics uses two view positions, ray/box occlusion and independent centroid
noise. The 1 µm synthetic standard deviation exercises uncertainty gates; it is not a
prediction of achievable camera accuracy. The 10 µm position tolerance and 30 µm gap
threshold are selected to exceed this synthetic uncertainty. The full camera, marker,
window and orientation design remains open. Retention evidence is generated from the
synthetic contact state, not inferred from a real camera or force sensor.

R1 is an **unselected** structural-resin process identity, with no batch or calibrated
property. Choosing one actual resin/process and native surface is required before
fabrication; the software must not invent that evidence. The specified temperature/RH
are recorded exploratory conditions, not a material recipe. Transfer port and service
interfaces reserve access but do not constitute a manufacturable sealed assembly.

Next physical evidence: choose resin/process and specimen identity; test enlarged
routing/assembly geometry and actual-scale beam travel, load response and native-surface
pickup/release. Compare smooth and one geometry variant. Record preload, dwell, path,
speed, humidity, temperature, process and failure class. Independently verify final part
pose and donor separation. These experiments may reject this candidate.
