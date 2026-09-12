# Remote tendon actuation

**Chosen direction:** accessible external motors drive lightweight distal mechanisms through tendons.

The transmission can place the motor, capstan and optional tension sensing outside the manipulation volume. The arm then carries fibre and printed geometry. Candidate tendon materials include polymer fibres such as UHMWPE, but no fibre type or diameter is selected by this atlas.

## What the tendon transmits

A tendon pulls; it does not push. One tendon can work against a flexure's elastic return. An antagonistic pair can maintain engagement on both sides of a motion. More complex routing can distribute effort across several joints, with coupling that must be understood.

For an ideal tendon acting at moment arm r, joint torque is approximately τ = Fr. For a simple elastic fibre, stretch is approximately ΔL = FL/(EA). These relations explain two architectural tensions: a small joint lever reduces the length change needed for an angle, but raises the tendon force needed for a given torque; a long remote transmission can contribute appreciable stretch even if the distal arm is small.

## Motor resolution versus real motion

At a capstan, ideal paid-out length is Δs = r_c Δφ. A smaller capstan gives less tendon motion per motor-angle increment, but also changes bend radius, available torque-to-tension conversion and winding behaviour. Command quantisation is only one part of the error budget. Slip, elastic stretch, hysteresis, preload and routing friction sit between the command and the tool.

The optical loop should observe the resulting motion. That observation complements mechanical characterisation; it cannot instantly cancel high-frequency disturbances or recover motion beyond the available travel.

## Sensing location

External tension sensing can help detect slack or overload. It measures the transmission at its mounting point. Guide friction and the flexure's restoring force mean it is not automatically a direct measurement of tool contact force. A local optically observed compliant element offers a separate candidate measurement route.

See [routing and preload](routing-and-preload.md) for the main mechanical coupling, and [optical sensing opportunities](../../04-2pp-opportunities/optical-and-mechanical-sensing.md) for printed force-indicating features.

## Browse this folder

- [Routing, preload and service motion](routing-and-preload.md)

---

[Start](../../README.md) · [Parent folder](../README.md) · [Complete directory](../../DIRECTORY.md)
