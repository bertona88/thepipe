# Design principles

## 1. Scale each part for its job

Small tools belong near small parts. Large optical baselines, accessible motors and maintainable fluid hardware can remain outside. An enclosure dimension, a transport radius, an arm reach and an assembly volume are separate choices. The [scale and workspace document](../01-architecture/scale-and-workspace.md) makes those distinctions explicit.

## 2. Let geometry earn its place

Flexure joints, tendon seats, contact pads, markers and accessible channels can be designed together. Begin with one structural resin and native surfaces. Geometry is the first experimental variable; chemistry and additional materials remain available when a specific limitation justifies them.

## 3. Keep actuation accessible

The working volume should contain as little actuator mass and wiring as practical. Tendons carry mechanical effort inward; fluid lines can carry pressure or liquid. The external equipment remains serviceable. Remote actuation still has transmission stiffness, friction and stored energy, so it does not make the distal mechanism force-free.

## 4. Observe the interaction

Optics supplies geometric information about actual tools and objects. Motor commands are not absolute position measurements. The design should expose useful features at the tip, the part and the destination, with enough viewpoint diversity to understand contact and separation.

## 5. Use compliance deliberately

Compliance can enable motion, accommodate small alignment errors and expose force through deformation. Its direction and magnitude matter. A flexible arm that bends unpredictably under tendon preload is different from a designed compliant insertion tool.

## 6. Make release a cooperative operation

Do not use falling under gravity as the mechanism for transferring responsibility for a component. Supply receiving support, change the donor interface and verify separation. Gravity remains a real disturbance in the physical model.

## 7. Treat the environment as part of the machine

Humidity, temperature, gas flow, contamination and charge history affect assembly. A sealed chamber and transfer locks provide a way to control conditions. A humidity setting is not a measurement of surface charge, and a closed lid is not proof of cleanliness.

## 8. Keep mechanisms open until their purpose selects them

Longitudinal and circumferential positioning are desired capabilities. A literal rail is one way to obtain them. A serial flexure chain is one arm family. A tweezer is one tool family. Their roles are firmer than their implementation details.

## 9. Keep imagination and evidence connected

The [opportunity atlas](../04-2pp-opportunities/README.md) deliberately contains ambitious ideas. Each idea identifies the geometric freedom that enables it and a physical question that could defeat it. The [evidence documents](../05-evidence/README.md) explain how a concept becomes a bounded claim without turning this atlas into a delivery plan.

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
