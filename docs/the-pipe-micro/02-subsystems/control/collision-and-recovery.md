# Collision, disturbance and recovery

Small distal mass is helpful, but damage is governed by more than moving mass. External motors can continue pulling, tendons and flexures store energy, a sharp asperity can concentrate stress, and a small part can be disturbed by a hose or gas pulse.

## What belongs in the physical scene

The relevant scene contains arm bodies, deformed flexures, open and closed jaws, payloads, support booms, fixtures, carriages, windows and service bundles. A tool envelope changes with jaw opening and pressure or tendon-induced deformation. Uncertainty in an observed pose also affects the usable clearance.

A deliberately contacting tool needs different treatment from a body that should never touch the part. Guarded contact is an intended interaction under bounded conditions; it is not a reason to omit collision constraints elsewhere.

## What stopping means

An operation may need to stop because observation is lost, tension departs from its useful range, motion disagrees with the model, pressure changes unexpectedly or a part starts slipping. There is no universal instruction to release all tendons. The suitable response depends on which interfaces currently retain the part and how the mechanism behaves when unloaded.

A stable retained state can allow re-observation. A sustained damaging load may require controlled unloading. A tool with a qualified passive holding feature can behave differently from a normally open tweezer. These behaviours are properties of the tool and operation together.

## Honest demonstrations

Any later animation should be generated from the model or measured operation being described. It should expose meaningful failures such as residual attachment or lost observation where those are modelled. A rendered gap is only evidence of physical release if the underlying model represents the relevant interface physics.

This document does not add a software subsystem or prescribe a collision engine. It states the physical coverage that a credible assembly demonstration eventually needs.

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
