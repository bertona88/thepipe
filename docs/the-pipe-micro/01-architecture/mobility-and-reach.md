# Mobility and reach

## What z and θ actually mean

For an independently positioned base i, write q_base,i = (z_i, θ_i). z_i selects longitudinal location; θ_i selects azimuth around the cylinder. At radius r, circumferential travel is s = rθ when θ is measured in radians.

Moving around the cylinder usually changes the shoulder's position and may also rotate its mounted frame with the supporting surface. This is not automatically an independently controllable tool-roll axis. The mount geometry determines the frame transform.

The phrase “shared two-degree-of-freedom macro joint” describes a common positioning concept. It does not establish independent actuation for every arm. If several arms ride one translating ring, they share its z motion unless another mechanism gives them independent travel.

## Why base mobility is useful

The base can select a favourable approach direction, move an arm between stations, park a tool, or bring local workspaces into overlap. The distal mechanism can then spend its limited travel on alignment and interaction. A carriage only needs repeatability and stiffness appropriate to finding and maintaining the observed operating neighbourhood.

Its error still matters. The arm has finite correction travel. Sudden slip can exceed optical feedback bandwidth. Compliance or friction at the base can move the tool during contact. External observation relaxes absolute placement requirements; it does not remove mechanical constraints.

## Local orientation and manipulation

A task requires more than reaching a point. A gripper may need a jaw approach direction, an allowable peel direction and clearance for withdrawal. The required pose set should be described for the operation. A low-degree-of-freedom flexure arm may achieve useful assembly through coordinated base motion, shaped fixtures and regrasping by other arms.

The useful reach of an arm is therefore a set of allowable tool poses, not just a sphere. The cooperative workspace is where multiple such sets overlap while the scene remains observable and the service bundles remain feasible.

## Range and wrap

Full circumferential travel is an aspiration that competes with tendon and fluid routing. Finite θ travel with an unwind path may be simpler than continuous rotation. Multiple bases on one track may be unable to pass each other. These constraints belong to the concept of transport itself and are explored in [mechanism options](../02-subsystems/transport/mechanism-options.md).

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
