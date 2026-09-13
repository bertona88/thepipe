# Routing, preload and service motion

## Preload is a real load

Preload can keep tendons seated and remove slack, but it also stresses anchors, bends compliant structures and loads the base. Shrinking arm mass does not automatically reduce the preload necessary for the chosen transmission.

The useful preload range lies between insufficient engagement and excessive structural distortion or fatigue. It changes with posture, route length, temperature, creep and tool loading. A single nominal tension does not describe a full motion cycle.

## Guides and friction

Printed eyes, grooves and guide surfaces can route fibres through a compact mechanism. They can also produce abrasion, stiction and direction-dependent tension transmission. Tight turns raise local contact stress. Low friction is desirable, but a chosen polymer/fibre pair needs evidence in its actual print and finish condition.

Routing near a joint's rotation region may reduce length coupling; routing across other moving joints can make one command influence several degrees of freedom. This coupling can be useful for an underactuated grasp, provided it is intentional and observable.

## Base motion changes the transmission

When the shoulder translates or moves around the cylinder, the external-to-distal route can change length and direction. Options include carrying intermediate guides on the base, moving an external actuator assembly, using service loops, or compensating a measured route change. Each has packaging and force consequences.

A proposed z/θ transport mechanism should therefore be drawn with tendons and hoses in more than one posture. The range of the bare carriage alone is not its operating range.

## Stored energy and stopping

Elastic tendons and flexures store energy. Stopping the motor does not remove that energy. Releasing all tension can unexpectedly move the arm or drop a mechanically held component; maintaining tension can preserve an excessive contact load. Recovery depends on the current support configuration and the reason for stopping.

## Useful physical records

Tendon identity, free length, guide route, capstan geometry, anchor geometry, preload, operating posture and load history belong together. These records make measured tip behaviour reproducible without pretending one friction coefficient describes every route.

This is a design description, not a demand for a particular sensor package or tension-control algorithm. The transmission can remain simple if the required operation and its evidence support that simplicity.

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
