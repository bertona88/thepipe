# The Pipe µ — big eyes, tiny hands

**A design atlas for a cylindrical microassembly cell built around printed flexures, cooperative manipulation and engineered contact.**

This is the new direction discussed on 12 September 2026, written as a standalone concept. Its organising idea is simple: keep the observation and actuation infrastructure as large as useful, while making the mechanisms that enter the assembly volume very small. The machine's character comes from the combination: a shared cylindrical workspace, multiple mobile hands, external geometric truth, and printed structures whose geometry does much of the work.

The first world is **300 µm–1 mm parts**, with roughly **500 µm** as a useful reference object. The conversation proposed **10–15 mm distal arms** inside an approximately **80–120 mm optical enclosure**. These are architectural reference scales, not qualified dimensions. How the small arms reach a common workspace is a distinct design question.

## The machine we mean

The Pipe µ is a small, enclosed workshop. Polymer manipulators approach an observed assembly from several directions. Their bases can move along the cylinder and around its circumference. Tendons transmit motion from motors outside. Flexures provide joints, return forces and deliberate compliance. Printed fingertips determine where contact occurs and how attachment changes during release. Some future tools may also carry vacuum or liquid through printed channels.

The enclosure conditions the assembly environment. Parts and tools enter through small transfer chambers. Cameras observe the tools, objects and their relationships. Arms and fixtures cooperate to transfer a component; opening a gripper is an action, while successful release is a separately observed outcome.

The first printed distal structure is intended to use **one known resin and its native surface**, exploiting geometry before adding coatings or multiple structural materials. A metal-free manipulation volume is a design preference, with motors, pumps and electronics outside. Neither choice assumes electrostatic effects have disappeared.

## Read it at three depths

| Depth | Start here | What it contains |
| --- | --- | --- |
| The idea | [Vision](00-vision/README.md), then [the machine in use](00-vision/machine-in-use.md) | Why this machine is interesting and what an operation feels like |
| The architecture | [Architecture](01-architecture/README.md), then [subsystems](02-subsystems/README.md) | Scale separation, spatial organisation and the roles of individual mechanisms |
| The design space | [Contact physics](03-physics/README.md), [2PP opportunities](04-2pp-opportunities/README.md), [evidence](05-evidence/README.md) | Why the ideas might work, what they unlock and how their limits become knowable |

[Design choices](00-vision/design-choices.md) records what was actually preferred. [Open questions](06-reference/open-questions.md) preserves unresolved alternatives. [The discussion map](06-reference/discussion-map.md) links the conversation's ideas to their detailed homes. [DIRECTORY.md](DIRECTORY.md) lists every document.

## How to read the confidence level

**Chosen direction** means an architectural preference expressed in the discussion. **Candidate** means a possible physical implementation. **Illustrative** marks a dimension, calculation or operating example used to reason about the concept. **Exploration** means a capability opened by this direction that is not required for the first machine. **Measured** is reserved for actual evidence on a specified specimen and setup; this atlas contains no new measurements.

This is a description of the desired machine and its possibilities. It does not prescribe repository migration, software work packages or a fixed development schedule. The discussion is the primary source; a small set of research pointers supports selected physical precedents. Existing implementation claims have not been re-audited for this document.

## Browse this folder

- [Vision — a tiny cooperative workshop](00-vision/README.md)
- [Architecture — separate the scales, connect the roles](01-architecture/README.md)
- [Subsystems](02-subsystems/README.md)
- [Physics — small bodies, consequential interfaces](03-physics/README.md)
- [What adopting 2PP unlocks](04-2pp-opportunities/README.md)
- [Evidence — how the concept becomes knowable](05-evidence/README.md)
- [Reference and traceability](06-reference/README.md)
- [Complete document directory](DIRECTORY.md)
