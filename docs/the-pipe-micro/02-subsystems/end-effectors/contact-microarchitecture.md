# Contact microarchitecture

**Chosen emphasis:** exploit printed surface geometry before introducing coatings. Smoothness and texture are both controllable design variables; neither is universally desirable.

Contact microarchitecture includes the shape and compliance of contact points, their arrangement, the surrounding clearance, and paths for liquid or contamination to leave. It is more specific than choosing a scalar roughness value.

## A useful surface vocabulary

| Geometry | Intended behaviour | What could reverse the benefit |
| --- | --- | --- |
| Flat or smooth pad | Reference interface; potentially conformal adhesion | Large intimate contact can make release difficult |
| Sparse rounded mesas | Limit nominal contact fraction while supporting load | Local stress, flattening or debris bridging the gaps |
| Stiff ribs | Directional shear resistance and defined separation lines | Mechanical interlocking or scratches on the object |
| Inclined compliant fibrils | Direction-dependent attachment | Excess compliance, wear or undesired persistent adhesion |
| Annular ridge | Local vacuum seal around an aperture | Seal stiction, leakage through texture or trapped debris |
| Open grooves | Escape routes for liquid or debris | Grooves may wick and retain liquid instead |
| Wicking features | Deliberately receive liquid or promote capillary attachment | Residue, cross-contamination or uncontrolled spreading |

## Geometry is coupled to loading

An asperity that is stiff under a small contact load may flatten under a larger jaw force. A texture can reduce real contact over one range and increase conformity over another. Contact fraction should therefore be considered under the operating preload, not only from a top-view CAD image.

Pitch, height, top radius, aspect ratio and direction influence behaviour together. A 5–30 µm width/height and 10–50 µm pitch range appeared in the discussion as an example for experimental pads; it is not a compatible full-factorial specification. Pitch must exceed feature width where separated features are intended, and available patch size must support the pattern.

## Separate optical texture from contact texture

A marker that improves pose estimation can change wetting or adhesion if placed in a handling region. A high-adhesion pattern can erase an optical edge through contact. Prefer separate functional zones where practical and characterise a combined zone when it is intentionally shared.

## What success looks like

The desired tool resists the task's tangential load while allowing a controlled release. The ratio R = F_shear,hold / F_normal,pull-off is one diagnostic measured under defined conditions. It is not an objective to maximise without constraints: a weak tool can have a large ratio, and near-zero pull-off makes the ratio noisy. Absolute holding force, damage, wear, reliability and placement error remain necessary.

See the [coupon atlas](../../05-evidence/coupon-atlas.md) for a coherent comparison and [directional adhesion](../../04-2pp-opportunities/directional-adhesion.md) for the deliberately adhesive branch.

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
