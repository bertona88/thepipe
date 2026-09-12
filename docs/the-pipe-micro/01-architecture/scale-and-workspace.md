# Scale and workspace

## Reference scales

| Quantity | Conversational reference | Interpretation |
| --- | --- | --- |
| Optical enclosure transverse size | 80–120 mm | An illustrative cell diameter, with useful room for optics and support |
| Distal arm reach | 10–15 mm | Local shoulder-to-tool reach, not wall-to-centre reach |
| Components | 300 µm–1 mm | Agreed initial object-size domain |
| Nominal example component | 500 µm | A convenient comparison case |
| Local optical position accuracy | 3–5 µm RMS | Inherited metrology ambition; must be re-established for this scene |
| Macro demonstrator enlargement | Approximately 5–10× | A topology demonstration, not uniform physical similarity |

There is no chosen camera count, frame rate, force range, full cell length or arm count. Texture, channel and hinge dimensions depend on their separate functions and fabrication capabilities.

## The reach constraint

If the enclosure radius is R and a shoulder sits on its inside wall, a distal reach l can only approach the central axis if l is at least R, before tool orientation and collision margins are considered. For a 100 mm diameter cell, R is 50 mm. A 10–15 mm arm mounted directly on that wall cannot reach the centre.

For a shoulder at radius r_s and a target near the axis, the simple necessary positional condition is r_s ≤ l. It is not sufficient: joint limits, orientation, other arms, tool length and occlusion can reduce the usable region considerably.

This separates three design choices that should never be drawn as one number:

- **Enclosure radius:** packaging for windows, conditioning and access.
- **Shoulder locus:** where arm bases actually move or are supported.
- **Cooperative workspace:** the intersection of useful reach and observation for the participating tools.

Candidate arrangements include inward support booms from outer transport, smaller internal support rings, near-wall assembly stations, or task modules positioned close to the shoulders. Each changes visibility, stiffness, accessible volume and routing. No one arrangement is selected here.

## Precision has several meanings

Absolute position accuracy, relative tool-to-part accuracy, angular accuracy, repeatability and verified final assembly quality are different quantities. At 500 µm, 3–5 µm is 0.6–1.0% of the reference length; that percentage says nothing by itself about a bearing fit or a tooth clearance.

If two independent position estimates each have 5 µm RMS uncertainty, their difference has about 7.1 µm RMS uncertainty under that simplified independent-error model. Shared imaging can correlate errors, so the actual relative uncertainty must be estimated rather than automatically using this number. A camera's pixel sampling is also not a calibrated 3D accuracy claim.

Shrinkage compensation and ordinary dimensional calibration belong in fabrication. Thermal drift, deformation and window effects belong in the measured geometry budget. None is removed simply by naming a smaller arm.

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
