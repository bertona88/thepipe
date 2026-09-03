# Optical/robot co-design M1d

Status: executable analytic design candidate; hardware qualification not started

M1d answers the immediate architecture question: the arms do not need micrometre absolute
accuracy everywhere, but they do need to enter a local optical capture volume, hold still during a
measurement burst, and execute small repeatable corrections whose residual fits the phase error
budget. The proposed system therefore retains wide-field global vision and adds a rigid local
camera/projector head for stop-and-look manipulation.

The authoritative study input is `scenarios/optical_codesign_m1d.json`. Run it with:

```bash
cargo run --locked -p pipe_sim_cli --bin pipe-optical-codesign -- --compact
```

The same deterministic report is exported to WebAssembly as `opticalCodesignReportJson`. CI saves
the JSON as `out/m1d_optical_codesign.json`.

## Claim boundary

Three evidence classes are kept separate:

| Class | Meaning in M1d |
| --- | --- |
| Manufacturer-specified | A supplier states a sensor/projector property. It does not establish system precision. |
| Modeled | The executable pinhole/error-budget model predicts a value from declared geometry and uncertainty assumptions. |
| Hardware measurement required | Focus, MTF, localization, sync, drift, glare, surface bias, arm hold and correction performance have not been demonstrated. |

Consequently, the report may say `model_feasible_hardware_qualification_required`; it can never
emit a hardware-qualified pass. The first-order model omits lens MTF, defocus, estimator bias,
feature geometry, transparent-tube multipath, occlusion correlations, vibration, rolling exposure,
and calibration target uncertainty. Those effects can only worsen the result until measured.

## Draft optical configuration

| Layer | Draft configuration | Job |
| --- | --- | --- |
| Global | Six 1280 x 800 monochrome global-shutter cameras; radius 60 mm; z = -106/+106 mm; azimuths 0/120/240° and 60/180/300°; 68° horizontal field | QWV coverage, coarse pose, transit and macro-head acquisition |
| Macro | One 1280 x 800 monochrome global-shutter camera plus one calibrated coded/line projector on the same rigid observer head | Local feature triangulation and alignment |
| Macro geometry | 12 mm effective entrance-pupil baseline, 15 mm perpendicular working distance, 2.5 x 1.5625 mm field | <=2 µm/px nominal sampling with useful triangulation angle |
| Timing | Eight-pattern burst at a required 240 patterns/s plus 20 ms processing; manipulated arm stationary | 53.3 ms conservative sensor-to-estimate budget |
| Illumination | Switchable bright/dark banks and coded source; wavelength/filter still open | Separate fiducials, projected code and glare rejection |

An OV9281-class module is the cost-oriented camera basis because the manufacturer documentation
lists a 1280 x 800, 3 µm-pixel, monochrome global-shutter family
([Arducam OV9281/OV9282 documentation](https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Global-Shutter/1MP-OV9281-OV9282/)).
That is a sensor-class choice, not a frozen SKU: trigger behavior, board packaging, lens mount,
close focus and host bandwidth must be checked on the exact module.

The [Raspberry Pi Global Shutter Camera](https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/)
is a documented IMX296/C-CS-mount alternative for the bench, but its 1456 x 1088 sensor, 3.45 µm
pixels, board envelope and lens choices require a separate optical model. It must not be dropped
into the OV9281 numbers by name substitution. The
[TI DLP3010LC](https://www.ti.com/product/DLP3010LC) and its
[light-control EVM documentation](https://www.ti.com/document-viewer/lit/html/DLPU070) provide a
development reference for coded projection and triggering; they are not accepted as the low-cost
production projector. The 240-pattern/s value in M1d is a requirement that the eventual source
must prove, not a claimed property of an unspecified budget projector.

The macro view is deliberately smaller than the complete 6 x 4 mm housing. Local marks and the
active insertion feature are observed in one view; full-housing inspection uses registered tiles.
The 12 mm value is an **effective entrance-pupil baseline**, not permission to assume that two
camera circuit boards fit 12 mm apart. A folded path or a camera/projector arrangement may be
needed, and the rigid package must be collision-checked before CAD is frozen.

## Precision model

For object-plane field width $W$, image width $N$, slant range $z$, effective focal length $f$,
included triangulation angle $\theta$, camera localization uncertainty $\sigma_c$, and
correspondence uncertainty $\sigma_p$:

$$
p = \frac{W}{N}, \qquad f = \frac{z}{p}
$$

$$
\sigma_{xy,random}=\frac{z\sigma_c}{f}=p\sigma_c,
\qquad
\sigma_{z,geometric}=\frac{z\sqrt{\sigma_c^2+\sigma_p^2}}{f\sin\theta}
$$

Independent random terms are combined by root-sum-square. The correlated calibration floor is
included once and is never divided by the number of frames. Quantization is treated as a uniform
distribution with $\sigma=q/\sqrt{12}$.

| Output | Global adjacent-pair model | Macro camera/projector model |
| --- | ---: | ---: |
| Object sampling | 128.371 µm/px | 1.953 µm/px |
| Range | 121.803 mm | 16.155 mm slant (15 mm perpendicular) |
| Effective baseline | 103.923 mm | 12.000 mm |
| Included angle | 50.504° | 43.603° |
| Localization assumption | 0.18 px per coordinate | 0.18 px per coordinate |
| Correlated calibration allocation | 8.0 µm | 3.0 µm |
| Surface/depth allocation | 8.0 µm axial | 1.5 µm axial |
| Modeled lateral RMS | **24.45 µm** | **3.02 µm** |
| Modeled depth RMS | **43.83 µm** | **3.43 µm** |

The global result narrowly fits the 25/50 µm nominal geometric targets, so there is little margin
for unmodeled glare or calibration bias. It is suitable as a hypothesis to test, not a reason to
buy six cameras before a two-view coupon succeeds. Its 128 µm pixel footprint also means that the
subpixel pose result depends on high-contrast multi-pixel features; it does not resolve arbitrary
24 µm surface detail.

The macro result is below the 8 µm nominal depth target in the model. Most of that result comes
from assumed 3 µm calibration and 1.5 µm surface floors, not sensor resolution. A bench result will
therefore be accepted only from held-out gauge residuals across position, incidence, material and
temperature—not from reprojection error on the calibration target.

## Field/baseline sweep

The deterministic sweep keeps the 15 mm perpendicular working distance and all macro uncertainty
allocations fixed. Depth values below are modeled RMS in micrometres.

| Field width | Sampling | 6 mm baseline | 12 mm baseline | 20 mm baseline | Sampling gate |
| ---: | ---: | ---: | ---: | ---: | --- |
| 2.0 mm | 1.563 µm/px | 3.51 | 3.41 | 3.39 | nominal pass |
| 2.5 mm | 1.953 µm/px | 3.60 | 3.43 | 3.40 | nominal pass |
| 3.0 mm | 2.344 µm/px | 3.70 | 3.47 | 3.42 | worst-case only |
| 4.0 mm | 3.125 µm/px | 3.94 | 3.55 | 3.47 | **fail** |

Increasing baseline helps geometric depth only modestly after the fixed calibration/surface floors
dominate. That makes 12 mm a reasonable starting point: 20 mm buys little modeled precision while
making tool visibility and packaging harder. The field width is more consequential; 4 mm misses
even the 3 µm/px worst-case sampling requirement.

## Phase-by-phase robot co-design

For each phase, the remaining loaded arm/control allocation is solved as:

$$
\sigma_{arm,max}=\sqrt{\sigma_{target}^{2}-\sigma_{optics}^{2}-
\sigma_{hold}^{2}-\sigma_{latency}^{2}-\sigma_{contact}^{2}}
$$

The result is the maximum residual after a commanded closed-loop correction. It is not the arm's
open-loop absolute accuracy or encoder resolution. Hold/contact numbers are provisional design
allocations and must be replaced with coupon data.

| Phase | Motion/observation policy | Target RMS lat/depth | Provisional hold/contact lat/depth | Remaining arm residual lat/depth |
| --- | --- | ---: | ---: | ---: |
| Survey and transit | Global, timestamped continuous prediction | 200/200 µm | 0/0 + 100/100 µm latency allocation | 171.5/167.6 µm |
| Macro capture | Stop, acquire burst, estimate | 10/20 µm | 2/2 + 0/0 µm | 9.32/19.60 µm |
| Approach and grasp | Increment, stop, measure | 10/20 µm | 2/2 + 3/3 µm | 8.82/19.37 µm |
| Grasp verify/transfer | Static checkpoint, then global transfer | 20/40 µm | 5/5 + 3/3 µm | 18.89/39.42 µm |
| Pre-insert alignment | Stop, acquire burst, correct | 10/20 µm | 2/2 + 0/0 µm | 9.32/19.60 µm |
| Guarded insertion | Increment, measure, force-check | 10/20 µm | 2/2 + 5/8 µm | **7.87/17.89 µm** |
| Seat/release verify | Static verification before release | 10/20 µm | 2/2 + 3/5 µm | 8.82/18.95 µm |

The tightest current requirement is therefore about **7.9 µm lateral loaded control residual**
during guarded insertion, provided the optical and contact assumptions survive measurement. That
does not mean an inexpensive tendon arm must repeatedly hit an absolute world coordinate to 8 µm.
It must hold to roughly 2 µm during the 53.3 ms burst and make a correction whose residual is below
7.9 µm in the local observed frame. If the arm cannot do that, the correct co-design response is
not automatically a more expensive arm: first test smaller correction steps, flexure guidance,
passive lead-ins, a tighter local field, better calibration, or more stop-and-look iterations.

For transit, the 100 µm latency allocation and 41.7 ms global estimate time imply at most about
2.4 mm/s uncompensated image-relative motion. Faster motion requires a timestamped motion model;
it cannot simply reuse a stale pose.

## Bench qualification sequence

1. Build only two global views and one rigid macro camera/projector coupon head.
2. Verify common exposure timing and measure the actual capture-to-estimate latency distribution.
3. Focus on a traceable gauge at 10, 15, 20 and 25 mm; measure object sampling and slanted-edge MTF.
4. Calibrate on one target, then report held-out 3D residuals on a separate gauge over the declared
   field, angle, material, lighting and temperature envelope.
5. Repeat through the clear tube and record false return, dropout and bias—not only accepted points.
6. Observe a loaded calibration pointer while each tendon arm holds for a coded burst; measure
   drift, vibration and minimum repeatable correction in lateral and axial directions.
7. Feed those measured distributions back into this scenario and rerun every phase allocation.
8. Permit near-contact motion only after all phase budgets remain feasible and two-view loss causes
   the configured hold/retreat action.

The first downselect is therefore a **two-view global coupon plus one macro head**, run alongside a
loaded one-arm repeatability coupon. Six-camera replication, the final projector, and wrist CAD
freeze come only after those measurements close the same executable budget.
