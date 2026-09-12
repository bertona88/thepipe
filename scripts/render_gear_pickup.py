#!/usr/bin/env python3
"""Render recorded authoritative gear-pickup frames, never generated motion.

Usage: python scripts/render_gear_pickup.py REPORT.json --output pickup.mp4
Dependencies: numpy, matplotlib and ffmpeg. Metres become mm only for display.
Playback selects recorded samples at uniform simulated-time thresholds;
poses are never interpolated. All changing geometry comes from SceneFrame.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def vector(value):
    if isinstance(value, dict):
        value = [value[k] for k in ("x", "y", "z")]
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("invalid 3D vector")
    return result


def transform(pose, points):
    q = np.asarray(pose["rotation_xyzw"], dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-5:
        raise ValueError("invalid unit quaternion")
    x, y, z, w = q
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result = (np.asarray(points) @ rotation.T + vector(pose["translation_m"])) * 1000
    if not np.isfinite(result).all():
        raise ValueError("nonfinite transformed geometry")
    return result


def mesh(shape):
    """Tessellate declared collision geometry; gear teeth are envelope-only."""
    kind = shape["kind"]
    if kind == "box":
        h = vector(shape["half_extents_m"])
        if np.any(h <= 0):
            raise ValueError("nonpositive box half extents")
        points = np.array([[i,j,k] for i in (-1,1) for j in (-1,1) for k in (-1,1)]) * h
        return points, [[0,1,3,2], [4,6,7,5], [0,4,5,1], [2,3,7,6], [0,2,6,4], [1,5,7,3]]
    n = 20
    angles = np.arange(n)*2*np.pi/n
    if kind in ("sphere", "capsule"):
        r = shape["radius_m"]
        half = shape.get("half_segment_m", 0)
        if r <= 0 or half < 0:
            raise ValueError("invalid capsule/sphere")
        profiles = [(r*np.cos(a), r*np.sin(a) + (-half if a < 0 else half))
                    for a in np.linspace(-np.pi/2,np.pi/2,14)]
        if half:
            profiles = [(r*np.cos(a), r*np.sin(a)-half) for a in np.linspace(-np.pi/2,0,8)] + [(r*np.cos(a),r*np.sin(a)+half) for a in np.linspace(0,np.pi/2,8)]
    elif kind == "gear":
        r, b, h = shape["tip_radius_m"], shape["bore_radius_m"], shape["half_thickness_m"]
        offset = shape["tooth_center_offset_m"]
        if not 0 < b < r or h <= 0:
            raise ValueError("invalid gear geometry")
        profiles = [(r,offset-h),(r,offset+h),(b,offset+h),(b,offset-h),(r,offset-h)]
    else:
        raise ValueError(f"unsupported shape {kind}")
    points = np.array([[r*np.cos(a),r*np.sin(a),z] for r,z in profiles for a in angles])
    faces = [[row*n+j,row*n+(j+1)%n,(row+1)*n+(j+1)%n,(row+1)*n+j]
             for row in range(len(profiles)-1) for j in range(n)]
    return points, faces


def draw_shape(ax, shape, pose, color, alpha=1):
    points, faces = mesh(shape)
    vertices = transform(pose, points)
    ax.add_collection3d(Poly3DCollection([vertices[f] for f in faces],
                      facecolors=color, edgecolors=(0.1,0.2,0.3,0.16), linewidths=.2, alpha=alpha, axlim_clip=True))


def load_report(path):
    source_bytes = path.read_bytes()
    report = json.loads(source_bytes, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(f"nonfinite JSON {v}")))
    report["_render_source_sha256"] = hashlib.sha256(source_bytes).hexdigest()
    desc = report["scene_description"]
    if report["schema_version"] != 1 or not report["status"]:
        raise ValueError("unsupported report schema or missing outcome")
    if desc["schema_version"] != 1 or desc["units"]["length"] != "m":
        raise ValueError("unsupported scene schema or units")
    if not report.get("source_revision") or len(desc["machine_config_sha256"]) != 64:
        raise ValueError("missing revision/configuration identity")
    if report["machine_config_sha256"] != desc["machine_config_sha256"]:
        raise ValueError("disconnected configuration identity")
    samples = report["samples"]
    if not samples:
        raise ValueError("no recorded samples")
    previous = (-1, -1.)
    for sample in samples:
        scene = sample["scene_frame"]
        tick, time = sample["tick"], sample["time_s"]
        if not math.isfinite(time) or tick < previous[0] or time < previous[1]:
            raise ValueError("invalid/nonmonotonic sample time")
        if scene["schema_version"] != 1 or scene["tick"] != tick or scene["time_s"] != time or scene["truth"] is None:
            raise ValueError("disconnected sample/scene")
        if not sample["phase"] or sample["command_sequence"] != scene["commanded"]["command_sequence"]:
            raise ValueError("missing phase/disconnected command")
        for arm in scene["truth"]["manipulators"]:
            transform(arm["tool_pose"], [[0,0,0]])
            for collider in arm["link_colliders"] + arm["tool_colliders"]:
                vertices, _ = mesh(collider["shape"])
                transform(collider["pose"], vertices)
        metrology = sample.get("metrology")
        if metrology is not None and (metrology["machine_tick"] != tick or metrology["time_s"] != time):
            raise ValueError("stale observation attached to sample")
        for body in scene["truth"]["rigid_bodies"]:
            transform(body["pose"], [[0,0,0]])
        previous = (tick,time)
    return report


def render(report, output, fps, duration):
    desc, samples = report["scene_description"], report["samples"]
    body_shapes = {b["id"]: b["shape"] for b in desc["rigid_bodies"]}
    times = np.array([s["time_s"] for s in samples])
    # Retain every phase transition even when uniform playback decimates samples.
    indices = set(np.minimum(np.searchsorted(times, np.linspace(times[0],times[-1],max(2,int(fps*duration)))),len(samples)-1))
    indices.update(i for i,s in enumerate(samples) if i == 0 or s["phase"] != samples[i-1]["phase"])
    indices = sorted(indices)
    first_arm = samples[0]["scene_frame"]["truth"]["manipulators"][0]
    arm_id = first_arm["id"]
    tools = np.array([vector(next(a for a in s["scene_frame"]["truth"]["manipulators"] if a["id"] == arm_id)["tool_pose"]["translation_m"])*1000 for s in samples])
    gear_bodies = [b for b in samples[0]["scene_frame"]["truth"]["rigid_bodies"] if body_shapes[b["id"]]["kind"] == "gear"]
    if len(gear_bodies) != 1:
        raise ValueError("pickup replay requires one identified gear")
    center = vector(gear_bodies[0]["pose"]["translation_m"])*1000
    close_radius = 6.5
    # Fixed world view must encompass the entire executed carriage sweep, not
    # merely its initial shoulder position. Bounds use recorded collider meshes.
    lower = np.full(3, np.inf)
    upper = np.full(3, -np.inf)
    for sample in samples:
        active = next(a for a in sample["scene_frame"]["truth"]["manipulators"] if a["id"] == arm_id)
        for collider in active["link_colliders"] + active["tool_colliders"]:
            points, _ = mesh(collider["shape"])
            world = transform(collider["pose"], points)
            lower = np.minimum(lower, world.min(axis=0))
            upper = np.maximum(upper, world.max(axis=0))
    lower = np.minimum(lower, center-close_radius)
    upper = np.maximum(upper, center+close_radius)
    whole_center = (lower+upper)/2
    whole_radius = float(np.max(upper-lower))/2+5.
    fig = plt.figure(figsize=(14,8), facecolor="#f7f9fc")
    axes = [fig.add_axes([.035,.20,.43,.61],projection="3d"),fig.add_axes([.51,.20,.46,.61],projection="3d")]
    title = fig.text(.045,.955,"",fontsize=19,weight="bold",color="#182b40")
    status = fig.text(.045,.895,"",fontsize=11,color="#182b40")
    detail = fig.text(.52,.16,"",fontsize=10,color="#182b40")
    outcome = str(report["status"]) + (": " + str(report["refusal"]) if report.get("refusal") else "")
    fig.text(.045,.86,"Recorded outcome: " + outcome, fontsize=9, color="#8b3b23" if report.get("refusal") else "#182b40")
    fig.text(.045,.11,"Gray: runtime arm/tool   •   Blue: part envelope   •   Amber: support\nGreen ×: optical estimate   •   Magenta +: calibrated feature inference",fontsize=10,color="#182b40")
    fig.text(.045,.037,"Reduced simulation · Rigid grasp · Zero gravity · Hardware unqualified\nGear cylindrical envelope; physical tooth-marker layout unqualified. No pose interpolation.\nReturn permits ≤20 µm support gap; no load-transfer or braking model. Recorded samples; accelerated playback.",fontsize=10,color="#586675")
    fig.text(.045,.012,f"Revision {report['source_revision']}  |  Config {desc['machine_config_sha256'][:16]}",fontsize=8,color="#586675")
    writer = FFMpegWriter(fps=fps,metadata={"title":"The Pipe — recorded gear pickup operation"},codec="libx264",extra_args=["-pix_fmt","yuv420p"])
    output.parent.mkdir(parents=True,exist_ok=True)
    held_preview_saved = False
    with writer.saving(fig,str(output),dpi=110):
        for frame_number,index in enumerate(indices):
            sample = samples[index]
            scene = sample["scene_frame"]
            arm = next(a for a in scene["truth"]["manipulators"] if a["id"] == arm_id)
            title.set_text("The Pipe  /  " + sample["phase"].replace("_"," "))
            status.set_text(f"Simulation {sample['time_s']:.3f} s  ·  Tick {sample['tick']}  ·  Command {sample['command_sequence']}  ·  Recorded sample {index+1}/{len(samples)}")
            metrology = sample.get("metrology")
            estimates=[]
            if metrology:
                for entity in metrology["world"]["entities"].values():
                    if entity.get("pose"):
                        estimates.append((vector(entity["pose"]["world_from_tcp"]["translation"])*1000,"#168457","x"))
                    elif entity.get("point"):
                        estimates.append((vector(entity["point"]["position_world_m"])*1000,"#168457","x"))
                for feature in metrology["inferred_features"].values():
                    estimates.append((vector(feature["center"]["position_world_m"])*1000,"#b234a7","+"))
            detail.set_text(f"Jaw opening: {arm['gripper']['opening_m']*1000:.3f} mm\nHeld body: {sample['held_part_body_id']}  ·  Observations: {len(estimates) if metrology else 'unavailable'}")
            for ax,view_center,radius,label in zip(axes,[whole_center,center],[whole_radius,close_radius],["Active arm — world geometry","Distal tool — fixed close-up"]):
                ax.clear()
                ax.set_title(label,fontsize=12,pad=10)
                ax.set_facecolor("#f7f9fc")
                ax.view_init(elev=24,azim=-65)
                ax.set_box_aspect((1,1,1))
                ax.set(xlim=(view_center[0]-radius,view_center[0]+radius),ylim=(view_center[1]-radius,view_center[1]+radius),zlim=(view_center[2]-radius,view_center[2]+radius),xlabel="X / mm",ylabel="Y / mm",zlabel="Z / mm")
                ax.tick_params(labelsize=7)
                for c in arm["link_colliders"]+arm["tool_colliders"]:
                    draw_shape(ax,c["shape"],c["pose"],"#80909f")
                for body in scene["truth"]["rigid_bodies"]:
                    if not body["enabled"]:
                        continue
                    shape=body_shapes[body["id"]]
                    # Ignore remote fixtures outside either fixed view, never move them.
                    position=vector(body["pose"]["translation_m"])*1000
                    if np.max(abs(position-view_center)) < radius*2:
                        draw_shape(ax,shape,body["pose"],"#329bd1" if shape["kind"] == "gear" else "#d9a653",.85)
                for point,color,marker in estimates:
                    ax.scatter(*point,color=color,marker=marker,s=55,depthshade=False,zorder=10)
            writer.grab_frame()
            if frame_number == len(indices)//2:
                fig.savefig(output.with_suffix(".png"),dpi=130,facecolor=fig.get_facecolor())
            if sample["phase"] == "hold_for_observation" and not held_preview_saved:
                fig.savefig(output.with_suffix(".held.png"),dpi=130,facecolor=fig.get_facecolor())
                held_preview_saved = True
            if frame_number == len(indices)-1:
                fig.savefig(output.with_suffix(".final.png"),dpi=130,facecolor=fig.get_facecolor())
    plt.close(fig)
    output.with_suffix(".render.json").write_text(json.dumps({
        "source_report_sha256": report["_render_source_sha256"],
        "renderer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "whole_view_center_mm": whole_center.tolist(),
        "whole_view_radius_mm": whole_radius,
        "source_revision": report["source_revision"],
        "source_tree_sha256": report["source_tree_sha256"],
        "configuration_sha256": report["configuration_sha256"],
        "fps": fps, "requested_duration_s": duration,
        "rendered_duration_s": len(indices)/fps,
        "sampling": "Recorded samples at uniform simulation-time thresholds plus all phase transitions; no interpolation",
        "sample_indices": [int(i) for i in indices],
        "ticks": [samples[i]["tick"] for i in indices],
        "length_display_unit": "mm",
        "geometry": "Authoritative collision primitives; gear tooth envelope shown as annulus",
    }, indent=2)+"\n")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--fps",type=int,default=20)
    parser.add_argument("--duration",type=float,default=16.)
    parser.add_argument("--validate-only",action="store_true")
    args=parser.parse_args()
    if not 1 <= args.fps <= 60 or not 0 < args.duration <= 300:
        parser.error("invalid playback limits")
    try:
        report=load_report(args.report)
        if not args.validate_only:
            render(report,args.output,args.fps,args.duration)
        print(f"Validated {len(report['samples'])} authoritative samples; source SHA256 {report['_render_source_sha256']}")
    except (KeyError,ValueError,TypeError,IndexError) as error:
        parser.exit(2,f"Replay unavailable: {error}\n")


if __name__ == "__main__":
    main()
