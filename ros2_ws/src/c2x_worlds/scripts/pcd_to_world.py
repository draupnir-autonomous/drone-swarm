#!/usr/bin/env python3
"""Convert a MARSIM benchmark scenario point cloud (.pcd) into a Gazebo
world: voxelize the raw scan points, greedy-mesh the voxel grid's exposed
surface into a binary STL, and wrap it in an SDF world usable by PX4 SITL.

MARSIM ray-casts directly against the raw point cloud with no mesh/physics
at all; Gazebo needs real collision geometry, so this is not a 1:1 format
conversion but a genuine reconstruction step. Greedy surface meshing (merge
adjacent coplanar voxel faces into larger rectangles, standard voxel-engine
technique) keeps the triangle count proportional to visible surface area
instead of point count, which is what keeps Gazebo's physics/rendering
tractable on a ~300-750k point scan.

Usage: pcd_to_world.py <input.pcd> <world_name> <output_dir> [--voxel 0.2]
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np


def read_pcd(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            if line.startswith("DATA"):
                data_type = line.split()[1]
                break
            parts = line.split()
            if parts:
                header[parts[0]] = parts[1:]
        n_points = int(header["POINTS"][0])
        if data_type == "binary":
            raw = f.read(n_points * 3 * 4)
            pts = np.frombuffer(raw, dtype=np.float32).reshape(n_points, 3).astype(np.float64)
        elif data_type == "ascii":
            pts = np.loadtxt(f, max_rows=n_points, dtype=np.float64)
        else:
            raise ValueError(f"Unsupported PCD DATA type: {data_type}")
    return pts


def voxelize(pts, voxel):
    mins = pts.min(axis=0)
    idx = np.floor((pts - mins) / voxel).astype(np.int64)
    dims = idx.max(axis=0) + 1
    occ = np.zeros(tuple(dims), dtype=bool)
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    return occ, mins


def exposed_faces(occ, axis, sign):
    """Boolean mask (same shape as occ) marking voxels with an exposed face
    on the +/- side of `axis`."""
    shifted = np.zeros_like(occ)
    sl_src = [slice(None)] * 3
    sl_dst = [slice(None)] * 3
    if sign > 0:
        sl_src[axis] = slice(1, None)
        sl_dst[axis] = slice(0, -1)
    else:
        sl_src[axis] = slice(0, -1)
        sl_dst[axis] = slice(1, None)
    shifted[tuple(sl_dst)] = occ[tuple(sl_src)]
    return occ & ~shifted


def greedy_rects(mask):
    """Greedy rectangle decomposition of a 2D boolean mask. Yields
    (row0, col0, row1, col1) exclusive-end rectangles covering all True
    cells, each cell covered exactly once."""
    mask = mask.copy()
    h, w = mask.shape
    for r in range(h):
        c = 0
        while c < w:
            if not mask[r, c]:
                c += 1
                continue
            # Expand width.
            c_end = c + 1
            while c_end < w and mask[r, c_end]:
                c_end += 1
            # Expand height while the same [c, c_end) span stays solid.
            r_end = r + 1
            while r_end < h and mask[r_end, c:c_end].all():
                r_end += 1
            mask[r:r_end, c:c_end] = False
            yield (r, c, r_end, c_end)
            c = c_end


def mesh_voxels(occ, voxel, origin):
    """Greedy-mesh every exposed face of the voxel grid. Returns an
    (N, 3, 3) float32 array of triangle vertices in world coordinates."""
    tris = []
    dims = occ.shape
    for axis in range(3):
        other = [a for a in range(3) if a != axis]
        for sign in (-1, 1):
            faces = exposed_faces(occ, axis, sign)
            # Slide along `axis`; for each layer, greedy-merge the 2D mask
            # of exposed faces into rectangles.
            for layer in range(dims[axis]):
                sl = [slice(None)] * 3
                sl[axis] = layer
                mask = faces[tuple(sl)]
                if not mask.any():
                    continue
                face_coord = layer + (1 if sign > 0 else 0)
                for r0, c0, r1, c1 in greedy_rects(mask):
                    corner = [0.0, 0.0, 0.0]
                    corner[axis] = face_coord
                    corner[other[0]] = r0
                    corner[other[1]] = c0
                    p00 = list(corner)
                    p10 = list(corner); p10[other[0]] = r1
                    p01 = list(corner); p01[other[0]] = r0; p01[other[1]] = c1
                    p11 = list(corner); p11[other[0]] = r1; p11[other[1]] = c1
                    quad = np.array([p00, p10, p11, p01], dtype=np.float64)
                    world_quad = origin + quad * voxel
                    if sign > 0:
                        tris.append(world_quad[[0, 1, 2]])
                        tris.append(world_quad[[0, 2, 3]])
                    else:
                        tris.append(world_quad[[0, 2, 1]])
                        tris.append(world_quad[[0, 3, 2]])
    return np.array(tris, dtype=np.float32)


def write_binary_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        norm_len = np.linalg.norm(normals, axis=1, keepdims=True)
        norm_len[norm_len == 0] = 1.0
        normals = (normals / norm_len).astype(np.float32)
        for i in range(len(tris)):
            f.write(struct.pack("<3f", *normals[i]))
            f.write(struct.pack("<9f", *tris[i].flatten()))
            f.write(struct.pack("<H", 0))


SDF_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{world_name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Required for PX4's simulated GPS to get a valid fix - without this
         the EKF never reports xy_valid=true regardless of spawn position. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{plane_x} {plane_y}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{plane_x} {plane_y}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="{world_name}_geometry">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.55 0.55 0.6 1</ambient>
            <diffuse>0.55 0.55 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd_path")
    ap.add_argument("world_name")
    ap.add_argument("output_dir")
    ap.add_argument("--voxel", type=float, default=0.2)
    ap.add_argument("--z-min", type=float, default=None,
                     help="drop points below this z (e.g. to remove a scanned floor)")
    ap.add_argument("--z-max", type=float, default=None)
    args = ap.parse_args()

    print(f"Reading {args.pcd_path} ...")
    pts = read_pcd(args.pcd_path)
    print(f"  {len(pts)} points, bounds x[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] "
          f"y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] "
          f"z[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")

    if args.z_min is not None:
        pts = pts[pts[:, 2] >= args.z_min]
    if args.z_max is not None:
        pts = pts[pts[:, 2] <= args.z_max]

    print(f"Voxelizing at {args.voxel} m ...")
    occ, mins = voxelize(pts, args.voxel)
    n_occ = int(occ.sum())
    print(f"  grid shape {occ.shape}, {n_occ} occupied voxels")

    print("Greedy-meshing exposed surface ...")
    tris = mesh_voxels(occ, args.voxel, mins)
    print(f"  {len(tris)} triangles")

    out_dir = Path(args.output_dir)
    meshes_dir = out_dir / "meshes"
    worlds_dir = out_dir / "worlds"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    worlds_dir.mkdir(parents=True, exist_ok=True)

    stl_path = meshes_dir / f"{args.world_name}.stl"
    write_binary_stl(stl_path, tris)
    print(f"Written {stl_path} ({stl_path.stat().st_size / 1e6:.1f} MB)")

    span = pts.max(axis=0) - pts.min(axis=0)
    sdf_path = worlds_dir / f"{args.world_name}.sdf"
    # Absolute file:// URI, not a model:// resource-path lookup: PX4's world
    # loading is not resource-path based (see c2x_worlds/empty_room's own
    # notes/memory) - a literal path is the only thing proven reliable here.
    sdf_path.write_text(SDF_TEMPLATE.format(
        world_name=args.world_name,
        mesh_uri=f"file://{stl_path.resolve()}",
        plane_x=span[0] + 4, plane_y=span[1] + 4,
    ))
    print(f"Written {sdf_path}")


if __name__ == "__main__":
    main()
