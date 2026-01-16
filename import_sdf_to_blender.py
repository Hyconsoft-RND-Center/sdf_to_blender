#!/usr/bin/env python3
"""
Import a Gazebo (Ignition/Fortress) world SDF, resolve Fuel models, and
instance their visual meshes in Blender with pose conversion.

Usage (from shell):
blender --python ./import_sdf_to_blender.py -- \
  --world (YOUR WORLD FILE DIRECTORY).sdf \
  --fuel-root ~/.ignition/fuel/fuel.ignitionrobotics.org \
  --axis-map sdf \
  --realize \
  --cleanup-sources
"""

import argparse
import hashlib
import importlib.util
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix


DEFAULT_WORLD = ""
DEFAULT_FUEL_ROOT = os.path.expanduser(
    "~/.ignition/fuel/fuel.ignitionrobotics.org"
)

SDF_TO_BLENDER = Matrix.Identity(4)
BLENDER_TO_SDF = Matrix.Identity(4)

def ensure_addon(module_name):
    if module_name in bpy.context.preferences.addons:
        return
    if importlib.util.find_spec(module_name) is None:
        return
    try:
        bpy.ops.preferences.addon_enable(module=module_name)
    except Exception:
        pass


def parse_pose(text):
    if not text:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    values = [float(v) for v in text.split()]
    if len(values) != 6:
        raise ValueError(f"pose must have 6 values, got: {text}")
    return values


def parse_scale(text):
    if not text:
        return 1.0, 1.0, 1.0
    values = [float(v) for v in text.split()]
    if len(values) != 3:
        raise ValueError(f"scale must have 3 values, got: {text}")
    return values


def rpy_matrix(roll, pitch, yaw):
    return (
        Matrix.Rotation(yaw, 4, "Z")
        @ Matrix.Rotation(pitch, 4, "Y")
        @ Matrix.Rotation(roll, 4, "X")
    )


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    return Matrix.Translation((x, y, z)) @ rpy_matrix(roll, pitch, yaw)


def convert_matrix(m):
    return SDF_TO_BLENDER @ m @ BLENDER_TO_SDF


def pick_latest_version(model_dir):
    if not os.path.isdir(model_dir):
        return None
    versions = [d for d in os.listdir(model_dir) if d.isdigit()]
    if not versions:
        return None
    return max(versions, key=lambda v: int(v))


def parse_fuel_uri(uri):
    # Example path: /1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfE_01
    if "://" not in uri:
        return None
    parts = uri.split("://", 1)[1].split("/", 1)
    if len(parts) != 2:
        return None
    path = parts[1].lstrip("/")
    fields = path.split("/")
    if len(fields) < 4 or fields[2] != "models":
        return None
    owner = fields[1]
    model_name = fields[3]
    return owner, model_name


def find_model_dir(model_name, fuel_root):
    if not os.path.isdir(fuel_root):
        return None
    for owner in os.listdir(fuel_root):
        owner_dir = os.path.join(fuel_root, owner, "models")
        candidate = os.path.join(owner_dir, model_name.lower())
        if os.path.isdir(candidate):
            return candidate
    return None


def resolve_model_dir(uri, fuel_root):
    parsed = parse_fuel_uri(uri)
    if parsed:
        owner, model_name = parsed
        model_dir = os.path.join(
            fuel_root, owner.lower(), "models", model_name.lower()
        )
        if os.path.isdir(model_dir):
            return model_dir
        return None
    if uri.startswith("model://"):
        model_name = uri[len("model://") :].split("/", 1)[0]
        return find_model_dir(model_name, fuel_root)
    return None


def resolve_mesh_uri(uri, model_version_dir, fuel_root):
    if not uri:
        return None
    if uri.startswith("file://"):
        return uri[len("file://") :]
    if uri.startswith("model://"):
        remainder = uri[len("model://") :]
        parts = remainder.split("/", 1)
        model_name = parts[0]
        relpath = parts[1] if len(parts) > 1 else ""
        model_dir = find_model_dir(model_name, fuel_root)
        if not model_dir:
            return None
        version = pick_latest_version(model_dir)
        if not version:
            return None
        return os.path.join(model_dir, version, relpath)
    if "://" in uri:
        return None
    return os.path.join(model_version_dir, uri)


def import_mesh_as_collection(mesh_path, source_root):
    ext = os.path.splitext(mesh_path)[1].lower()
    before = set(bpy.data.objects)
    if ext == ".dae":
        ensure_addon("io_scene_dae")
        if hasattr(bpy.ops.wm, "collada_import"):
            bpy.ops.wm.collada_import(filepath=mesh_path)
        else:
            before = set(bpy.data.objects)
        bpy.context.view_layer.update()
        new_objs = [obj for obj in bpy.data.objects if obj not in before]
        if not new_objs:
            obj_path = convert_dae_to_obj(mesh_path)
            if obj_path:
                try:
                    return import_mesh_as_collection(obj_path, source_root)
                except RuntimeError:
                    pass
            glb_path = convert_dae_to_glb(mesh_path)
            if glb_path:
                return import_mesh_as_collection(glb_path, source_root)
            raise RuntimeError("Collada import produced no objects")
    elif ext == ".obj":
        if not try_import_obj(mesh_path):
            raise RuntimeError("OBJ import operator not available")
    elif ext == ".stl":
        ensure_addon("io_mesh_stl")
        if hasattr(bpy.ops.wm, "stl_import"):
            bpy.ops.wm.stl_import(filepath=mesh_path)
        elif hasattr(bpy.ops.import_mesh, "stl"):
            bpy.ops.import_mesh.stl(filepath=mesh_path)
        else:
            raise RuntimeError("STL import operator not available")
    elif ext in (".glb", ".gltf"):
        if not try_import_gltf(mesh_path):
            raise RuntimeError("glTF import operator not available")
    else:
        raise RuntimeError(f"unsupported mesh format: {mesh_path}")

    bpy.context.view_layer.update()
    new_objs = [obj for obj in bpy.data.objects if obj not in before]
    if not new_objs:
        new_objs = [obj for obj in bpy.context.selected_objects]
    if not new_objs:
        return None

    mesh_coll = bpy.data.collections.new(
        f"_mesh_{os.path.basename(mesh_path)}"
    )
    source_root.children.link(mesh_coll)
    for obj in new_objs:
        for col in list(obj.users_collection):
            col.objects.unlink(obj)
        mesh_coll.objects.link(obj)
        obj.hide_set(False)
        obj.hide_render = False
    bpy.ops.object.select_all(action="DESELECT")
    return mesh_coll


def convert_dae_to_obj(mesh_path):
    assimp_bin = shutil.which("assimp")
    if not assimp_bin:
        print("[warn] assimp not found; cannot convert DAE")
        return None
    cache_dir = os.path.join(
        os.path.expanduser("~/.cache"), "gazebo_blender", "assimp_obj"
    )
    os.makedirs(cache_dir, exist_ok=True)
    digest = hashlib.sha1(mesh_path.encode("utf-8")).hexdigest()
    basename = os.path.splitext(os.path.basename(mesh_path))[0]
    obj_path = os.path.join(cache_dir, f"{basename}_{digest}.obj")
    if os.path.isfile(obj_path):
        src_mtime = os.path.getmtime(mesh_path)
        if os.path.getmtime(obj_path) >= src_mtime:
            return obj_path
    result = subprocess.run(
        [assimp_bin, "export", mesh_path, obj_path],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"[warn] assimp export failed: {mesh_path}")
        if result.stderr:
            print(result.stderr.strip())
        return None
    return obj_path


def convert_dae_to_glb(mesh_path):
    assimp_bin = shutil.which("assimp")
    if not assimp_bin:
        print("[warn] assimp not found; cannot convert DAE")
        return None
    cache_dir = os.path.join(
        os.path.expanduser("~/.cache"), "gazebo_blender", "assimp_glb"
    )
    os.makedirs(cache_dir, exist_ok=True)
    digest = hashlib.sha1(mesh_path.encode("utf-8")).hexdigest()
    basename = os.path.splitext(os.path.basename(mesh_path))[0]
    glb_path = os.path.join(cache_dir, f"{basename}_{digest}.glb")
    if os.path.isfile(glb_path):
        src_mtime = os.path.getmtime(mesh_path)
        if os.path.getmtime(glb_path) >= src_mtime:
            return glb_path
    result = subprocess.run(
        [assimp_bin, "export", mesh_path, glb_path],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"[warn] assimp export failed: {mesh_path}")
        if result.stderr:
            print(result.stderr.strip())
        return None
    return glb_path


def try_import_obj(mesh_path):
    ensure_addon("io_scene_obj")
    if hasattr(bpy.ops.wm, "obj_import"):
        try:
            bpy.ops.wm.obj_import(filepath=mesh_path)
            return True
        except Exception:
            pass
    if hasattr(bpy.ops.import_scene, "obj"):
        try:
            bpy.ops.import_scene.obj(filepath=mesh_path)
            return True
        except Exception:
            return False
    return False


def try_import_gltf(mesh_path):
    ensure_addon("io_scene_gltf2")
    if hasattr(bpy.ops.wm, "gltf_import"):
        try:
            bpy.ops.wm.gltf_import(filepath=mesh_path)
            return True
        except Exception:
            pass
    if hasattr(bpy.ops.import_scene, "gltf"):
        try:
            bpy.ops.import_scene.gltf(filepath=mesh_path)
            return True
        except Exception:
            return False
    return False


def ensure_collection(name, parent=None):
    coll = bpy.data.collections.get(name)
    if coll:
        return coll
    coll = bpy.data.collections.new(name)
    if parent:
        parent.children.link(coll)
    else:
        bpy.context.scene.collection.children.link(coll)
    return coll


def parse_model_visuals(model_sdf_path):
    tree = ET.parse(model_sdf_path)
    root = tree.getroot()
    model_el = root.find("model")
    if model_el is None:
        return []
    visuals = []
    for link in model_el.findall("link"):
        link_pose = pose_matrix(parse_pose(link.findtext("pose")))
        for visual in link.findall("visual"):
            mesh_el = visual.find("geometry/mesh")
            if mesh_el is None:
                continue
            uri = mesh_el.findtext("uri")
            scale = parse_scale(mesh_el.findtext("scale"))
            visual_pose = pose_matrix(parse_pose(visual.findtext("pose")))
            # Only link + visual pose here; model pose is applied at the world level.
            local_matrix = link_pose @ visual_pose
            visuals.append((uri, local_matrix, scale))
    return visuals


def import_world(world_sdf, fuel_root, out_collection_name):
    tree = ET.parse(world_sdf)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("no <world> in SDF")

    out_coll = ensure_collection(out_collection_name)
    source_coll = ensure_collection("_SourceMeshes")

    mesh_cache = {}

    for model in world.findall("model"):
        include = model.find("include")
        if include is None:
            continue
        uri = include.findtext("uri")
        if not uri:
            continue

        model_dir = resolve_model_dir(uri, fuel_root)
        if not model_dir:
            print(f"[warn] model not found for uri: {uri}")
            continue

        version = pick_latest_version(model_dir)
        if not version:
            print(f"[warn] no version dir in: {model_dir}")
            continue

        model_sdf = os.path.join(model_dir, version, "model.sdf")
        if not os.path.isfile(model_sdf):
            print(f"[warn] model.sdf missing: {model_sdf}")
            continue

        pose_text = model.findtext("pose")
        if not pose_text:
            pose_text = include.findtext("pose")
        model_pose = pose_matrix(parse_pose(pose_text))
        visuals = parse_model_visuals(model_sdf)
        model_version_dir = os.path.join(model_dir, version)

        for idx, (mesh_uri, local_matrix, scale) in enumerate(visuals):
            mesh_path = resolve_mesh_uri(
                mesh_uri, model_version_dir, fuel_root
            )
            if not mesh_path or not os.path.isfile(mesh_path):
                print(
                    f"[warn] mesh not found for uri: {mesh_uri} "
                    f"(resolved: {mesh_path})"
                )
                continue

            mesh_coll = mesh_cache.get(mesh_path)
            if not mesh_coll:
                mesh_coll = import_mesh_as_collection(mesh_path, source_coll)
                if not mesh_coll:
                    print(f"[warn] mesh import failed: {mesh_path}")
                    continue
                mesh_cache[mesh_path] = mesh_coll

            scale_matrix = Matrix.Diagonal(
                (scale[0], scale[1], scale[2], 1.0)
            )
            world_matrix = model_pose @ local_matrix @ scale_matrix
            world_matrix = convert_matrix(world_matrix)

            inst = bpy.data.objects.new(
                f"{model.get('name', 'model')}_{idx}", None
            )
            inst.instance_type = "COLLECTION"
            inst.instance_collection = mesh_coll
            inst.show_instancer_for_viewport = True
            inst.show_instancer_for_render = True
            inst.matrix_world = world_matrix
            out_coll.objects.link(inst)
    return out_coll


def realize_collection_instances(out_coll):
    instancers = [
        obj
        for obj in out_coll.objects
        if obj.instance_type == "COLLECTION" and obj.instance_collection
    ]
    if not instancers:
        return out_coll
    bpy.ops.object.select_all(action="DESELECT")
    for obj in instancers:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = instancers[0]
    bpy.ops.object.duplicates_make_real()
    new_objs = [
        obj
        for obj in bpy.context.selected_objects
        if obj not in instancers and obj.type in {"MESH", "EMPTY", "CURVE"}
    ]
    # Link new objects to the target collection.
    for obj in new_objs:
        if out_coll not in obj.users_collection:
            out_coll.objects.link(obj)
    # Remove instancer empties.
    for inst in instancers:
        try:
            out_coll.objects.unlink(inst)
        except Exception:
            pass
        bpy.data.objects.remove(inst, do_unlink=True)
    return out_coll


def remove_source_meshes():
    src = bpy.data.collections.get("_SourceMeshes")
    if not src:
        return
    for obj in list(src.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.collections.remove(src)


def export_to_obj(path):
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    # Axis settings: Blender default OBJ uses -Z forward, Y up. Keep default.
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.export_scene.obj(
        filepath=path,
        use_selection=False,
        use_materials=False,
        axis_forward="-Y",
        axis_up="Z",
    )
    print(f"[done] exported OBJ to {path}")


def parse_args():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default=DEFAULT_WORLD)
    parser.add_argument("--fuel-root", default=DEFAULT_FUEL_ROOT)
    parser.add_argument("--collection", default="GazeboWorld")
    parser.add_argument(
        "--axis-map",
        choices=("blender", "sdf"),
        default="sdf",
        help="sdf: keep Gazebo axes as-is; blender: rotate +90deg Z (swap X/Y)",
    )
    parser.add_argument(
        "--realize",
        action="store_true",
        help="Make collection instances real meshes (applied transforms).",
    )
    parser.add_argument(
        "--cleanup-sources",
        action="store_true",
        help="Remove SourceMeshes after realize to keep only real meshes.",
    )
    parser.add_argument(
        "--export-obj",
        help="If set, export the scene to OBJ at this path after import/realize.",
    )
    return parser.parse_args(argv)


def main():
    args = parse_args()
    world_path = os.path.expanduser(args.world)
    fuel_root = os.path.expanduser(args.fuel_root)
    global SDF_TO_BLENDER, BLENDER_TO_SDF
    if args.axis_map == "blender":
        SDF_TO_BLENDER = Matrix.Rotation(math.radians(90.0), 4, "Z")
    else:
        SDF_TO_BLENDER = Matrix.Identity(4)
    BLENDER_TO_SDF = SDF_TO_BLENDER.inverted()
    out_coll = import_world(world_path, fuel_root, args.collection)
    if args.realize:
        realized = realize_collection_instances(out_coll)
        if args.cleanup_sources:
            remove_source_meshes()
        if realized:
            out_coll = realized
    print("[done] import complete")
    if args.export_obj:
        export_to_obj(args.export_obj)


if __name__ == "__main__":
    main()
