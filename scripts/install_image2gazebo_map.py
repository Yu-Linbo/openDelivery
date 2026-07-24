#!/usr/bin/env python3
"""Install an image2gazebo export into openDelivery.

1) Replace simulate 3D mesh (drawn_model) used by drawn_model.world (4 quadrant copies).
2) Materialize floor dirs map/<name>/ with occupancy + semantic maps.

Default: clone one export into test_101 .. test_104 (same geometry, renamed assets).

Examples:
  python3 scripts/install_image2gazebo_map.py \\
    --zip map/test_101_20260723_042608.zip

  python3 scripts/install_image2gazebo_map.py \\
    --source ../tools/image2gazebo/result/test_101/20260723_042608 \\
    --floors test_101,test_102,test_103,test_104
"""

from __future__ import annotations

import argparse
import re
import shutil
import struct
import tempfile
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SIM_MODEL = ROOT / "src/simulate/simulate/model/drawn_model"
DEFAULT_MAP_DIR = ROOT / "map"
DEFAULT_FLOORS = ("test_101", "test_102", "test_103", "test_104")

MODEL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="drawn_model">
    <link name="link">
      <!-- Center STL AABB on model origin so world poses (±10,±10) sit in zone centers. -->
      <collision name="collision">
        <pose>{ox:.6f} {oy:.6f} 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://drawn_model/map.stl</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>{ox:.6f} {oy:.6f} 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://drawn_model/map.stl</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
    <static>1</static>
  </model>
</sdf>
"""


def _stl_xy_center(stl_path: Path) -> tuple[float, float]:
    """Return (cx, cy) of binary/ascii STL axis-aligned bounding box."""
    data = stl_path.read_bytes()
    mins = [1e30, 1e30, 1e30]
    maxs = [-1e30, -1e30, -1e30]

    def acc(x: float, y: float, z: float) -> None:
        mins[0] = min(mins[0], x)
        mins[1] = min(mins[1], y)
        mins[2] = min(mins[2], z)
        maxs[0] = max(maxs[0], x)
        maxs[1] = max(maxs[1], y)
        maxs[2] = max(maxs[2], z)

    if data[:5].lower() == b"solid" and b"\x00" not in data[:80]:
        # ASCII STL
        for line in data.decode("utf-8", errors="ignore").splitlines():
            parts = line.strip().split()
            if len(parts) >= 4 and parts[0] == "vertex":
                acc(float(parts[1]), float(parts[2]), float(parts[3]))
    else:
        n = struct.unpack_from("<I", data, 80)[0]
        off = 84
        for _ in range(n):
            vals = struct.unpack_from("<12fH", data, off)
            off += 50
            for v in range(3):
                acc(vals[3 + 3 * v], vals[4 + 3 * v], vals[5 + 3 * v])

    if mins[0] > maxs[0]:
        raise ValueError(f"empty or unreadable STL: {stl_path}")
    return (mins[0] + maxs[0]) / 2.0, (mins[1] + maxs[1]) / 2.0


def _find_run_root(extracted: Path) -> Path:
    """Locate the timestamped run dir that contains maps/ + model/."""
    candidates = []
    for maps_dir in extracted.rglob("maps"):
        if not maps_dir.is_dir():
            continue
        run = maps_dir.parent
        if (run / "model").is_dir() and (run / "worlds").is_dir():
            candidates.append(run)
    if not candidates:
        raise FileNotFoundError(
            f"no image2gazebo run (maps/+model/+worlds/) under {extracted}"
        )
    # Prefer deepest / most recently named path
    candidates.sort(key=lambda p: str(p))
    return candidates[-1]


def _detect_model_name(run: Path) -> str:
    models = [p for p in (run / "model").iterdir() if p.is_dir()]
    if len(models) != 1:
        raise FileNotFoundError(f"expected one model dir under {run / 'model'}, got {models}")
    return models[0].name


def _rewrite_yaml_image(text: str, image_name: str) -> str:
    if re.search(r"^image:\s*.+$", text, flags=re.M):
        return re.sub(r"^image:\s*.+$", f"image: {image_name}", text, count=1, flags=re.M)
    return f"image: {image_name}\n{text}"


def _install_simulate_mesh(run: Path, model_name: str, sim_model: Path, dry_run: bool) -> None:
    src_stl = run / "model" / model_name / "map.stl"
    if not src_stl.is_file():
        raise FileNotFoundError(f"missing STL: {src_stl}")
    dst_stl = sim_model / "map.stl"
    cx, cy = _stl_xy_center(src_stl)
    ox, oy = -cx, -cy
    print(f"[simulate] {src_stl} -> {dst_stl}")
    print(f"[simulate] STL xy center=({cx:.3f}, {cy:.3f}) -> mesh pose=({ox:.3f}, {oy:.3f}, 0)")
    if dry_run:
        return
    sim_model.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_stl, dst_stl)
    (sim_model / "model.sdf").write_text(
        MODEL_SDF_TEMPLATE.format(ox=ox, oy=oy), encoding="utf-8"
    )
    readme = sim_model / "readme"
    note = (
        f"Replaced by image2gazebo model '{model_name}' "
        f"(run: {run.name}). Keep Gazebo name drawn_model for world/spawn compatibility.\n"
        f"Mesh pose offset ({ox:.6f}, {oy:.6f}, 0) centers STL AABB on model origin.\n"
    )
    readme.write_text(note, encoding="utf-8")


def _install_floor(
    run: Path,
    model_name: str,
    floor: str,
    map_dir: Path,
    dry_run: bool,
) -> None:
    maps = run / "maps"
    src_pgm = maps / f"{model_name}.pgm"
    src_yaml = maps / f"{model_name}.yaml"
    src_sem_png = maps / f"{model_name}_semantic.png"
    src_sem_yaml = maps / f"{model_name}_semantic.yaml"
    src_legend = maps / "semantic_legend.json"

    for required in (src_pgm, src_yaml):
        if not required.is_file():
            raise FileNotFoundError(f"missing occupancy map asset: {required}")

    out = map_dir / floor
    print(f"[map] {floor} <- {model_name} maps -> {out}")
    if dry_run:
        return

    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    shutil.copy2(src_pgm, out / f"{floor}.pgm")
    yaml_text = src_yaml.read_text(encoding="utf-8")
    (out / f"{floor}.yaml").write_text(
        _rewrite_yaml_image(yaml_text, f"{floor}.pgm"), encoding="utf-8"
    )

    if src_sem_png.is_file() and src_sem_yaml.is_file():
        shutil.copy2(src_sem_png, out / f"{floor}_semantic.png")
        sem_text = src_sem_yaml.read_text(encoding="utf-8")
        # Keep legend filename stable; rewrite image + legend refs.
        sem_text = _rewrite_yaml_image(sem_text, f"{floor}_semantic.png")
        if re.search(r"^legend:\s*.+$", sem_text, flags=re.M):
            sem_text = re.sub(
                r"^legend:\s*.+$",
                "legend: semantic_legend.json",
                sem_text,
                count=1,
                flags=re.M,
            )
        (out / f"{floor}_semantic.yaml").write_text(sem_text, encoding="utf-8")
        if src_legend.is_file():
            shutil.copy2(src_legend, out / "semantic_legend.json")
    else:
        print(f"[map] warn: no semantic pair for {model_name}; floor {floor} occupancy only")


def resolve_run(zip_path: Path | None, source: Path | None) -> tuple[Path, Path | None]:
    """Return (run_root, temp_dir_to_cleanup_or_None)."""
    if zip_path and source:
        raise SystemExit("pass only one of --zip / --source")
    if not zip_path and not source:
        raise SystemExit("need --zip or --source")

    if source:
        run = source if (source / "maps").is_dir() else _find_run_root(source)
        return run.resolve(), None

    tmp = Path(tempfile.mkdtemp(prefix="i2g_install_"))
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(tmp)
    return _find_run_root(tmp).resolve(), tmp


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zip", type=Path, help="image2gazebo export zip")
    ap.add_argument("--source", type=Path, help="extracted run dir (…/<timestamp>/)")
    ap.add_argument(
        "--floors",
        default=",".join(DEFAULT_FLOORS),
        help="comma-separated floor names under map/ (default: test_101..104)",
    )
    ap.add_argument(
        "--sim-model",
        type=Path,
        default=DEFAULT_SIM_MODEL,
        help="drawn_model directory to overwrite map.stl",
    )
    ap.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR)
    ap.add_argument("--skip-simulate", action="store_true", help="only write map floors")
    ap.add_argument("--skip-maps", action="store_true", help="only replace simulate STL")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    floors = [f.strip() for f in args.floors.split(",") if f.strip()]
    if not floors and not args.skip_maps:
        raise SystemExit("no floors specified")

    run, tmp = resolve_run(
        args.zip.resolve() if args.zip else None,
        args.source.resolve() if args.source else None,
    )
    try:
        model_name = _detect_model_name(run)
        print(f"[source] run={run}")
        print(f"[source] model_name={model_name}")

        if not args.skip_simulate:
            _install_simulate_mesh(run, model_name, args.sim_model.resolve(), args.dry_run)

        if not args.skip_maps:
            args.map_dir.resolve().mkdir(parents=True, exist_ok=True)
            for floor in floors:
                _install_floor(run, model_name, floor, args.map_dir.resolve(), args.dry_run)

        print("[done] rebuild simulate package (colcon) so share/ picks up new STL if installed.")
        print("[done] floors appear in GET /api/floors when <floor>/<floor>.pgm+.yaml exist.")
    finally:
        if tmp and tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
