# clean_grid.py
import os
import re
import csv
import json
import math
import random
import shutil
import subprocess
import pandas as pd
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple
from concurrent.futures import ProcessPoolExecutor, as_completed

# ==================== Config ====================

@dataclass
class CleanGridConfig:
    # Grid / sim
    grid_size: int = 6
    edge_length: int = 300
    sim_duration: int = 3600 * 24       # seconds
    sensor_freq: int = 300              # seconds between measurements
    trip_period_sec: float = 2.0        # avg seconds between departures

    # Paths
    source_folder: str = "my_grid_sim"  # where plain.* + MySUMOFile.net.xml live
    closure_parent: str = "closures"    # parent for closure_* dirs
    portal_weights_file: str = "portal_weights.json"

    # Weights
    weight_mode: str = "uniform"        # "uniform" | "random" | "custom"
    custom_weights: Dict[str, Dict[str, float]] = None  # {"origin": {...}, "sink": {...}}

    # Execution
    use_gui_full: bool = True
    use_gui_removal: bool = False
    run_closures_in_parallel: bool = True
    max_workers: int = 6
    skip_existing: bool = True
    random_seed: int = 43

    # Misc
    protected_portal_prefix: str = "portal_"

    def __post_init__(self):
        if self.custom_weights is None:
            self.custom_weights = {"origin": {}, "sink": {}}

# ==================== Utilities ====================

def run_command(cmd_list: List[str]) -> subprocess.CompletedProcess:
    print("▶", " ".join(cmd_list))
    result = subprocess.run(cmd_list, capture_output=True, text=True)
    if result.returncode != 0:
        print("❌ Error:\n", result.stderr)
        raise subprocess.CalledProcessError(result.returncode, cmd_list)
    return result

# ---------- Portal augmentation ----------

def _read_plain_nodes(plain_nod_path: str):
    tree = ET.parse(plain_nod_path)
    root = tree.getroot()
    pos = {}
    for n in root.findall("node"):
        nid = n.attrib.get("id")
        x = n.attrib.get("x")
        y = n.attrib.get("y")
        if nid and x and y:
            pos[nid] = (float(x), float(y))
    return tree, root, pos

def _add_boundary_portals(plain_dir: str, edge_length: int, protected_prefix: str):
    """
    For every boundary node in plain.nod.xml:
      - create a portal node slightly outside the grid
      - add 2 edges in plain.edg.xml: portal->boundary and boundary->portal
    Skips duplicates if already present (id-based).
    """
    spur_len = edge_length * 0.5

    nod_path = os.path.join(plain_dir, "plain.nod.xml")
    edg_path = os.path.join(plain_dir, "plain.edg.xml")

    nod_tree, nod_root, pos = _read_plain_nodes(nod_path)
    if not pos:
        raise RuntimeError("No node positions found in plain.nod.xml")

    xs = [p[0] for p in pos.values()]
    ys = [p[1] for p in pos.values()]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    eps = max(1e-6, 0.001 * max(max_x - min_x, max_y - min_y))

    def side_of(nid):
        x, y = pos[nid]
        if abs(x - min_x) <= eps: return "L"
        if abs(x - max_x) <= eps: return "R"
        if abs(y - min_y) <= eps: return "B"
        if abs(y - max_y) <= eps: return "T"
        return None

    existing_node_ids = {n.attrib.get("id") for n in nod_root.findall("node")}
    edg_tree = ET.parse(edg_path)
    edg_root = edg_tree.getroot()
    existing_edge_ids = {e.attrib.get("id") for e in edg_root.findall("edge")}

    new_nodes = 0
    new_edges = 0

    for nid in pos:
        side = side_of(nid)
        if side is None:
            continue

        x, y = pos[nid]
        if side == "L":   px, py = x - spur_len, y
        elif side == "R": px, py = x + spur_len, y
        elif side == "B": px, py = x, y - spur_len
        else:             px, py = x, y + spur_len

        portal_id = f"{protected_prefix}{nid}"

        if portal_id not in existing_node_ids:
            n_elem = ET.Element("node", id=portal_id, x=str(px), y=str(py))
            nod_root.append(n_elem)
            existing_node_ids.add(portal_id)
            new_nodes += 1

        e_in_id  = f"portal_to_{nid}"
        e_out_id = f"{nid}_to_portal"

        if e_in_id not in existing_edge_ids:
            e_in = ET.Element("edge", id=e_in_id, **{"from": portal_id, "to": nid, "numLanes": "1", "speed": "13.9"})
            edg_root.append(e_in)
            existing_edge_ids.add(e_in_id)
            new_edges += 1

        if e_out_id not in existing_edge_ids:
            e_out = ET.Element("edge", id=e_out_id, **{"from": nid, "to": portal_id, "numLanes": "1", "speed": "13.9"})
            edg_root.append(e_out)
            new_edges += 1

    nod_tree.write(nod_path, encoding="utf-8", xml_declaration=True)
    edg_tree.write(edg_path, encoding="utf-8", xml_declaration=True)
    print(f"🟢 Added {new_nodes} portal nodes and {new_edges} portal edges to {plain_dir}")

def _rebuild_net_from_plain(plain_dir: str, out_net_path: str):
    type_file = "plain.typ.xml" if os.path.exists(os.path.join(plain_dir, "plain.typ.xml")) else "plain.con.xml"
    args = [
        "netconvert",
        "--node-files", os.path.join(plain_dir, "plain.nod.xml"),
        "--edge-files", os.path.join(plain_dir, "plain.edg.xml"),
        "--type-files", os.path.join(plain_dir, type_file),
        "--output-file", out_net_path
    ]
    if os.path.exists(os.path.join(plain_dir, "plain.tll.xml")):
        args.extend(["--tls.guess", "true"])
    run_command(args)
    print(f"✅ Rebuilt .net.xml with portals: {out_net_path}")

def _collect_portal_edges(net_file: str, protected_prefix: str) -> Tuple[List[str], List[str]]:
    """Return (origin_edges: portal->boundary, sink_edges: boundary->portal)."""
    tree = ET.parse(net_file)
    root = tree.getroot()
    origin_edges, sink_edges = [], []
    for e in root.findall("edge"):
        eid = e.attrib.get("id", "")
        if eid.startswith(":"):
            continue
        frm = e.attrib.get("from", "")
        to  = e.attrib.get("to", "")
        if frm.startswith(protected_prefix) and not to.startswith(protected_prefix):
            origin_edges.append(eid)
        if to.startswith(protected_prefix) and not frm.startswith(protected_prefix):
            sink_edges.append(eid)
    if not origin_edges or not sink_edges:
        raise RuntimeError("Portal edges not found; did portal augmentation run?")
    return origin_edges, sink_edges

# ---------- Weights ----------

def _normalize(ws: List[float]) -> List[float]:
    s = sum(ws)
    return [w / s for w in ws] if s > 0 else [1.0/len(ws)] * len(ws)

def _load_or_make_portal_weights(folder: str, origin_edges: List[str], sink_edges: List[str], cfg: CleanGridConfig):
    path = os.path.join(folder, cfg.portal_weights_file)
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        if cfg.weight_mode == "custom":
            data = {"origin": cfg.custom_weights.get("origin", {}), "sink": cfg.custom_weights.get("sink", {})}
        elif cfg.weight_mode == "uniform":
            data = {"origin": {e: 1.0 for e in origin_edges},
                    "sink":   {e: 1.0 for e in sink_edges}}
        else:  # random
            data = {"origin": {e: random.expovariate(1.0) for e in origin_edges},
                    "sink":   {e: random.expovariate(1.0) for e in sink_edges}}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    ow = [float(data["origin"].get(e, 1.0)) for e in origin_edges]
    sw = [float(data["sink"].get(e, 1.0))   for e in sink_edges]
    return _normalize(ow), _normalize(sw), path

# ---------- Network & I/O ----------

def generate_grid_with_plain(output_folder: str, cfg: CleanGridConfig) -> str:
    os.makedirs(output_folder, exist_ok=True)
    tmp_net = os.path.join(output_folder, "MySUMOFile.net.xml")
    plain_prefix = os.path.join(output_folder, "plain")
    run_command([
        "netgenerate",
        "--grid",
        f"--grid.number={cfg.grid_size}",
        f"--grid.length={cfg.edge_length}",
        f"--plain-output-prefix={plain_prefix}",
        f"--output-file={tmp_net}"
    ])
    print(f"✅ Network generated: {tmp_net}")
    print(f"✅ Plain files generated with prefix: {plain_prefix}")

    # Augment portals and rebuild net
    _add_boundary_portals(output_folder, edge_length=cfg.edge_length, protected_prefix=cfg.protected_portal_prefix)
    net_file = os.path.join(output_folder, "MySUMOFile.net.xml")
    _rebuild_net_from_plain(output_folder, net_file)
    return net_file

def generate_trips(net_file: str, output_folder: str, cfg: CleanGridConfig) -> str:
    """
    Create trips that start on portal->boundary edges and end on boundary->portal edges.
    Weighted sampling via portal_weights.json (created if missing).
    """
    trips_file = os.path.join(output_folder, "trips.rou.xml")
    os.makedirs(output_folder, exist_ok=True)

    origin_edges, sink_edges = _collect_portal_edges(net_file, cfg.protected_portal_prefix)
    origin_probs, sink_probs, weights_path = _load_or_make_portal_weights(output_folder, origin_edges, sink_edges, cfg)

    total_trips = max(1, int(cfg.sim_duration / max(0.1, cfg.trip_period_sec)))
    dep_times = [random.uniform(0, cfg.sim_duration) for _ in range(total_trips)]
    dep_times.sort()

    with open(trips_file, "w", encoding="utf-8") as f:
        f.write('<routes>\n')
        for i, t in enumerate(dep_times):
            fr = random.choices(origin_edges, weights=origin_probs, k=1)[0]
            to = fr
            for _ in range(3):
                to = random.choices(sink_edges, weights=sink_probs, k=1)[0]
                if to != fr:
                    break
            f.write(f'  <trip id="t{i}" depart="{t:.2f}" from="{fr}" to="{to}"/>\n')
        f.write('</routes>\n')

    print(f"✅ Weighted portal trips: {trips_file} (weights: {os.path.basename(weights_path)})")
    return trips_file

def route_trips(net_file: str, trips_file: str, output_folder: str) -> str:
    routed_file = os.path.join(output_folder, "duarouter.rou.xml")
    run_command([
        "duarouter",
        "--net-file", net_file,
        "--route-files", trips_file,
        "--output-file", routed_file,
        "--ignore-errors"
    ])
    print(f"✅ Routes generated at: {routed_file}")
    return routed_file

def add_manual_sensors(net_file: str, output_file: str, sensor_output_file: str, cfg: CleanGridConfig):
    tree = ET.parse(net_file)
    root = tree.getroot()
    detectors = ET.Element("additional")

    for edge in root.findall("edge"):
        eid = edge.attrib.get("id", "")
        if eid.startswith(":"):
            continue
        # exclude portal spur edges
        if eid.startswith("portal_to_") or eid.endswith("_to_portal"):
            continue
        for lane in edge.findall("lane"):
            ET.SubElement(detectors, "e1Detector", {
                "id": f"det_{lane.attrib['id']}",
                "lane": lane.attrib["id"],
                "pos": str(float(lane.attrib.get("length", "0")) / 2),
                "freq": str(cfg.sensor_freq),
                "file": os.path.basename(sensor_output_file)
            })
    ET.ElementTree(detectors).write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"✅ Manual detectors written to: {output_file} (portal edges excluded)")

def generate_sumocfg(net_file: str, route_file: str, detector_file: str, output_file: str):
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="{os.path.basename(net_file)}"/>
        <route-files value="{os.path.basename(route_file)}"/>
        <additional-files value="{os.path.basename(detector_file)}"/>
    </input>
</configuration>
""")
    print(f"✅ SUMO config created at: {output_file}")

def _run_sumo_cfg(cfg_file: str, use_gui: bool):
    folder = os.path.dirname(cfg_file)
    filename = os.path.basename(cfg_file)
    cwd = os.getcwd()
    try:
        os.chdir(folder)
        cmd = ["sumo-gui" if use_gui else "sumo", "-c", filename]
        subprocess.run(cmd, check=True)
    finally:
        os.chdir(cwd)

def convert_detector_xml_to_csv(xml_file: str, output_csv: str, expected_sensors: Optional[List[str]] = None):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    records = []
    for interval in root.findall("interval"):
        begin_sec = float(interval.attrib.get("begin", "0"))
        sensor_id = interval.attrib.get("id", "")
        # SUMO sometimes writes floats; cast safely
        count = int(float(interval.attrib.get("nVehEntered", 0)))
        minute = int(begin_sec // 60)
        records.append({"minute": minute, "sensor_id": sensor_id, "vehicle_count": count})

    df = (   # wide pivot
        (pd.DataFrame(records)
           .pivot(index="sensor_id", columns="minute", values="vehicle_count")
           .fillna(0)
           .astype(int))
        if records else pd.DataFrame()
    )

    if df.empty:
        os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)
        df.to_csv(output_csv)
        print(f"⚠️ No intervals found; wrote empty CSV: {output_csv}")
        return

    # pad missing sensors to align shapes across runs
    if expected_sensors is not None and len(df.columns) > 0:
        missing = [s for s in expected_sensors if s not in df.index]
        if missing:
            for s in missing:
                df.loc[s] = [0] * len(df.columns)
        df = df.sort_index()

    os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)
    # make minute columns strings for consistency
    df.columns = [f"{c}" for c in df.columns]
    df.to_csv(output_csv)
    print(f"✅ Sensor data saved to CSV: {output_csv}")

# ---------- Baseline ----------

def run_full_grid_simulation(cfg: CleanGridConfig) -> Tuple[str, List[str]]:
    """
    Returns: (net_file_path, full_sensor_ids_from_baseline)
    """
    print("🏁 Starting full grid simulation...")
    net_file = generate_grid_with_plain(cfg.source_folder, cfg)
    trips_file = generate_trips(net_file, cfg.source_folder, cfg)
    routed_file = route_trips(net_file, trips_file, cfg.source_folder)
    detector_file = os.path.join(cfg.source_folder, "detectors.add.xml")
    sensor_output_file = os.path.join(cfg.source_folder, "sensor_output.xml")
    add_manual_sensors(net_file, detector_file, sensor_output_file, cfg)
    sumocfg_file = os.path.join(cfg.source_folder, "sumo.sumocfg")
    generate_sumocfg(net_file, routed_file, detector_file, sumocfg_file)
    _run_sumo_cfg(sumocfg_file, use_gui=cfg.use_gui_full)

    sensor_csv_file = os.path.join(cfg.source_folder, "sensor_output.csv")
    convert_detector_xml_to_csv(sensor_output_file, sensor_csv_file)
    if os.path.exists(sensor_csv_file) and os.path.getsize(sensor_csv_file) > 0:
        sensor_ids = list(pd.read_csv(sensor_csv_file, index_col=0).index)
    else:
        sensor_ids = []

    print("✅ Full simulation pipeline completed.")
    return net_file, sensor_ids

# ---------- Closures ----------

def remove_edge_from_plain_files(edge_id: str, cfg: CleanGridConfig) -> Tuple[str, str]:
    """
    Create isolated net in closures/closure_<edge>/ with the edge removed.
    Portal edges ('portal_to_*' or '*_to_portal') are protected and will NOT be removed.
    Returns: (closure_folder, updated_net_path)
    """
    os.makedirs(cfg.closure_parent, exist_ok=True)
    closure_folder = os.path.join(cfg.closure_parent, f"closure_{edge_id}")
    os.makedirs(closure_folder, exist_ok=True)

    typ_file = "plain.typ.xml" if os.path.exists(os.path.join(cfg.source_folder, "plain.typ.xml")) else "plain.con.xml"

    # Copy plain files for isolated netconvert
    for fname in ["plain.nod.xml", "plain.edg.xml", typ_file]:
        shutil.copyfile(os.path.join(cfg.source_folder, fname), os.path.join(closure_folder, fname))

    tls_args = []
    tls_src = os.path.join(cfg.source_folder, "plain.tll.xml")
    if os.path.exists(tls_src):
        shutil.copyfile(tls_src, os.path.join(closure_folder, "plain.tll.xml"))
        tls_args = ["--tls.guess", "true"]

    # Protect portal edges from removal
    is_portal_edge = edge_id.startswith("portal_to_") or edge_id.endswith("_to_portal")

    input_edg = os.path.join(closure_folder, "plain.edg.xml")
    tree = ET.parse(input_edg)
    root = tree.getroot()
    original_count = len(root.findall("edge"))

    if not is_portal_edge:
        root[:] = [edge for edge in root if edge.attrib.get("id") != edge_id]
        tree.write(input_edg)
        print(f"✅ Removed edge {edge_id} from edge file ({original_count} → {len(root.findall('edge'))})")
    else:
        print(f"🔒 Portal edge '{edge_id}' is protected; no removal performed.")

    _add_boundary_portals(closure_folder, edge_length=cfg.edge_length, protected_prefix=cfg.protected_portal_prefix)

    updated_net = os.path.join(closure_folder, f"net_no_{edge_id}.net.xml")
    args = [
        "netconvert",
        "--node-files", os.path.join(closure_folder, "plain.nod.xml"),
        "--edge-files", input_edg,
        "--type-files", os.path.join(closure_folder, typ_file),
        *tls_args,
        "--output-file", updated_net
    ]
    run_command(args)
    return closure_folder, updated_net

def _copy_weights_if_present(src_folder: str, dest_folder: str, cfg: CleanGridConfig):
    src_weights = os.path.join(src_folder, cfg.portal_weights_file)
    dst_weights = os.path.join(dest_folder, cfg.portal_weights_file)
    if os.path.exists(src_weights) and not os.path.exists(dst_weights):
        shutil.copyfile(src_weights, dst_weights)

def run_simulation_with_edge_removed(edge_id: str, net_file: str, expected_sensors: List[str], cfg: CleanGridConfig):
    """Sequential version (honors cfg.use_gui_removal)."""
    closure_folder, net_path = remove_edge_from_plain_files(edge_id, cfg)
    _copy_weights_if_present(cfg.source_folder, closure_folder, cfg)
    trips = generate_trips(net_path, closure_folder, cfg)
    routes = route_trips(net_path, trips, closure_folder)
    detectors_file = os.path.join(closure_folder, "detectors.add.xml")
    sensor_output_file = os.path.join(closure_folder, "sensor_output.xml")
    add_manual_sensors(net_path, detectors_file, sensor_output_file, cfg)
    cfg_file = os.path.join(closure_folder, "sumo.sumocfg")
    generate_sumocfg(net_path, routes, detectors_file, cfg_file)
    _run_sumo_cfg(cfg_file, use_gui=cfg.use_gui_removal)
    convert_detector_xml_to_csv(
        sensor_output_file,
        os.path.join(closure_folder, "sensor_output.csv"),
        expected_sensors=expected_sensors
    )

# --- Parallel worker ---

def _run_one_closure(edge_id: str, expected_sensors: List[str], cfg_dict: dict) -> Tuple[str, str]:
    """Worker for parallel closure (always headless). Returns (edge_id, status)."""
    # Reconstruct config (safe, picklable)
    cfg = CleanGridConfig(**cfg_dict)
    try:
        closure_folder = os.path.join(cfg.closure_parent, f"closure_{edge_id}")
        if cfg.skip_existing and os.path.exists(os.path.join(closure_folder, "sensor_output.csv")):
            return (edge_id, "skipped (exists)")

        # Build isolated network with the edge removed
        closure_folder, net_file = remove_edge_from_plain_files(edge_id, cfg)
        _copy_weights_if_present(cfg.source_folder, closure_folder, cfg)

        # Trips → routes
        trips  = generate_trips(net_file, closure_folder, cfg)
        routes = route_trips(net_file, trips, closure_folder)

        # Detectors + SUMO cfg
        detectors_file     = os.path.join(closure_folder, "detectors.add.xml")
        sensor_output_file = os.path.join(closure_folder, "sensor_output.xml")
        add_manual_sensors(net_file, detectors_file, sensor_output_file, cfg)

        cfg_file = os.path.join(closure_folder, "sumo.sumocfg")
        # 🔧 Create the per-closure SUMO config (uses basenames)
        generate_sumocfg(net_file=net_file, route_file=routes, detector_file=detectors_file, output_file=cfg_file)

        # ✅ Run SUMO headless from the closure folder so relative paths resolve
        proc = subprocess.run(
            ["sumo", "-c", "sumo.sumocfg"],
            cwd=closure_folder,
            text=True,
            capture_output=True,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"SUMO failed for {edge_id}: {proc.stderr.strip()}")

        # Convert detector XML → CSV (pad to expected sensors)
        convert_detector_xml_to_csv(
            sensor_output_file,
            os.path.join(closure_folder, "sensor_output.csv"),
            expected_sensors=expected_sensors
        )
        return (edge_id, "ok")

    except Exception as e:
        return (edge_id, f"error: {e}")


def collect_edge_ids(net_file: str) -> List[str]:
    tree = ET.parse(net_file)
    root = tree.getroot()
    ids = []
    for e in root.findall("edge"):
        eid = e.attrib.get("id", "")
        if eid.startswith(":"):
            continue
        if eid.startswith("portal_to_") or eid.endswith("_to_portal"):
            continue
        ids.append(eid)
    return ids

def run_simulations_for_all_edges_sequential(net_file: str, expected_sensors: List[str], cfg: CleanGridConfig):
    edge_ids = collect_edge_ids(net_file)
    print(f"🔍 Found {len(edge_ids)} edges in the network (sequential).")
    for i, edge_id in enumerate(edge_ids, 1):
        print(f"\n🚧 [{i}/{len(edge_ids)}] Removing edge: {edge_id}")
        try:
            run_simulation_with_edge_removed(edge_id, net_file, expected_sensors, cfg)
        except Exception as e:
            print(f"⚠️ Skipped edge {edge_id} due to error: {e}")

def run_simulations_for_all_edges_parallel(net_file: str, expected_sensors: List[str], cfg: CleanGridConfig) -> Dict[str, str]:
    edge_ids = collect_edge_ids(net_file)
    print(f"🔍 Found {len(edge_ids)} edges in the network (parallel, workers={cfg.max_workers}).")
    results: Dict[str, str] = {}
    with ProcessPoolExecutor(max_workers=cfg.max_workers) as ex:
        futures = {ex.submit(_run_one_closure, edge, expected_sensors, asdict(cfg)): edge for edge in edge_ids}
        for fut in as_completed(futures):
            edge = futures[fut]
            try:
                eid, status = fut.result()
            except Exception as e:
                eid, status = edge, f"error: {e}"
            results[eid] = status
            print(f"➡ {eid}: {status}")
    ok = sum(1 for s in results.values() if s == "ok")
    skipped = sum(1 for s in results.values() if s.startswith("skipped"))
    errs = [(e, s) for e, s in results.items() if s.startswith("error")]
    print(f"\n✅ Done. OK={ok}, Skipped={skipped}, Errors={len(errs)}")
    if errs:
        print("Errors (first 10):")
        for e, s in list(errs)[:10]:
            print(" -", e, s)
    return results

# ==================== Orchestrator ====================

def run_clean_grid(cfg: CleanGridConfig) -> dict:
    """
    Full pipeline:
      1) Build grid with portals, generate weighted trips, baseline run (open network)
      2) For every non-portal edge, create closure scenario and run a day
    Returns a summary dict.
    """
    # set seed once (child processes will inherit a copy on POSIX; on Windows each starts fresh)
    random.seed(cfg.random_seed)

    net_file, sensor_ids = run_full_grid_simulation(cfg)

    if cfg.run_closures_in_parallel:
        statuses = run_simulations_for_all_edges_parallel(net_file, sensor_ids, cfg)
    else:
        run_simulations_for_all_edges_sequential(net_file, sensor_ids, cfg)
        statuses = {}

    summary = {
        "net_file": net_file,
        "baseline_sensor_count": len(sensor_ids),
        "closure_mode": "parallel" if cfg.run_closures_in_parallel else "sequential",
        "max_workers": cfg.max_workers if cfg.run_closures_in_parallel else 1,
        "closures_status": statuses,
    }
    return summary

# ==================== Standalone ====================

if __name__ == "__main__":
    # Defaults mirror your original script
    cfg = CleanGridConfig(
        grid_size=6,
        edge_length=300,
        sim_duration=3600*24,
        sensor_freq=300,
        trip_period_sec=2.0,
        source_folder="my_grid_sim",
        closure_parent="closures",
        portal_weights_file="portal_weights.json",
        weight_mode="uniform",          # "uniform" | "random" | "custom"
        custom_weights={"origin": {}, "sink": {}},
        use_gui_full=True,
        use_gui_removal=False,
        run_closures_in_parallel=True,
        max_workers=6,
        skip_existing=True,
        random_seed=43,
    )

    # 1) Baseline (open network) + 2) closures (all non-portal edges)
    run_clean_grid(cfg)
