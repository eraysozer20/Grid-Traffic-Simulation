# distance_matrix.py
#!/usr/bin/env python3
"""
Build a sensor-to-sensor driving distance matrix using SUMO/TraCI.

- Reads sensors from detectors.add.xml (<e1Detector id="..." lane="EDGE_LANEID"...>).
- Uses SUMO routing (simulation.findRoute) to compute shortest-driving distance
  between each pair of sensors' edges (meters).
- Writes a square CSV with sensor IDs as both row/column headers.

You can call run_distance_matrix(...) from a master script or run this file directly.
"""

import os
import csv
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Optional

TRACI = None  # populated lazily by _ensure_traci()


# ========================== Utilities ==========================

def _ensure_traci(sumo_home: str):
    """Import traci after adding SUMO tools path; cache globally."""
    global TRACI
    if TRACI is not None:
        return TRACI

    if not sumo_home:
        raise EnvironmentError("SUMO_HOME is not set. Provide sumo_home to run_distance_matrix().")

    import sys
    tools_path = os.path.join(sumo_home, "tools")
    if tools_path not in sys.path:
        sys.path.append(tools_path)

    import traci  # type: ignore
    TRACI = traci
    return TRACI


def _lane_id_to_edge_id(lane_id: str) -> str:
    """
    Convert lane id like 'A1A0_0' -> edge id 'A1A0'.
    Works even if edge id contains underscores by splitting only the last suffix.
    """
    if "_" not in lane_id:
        return lane_id
    return lane_id.rsplit("_", 1)[0]


def load_sensors_from_detectors(detectors_xml: str) -> List[Tuple[str, str]]:
    """
    Returns [(sensor_id, edge_id), ...] in file order.
    Only unique sensor ids are kept; first occurrence wins.
    """
    tree = ET.parse(detectors_xml)
    root = tree.getroot()

    sensors: List[Tuple[str, str]] = []
    seen = set()

    for e1 in root.findall(".//e1Detector"):
        sid = e1.get("id") or e1.get("name")
        lane = e1.get("lane")
        if not sid or not lane or sid in seen:
            continue
        sensors.append((sid, _lane_id_to_edge_id(lane)))
        seen.add(sid)

    if not sensors:
        raise RuntimeError(f"No <e1Detector> entries found in {detectors_xml}")
    return sensors


def _start_sumo(net_file: str, sumo_home: str, use_gui: bool = False) -> None:
    """Start SUMO with just the network; routing only."""
    traci = _ensure_traci(sumo_home)
    exe = os.path.join(sumo_home, "bin", "sumo-gui" if use_gui else "sumo")
    if os.name == "nt":
        exe += ".exe"
    args = [
        exe,
        "-n", os.path.abspath(net_file),
        "--no-step-log",
        "--duration-log.disable",
        "--ignore-route-errors",
    ]
    traci.start(args)


def _stop_sumo() -> None:
    try:
        if TRACI:
            TRACI.close(False)
    except Exception:
        pass


def _route_distance_m(from_edge: str, to_edge: str, vtype: str) -> Optional[float]:
    """
    Ask SUMO for a route between edges and return distance in meters.
    Returns None if no route exists.
    """
    traci = TRACI
    if from_edge == to_edge:
        return 0.0
    try:
        # NOTE: correct kwarg is vType (not vTypeID)
        r = traci.simulation.findRoute(from_edge, to_edge, vType=vtype)
        dist = getattr(r, "length", None)
        if dist is None:
            # Fallback: sum edge lengths
            dist = 0.0
            for e in getattr(r, "edges", []):
                try:
                    dist += traci.edge.getLength(e)
                except traci.TraCIException:
                    return None
        if dist is None or dist != dist or dist <= 0:
            return None
        return float(dist)
    except traci.TraCIException:
        return None


def build_distance_matrix(
    sensors: List[Tuple[str, str]],
    vtype: str,
    assume_symmetric: bool = False,
    progress_every: int = 1,
) -> List[List[Optional[float]]]:
    """
    Returns an N x N matrix of distances (meters) where N = len(sensors).
    matrix[i][j] = distance from sensors[i] to sensors[j]

    If assume_symmetric=True, mirror distances to [j][i] to halve routing calls (for undirected).
    """
    n = len(sensors)
    mat: List[List[Optional[float]]] = [[None] * n for _ in range(n)]
    for i in range(n):
        mat[i][i] = 0.0

    for i, (sid_i, edge_i) in enumerate(sensors):
        if progress_every and (i % progress_every == 0):
            print(f"Routing from {sid_i} ({i+1}/{n})...")
        for j, (_sid_j, edge_j) in enumerate(sensors):
            if mat[i][j] is not None:
                continue
            d = _route_distance_m(edge_i, edge_j, vtype)
            mat[i][j] = d
            if assume_symmetric:
                mat[j][i] = d
    return mat


def write_matrix_csv(out_csv: str, sensors: List[Tuple[str, str]], matrix: List[List[Optional[float]]]) -> None:
    """
    Writes a square CSV: first row is ["sensor_id", <sensor ids...>]
    Each subsequent row begins with the row sensor_id, then distances.
    Distances are written as integers (meters) if close to int, else float.
    Missing routes are written as empty cells.
    """
    sensor_ids = [sid for sid, _edge in sensors]
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)) or ".", exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["sensor_id"] + sensor_ids)
        for i, sid in enumerate(sensor_ids):
            row = [sid]
            for j in range(len(sensor_ids)):
                d = matrix[i][j]
                if d is None:
                    row.append("")
                else:
                    if abs(d - round(d)) < 1e-6:
                        row.append(str(int(round(d))))
                    else:
                        row.append(f"{d:.3f}")
            w.writerow(row)


# ========================== Public entrypoint ==========================

def run_distance_matrix(
    sumo_home: str,
    net_file: str,
    detectors_file: str,
    out_csv: str,
    vtype_id: str = "DEFAULT_VEHTYPE",
    assume_symmetric: bool = False,
    gui: bool = False,
    cap_sensors: Optional[int] = None,
    progress_every: int = 1,
) -> dict:
    """
    Orchestrates loading detectors, starting SUMO, computing the matrix, and writing CSV.

    Args:
        sumo_home: SUMO installation directory (must contain /tools and /bin).
        net_file: path to .net.xml.
        detectors_file: path to detectors.add.xml used in your runs.
        out_csv: destination CSV path.
        vtype_id: vehicle type id for routing (must be allowed on edges).
        assume_symmetric: if True, mirror distances to reduce routing calls.
        gui: start sumo-gui instead of sumo (useful for debugging).
        cap_sensors: if provided, only use the first N sensors.
        progress_every: print progress every N source sensors.

    Returns:
        dict with keys: sensors, out_csv, n, assume_symmetric
    """
    sensors = load_sensors_from_detectors(detectors_file)
    if cap_sensors is not None:
        sensors = sensors[:int(cap_sensors)]
    print(f"Loaded {len(sensors)} sensors from {detectors_file}")

    _start_sumo(net_file, sumo_home, use_gui=gui)
    try:
        mat = build_distance_matrix(
            sensors=sensors,
            vtype=vtype_id,
            assume_symmetric=assume_symmetric,
            progress_every=progress_every,
        )
    finally:
        _stop_sumo()

    write_matrix_csv(out_csv, sensors, mat)
    print(f"\n✅ Distance matrix written to: {out_csv}")
    print("   Rows/Cols:", len(sensors))
    print("   Note: blank cells mean no drivable route existed for that pair.")

    return {
        "sensors": [sid for sid, _ in sensors],
        "out_csv": out_csv,
        "n": len(sensors),
        "assume_symmetric": assume_symmetric,
    }


# ========================== Standalone CLI ==========================

if __name__ == "__main__":
    # Defaults (edit or override via master script)
    SUMO_HOME = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
    NET_FILE  = r"my_grid_sim\MySUMOFile.net.xml"
    DETECTORS = r"month_run\config\detectors.add.xml"
    OUT_CSV   = r"combined_outputs\sensor_distance_matrix.csv"
    VTYPE_ID  = "DEFAULT_VEHTYPE"

    run_distance_matrix(
        sumo_home=SUMO_HOME,
        net_file=NET_FILE,
        detectors_file=DETECTORS,
        out_csv=OUT_CSV,
        vtype_id=VTYPE_ID,
        assume_symmetric=False,  # set True if your network is effectively undirected
        gui=False,
        cap_sensors=None,
        progress_every=1,
    )
