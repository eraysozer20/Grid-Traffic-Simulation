# closure_impact.py
import os
import re
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# ----------------------- Config -----------------------

@dataclass
class ClosureImpactConfig:
    source_folder: str = "my_grid_sim"          # where baseline sensor_output.csv lives
    closure_parent: str = "closures"            # parent dir with closure_* folders
    out_dir: str = "combined_outputs/impact"    # where to save xlsx/csv/png
    impact_norm_max: float = 2.0                # color scale upper bound
    save_excel: bool = True
    save_csv: bool = True
    save_png: bool = True
    show_plot: bool = False                     # avoid blocking in pipeline

    # If you want to override baseline path:
    baseline_csv: Optional[str] = None

# ----------------------- Helpers -----------------------

# det_A2B2_0, det_-A2B2_1, det_A12B3#1_0, det_-A10B3#xyz_2
DET_PATTERN = re.compile(r"^det_-?([A-Z]\d+)([A-Z]\d+)(?:#[^_]*)?_\d+$")

def parse_edge_from_detector(det_id: str) -> Optional[str]:
    m = DET_PATTERN.match(det_id)
    if not m: return None
    src, dst = m.group(1), m.group(2)
    return f"{src}{dst}"

def to_numeric_df(df: pd.DataFrame) -> pd.DataFrame:
    df2 = df.copy()
    for c in df2.columns:
        df2[c] = pd.to_numeric(df2[c], errors="coerce").fillna(0)
    return df2

def load_and_aggregate(csv_path: str) -> pd.Series:
    """
    Load detector CSV (rows=sensor_id/lane, cols=minutes), sum across minutes,
    map each detector row to a canonical directed edge (e.g., 'A2B2'), and
    aggregate lanes → one value per directed edge.
    """
    df = pd.read_csv(csv_path, index_col=0)
    df = to_numeric_df(df)
    totals_per_detector = df.sum(axis=1)

    edges: List[str] = []
    vals:  List[float] = []
    for det_id, val in totals_per_detector.items():
        e = parse_edge_from_detector(det_id)
        if e is None:  # skip unparseable detector names
            continue
        edges.append(e); vals.append(val)

    if not edges:
        return pd.Series(dtype=float)

    ser = pd.Series(vals, index=edges, dtype=float)
    return ser.groupby(level=0).sum()

def edge_sort_key(edge_id: str):
    m = re.match(r"^([A-Z]\d+)([A-Z]\d+)$", edge_id)
    if not m: return (edge_id,)
    s, d = m.group(1), m.group(2)
    sx, sy = ord(s[0]) - ord('A'), int(s[1:])
    dx, dy = ord(d[0]) - ord('A'), int(d[1:])
    return (sx, sy, dx, dy)

def _plot_impact_matrix(impact_df: pd.DataFrame,
                        title: str,
                        vmax: float,
                        save_path: Optional[str],
                        show: bool):
    Z = impact_df.clip(lower=0.0, upper=vmax).to_numpy()
    norm = mcolors.TwoSlopeNorm(vmin=0.0, vcenter=1.0, vmax=vmax)

    fig, ax = plt.subplots(figsize=(12, 9))
    im = ax.imshow(Z, aspect="auto", cmap="seismic", norm=norm, interpolation="nearest")

    ax.set_title(title)
    ax.set_xlabel("Monitored directed edge (columns)")
    ax.set_ylabel("Removed directed edge (rows)")

    n_rows, n_cols = Z.shape
    xstep = max(1, n_cols // 12)
    ystep = max(1, n_rows // 25)
    ax.set_xticks(np.arange(0, n_cols, xstep))
    ax.set_xticklabels(impact_df.columns[::xstep], rotation=90, fontsize=8)
    ax.set_yticks(np.arange(0, n_rows, ystep))
    ax.set_yticklabels(impact_df.index[::ystep], fontsize=8)

    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Impact factor (removed / full)  •  0=blue, 1=white, 2=red")

    fig.tight_layout()
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)

# ----------------------- Public API -----------------------

def run_closure_impact(cfg: ClosureImpactConfig) -> Dict[str, str]:
    """
    Build the closure impact matrix (removed / full) across all closure_* runs.
    Saves xlsx/csv/png to cfg.out_dir. Returns a dict of output paths.
    """
    baseline_csv = cfg.baseline_csv or os.path.join(cfg.source_folder, "sensor_output.csv")
    if not os.path.exists(baseline_csv):
        raise FileNotFoundError(f"Baseline CSV not found: {baseline_csv}")
    if not os.path.isdir(cfg.closure_parent):
        raise FileNotFoundError(f"Closure parent folder not found: {cfg.closure_parent}")

    base_series = load_and_aggregate(baseline_csv)
    if base_series.empty:
        raise RuntimeError("No parseable detectors found in baseline CSV.")

    all_edges = sorted(base_series.index.unique(), key=edge_sort_key)

    closure_rows: List[str] = []
    impact_rows:  List[np.ndarray] = []

    for name in sorted(os.listdir(cfg.closure_parent)):
        folder = os.path.join(cfg.closure_parent, name)
        if not (os.path.isdir(folder) and name.startswith("closure_")):
            continue

        removed_edge = name[len("closure_"):]
        csv_path = os.path.join(folder, "sensor_output.csv")
        if not os.path.exists(csv_path):
            continue

        removed_series = load_and_aggregate(csv_path)

        rem  = removed_series.reindex(all_edges, fill_value=0.0)
        full = base_series   .reindex(all_edges, fill_value=0.0)

        impact = rem / full.replace(0, np.nan)
        impact = impact.fillna(0.0).clip(lower=0.0)  # 0/0 → 0

        closure_rows.append(removed_edge)
        impact_rows.append(impact.values)

    if not impact_rows:
        raise RuntimeError("No closure impact data found. Make sure closure folders contain sensor_output.csv files.")

    impact_df = pd.DataFrame(impact_rows, index=closure_rows, columns=all_edges)

    os.makedirs(cfg.out_dir, exist_ok=True)
    out_paths: Dict[str, str] = {}

    if cfg.save_excel:
        out_xlsx = os.path.join(cfg.out_dir, "impact_matrix.xlsx")
        impact_df.to_excel(out_xlsx, sheet_name="impact_matrix", index=True)
        out_paths["xlsx"] = out_xlsx

    if cfg.save_csv:
        out_csv = os.path.join(cfg.out_dir, "impact_matrix.csv")
        impact_df.to_csv(out_csv, index=True)
        out_paths["csv"] = out_csv

    if cfg.save_png:
        out_png = os.path.join(cfg.out_dir, "impact_matrix.png")
        _plot_impact_matrix(
            impact_df,
            title="Closure Impact Matrix (removed / full)",
            vmax=cfg.impact_norm_max,
            save_path=out_png,
            show=cfg.show_plot,
        )
        out_paths["png"] = out_png

    print(f"✅ Impact matrix built: {impact_df.shape[0]} closures × {impact_df.shape[1]} edges")
    for k, v in out_paths.items():
        print(f"   - {k}: {v}")

    return out_paths
