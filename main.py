# main.py
import os
import sys
from typing import Optional

# =========================
# 🔧 TOGGLES (no CLI args)
# =========================
RUN_MONTHLY = True              # run the long month/day scheduler step?
SCHEDULE_MODE = "months"          # "days" or "months" (used only if RUN_MONTHLY=True)

# If SCHEDULE_MODE == "days"
DAYS_TO_RUN = 5
START_YEAR  = 2025
START_MONTH = 1
START_DAY   = 1

# If SCHEDULE_MODE == "months"
MONTHS_TO_RUN = 2               # number of months to simulate
MONTHS_START_YEAR  = 2025
MONTHS_START_MONTH = 1

# ======================================
# 📁 DIRECTORIES — EDIT THESE IN ONE PLACE
# ======================================
RUN_NAME = "2MON_COT"
PROJECT_ROOT = os.path.abspath(os.path.dirname(__file__))

# 👇 NEW: force everything under PROJECT_ROOT/data
DATA_ROOT = os.path.join(PROJECT_ROOT, "data")

# Clean-grid / baseline assets and closures
DIR_SOURCE_NET   = os.path.join(DATA_ROOT, "my_grid_sim")                 # clean_grid writes net & detectors here
DIR_CLOSURES     = os.path.join(DATA_ROOT, "closures")                    # per-edge closure sims from clean_grid
PATH_PORTAL_WEIGHTS = os.path.join(DIR_SOURCE_NET, "portal_weights.json") # portal weights used in both runners

# Monthly/Days runner (its outputs live under run_dir/months/YYYY-MM/day_##)
DIR_MONTH_RUN        = os.path.join(DATA_ROOT, RUN_NAME)
DIR_MONTHS_UNDER_RUN = os.path.join(DIR_MONTH_RUN, "months")
DIR_MONTH_CONFIG     = os.path.join(DIR_MONTH_RUN, "config")
DIR_MONTH_MERGED     = os.path.join(DIR_MONTH_RUN, "merged")              # where merged outputs go (labels/readings)

# If you already have an external months tree (when RUN_MONTHLY=False)
DIR_EXISTING_MONTHS = os.path.join(DATA_ROOT, RUN_NAME, "months")
DIR_EXISTING_MERGED = os.path.join(DATA_ROOT, "OT", "merged")

# Combined artifacts
DIR_COMBINED_OUT   = os.path.join(DATA_ROOT, "combined_outputs")
PATH_DISTANCE_CSV  = os.path.join(DIR_COMBINED_OUT, "sensor_distance_matrix.csv")
DIR_DESPT_OUT      = os.path.join(DIR_COMBINED_OUT, "DESPT", RUN_NAME)

# Optional (if you later compute a closure effect matrix CSV to convert)
PATH_EFFECT_MATRIX_CSV: Optional[str] = None   # e.g., os.path.join(DIR_COMBINED_OUT, "closure_effect_matrix.csv")

# SUMO
SUMO_HOME = os.environ.get("SUMO_HOME", r"C:\Program Files (x86)\Eclipse\Sumo")
VTYPE_ID  = "DEFAULT_VEHTYPE"

# ========= Imports for pipeline stages (expect the refactored modules) =========
import clean_grid
import distance_matrix
import closure_labeling
import compare_results as closure_impact
import xml_csv_converter          # must expose run_xml_to_csv(...)
import csv_merger                 # must expose run_csv_merger(...)
import create_DESPT_input         # must expose DesptConfig + run_despt_prep(...)
import closure_month_runner as cmr  # MonthRunnerConfig + run_closure_month_runner(...)


def run_all():
    # Make sure high-level dirs exist
    for d in [
        DATA_ROOT,
        DIR_SOURCE_NET,
        DIR_COMBINED_OUT,
        DIR_CLOSURES,
        DIR_MONTH_RUN,
        DIR_MONTH_CONFIG,
    ]:
        os.makedirs(d, exist_ok=True)

    # =====================================================
    # 1) CLEAN GRID: baseline + per-edge closure simulations
    # =====================================================
    print("\n=== [1/7] CLEAN GRID: baseline + per-edge closure simulations ===")
    cfg_clean = clean_grid.CleanGridConfig(
        grid_size=6,
        edge_length=300,
        sim_duration=3600*24,         # 1 day
        sensor_freq=300,              # 5 minutes
        trip_period_sec=2.0,          # demand knob → ~0.5 veh/s
        source_folder=DIR_SOURCE_NET, # centralized
        closure_parent=DIR_CLOSURES,  # centralized
        portal_weights_file=PATH_PORTAL_WEIGHTS,
        weight_mode="uniform",        # "uniform" | "random" | "custom"
        custom_weights={"origin": {}, "sink": {}},
        use_gui_full=False,
        use_gui_removal=False,
        run_closures_in_parallel=True,
        max_workers=6,
        skip_existing=True,
        random_seed=43,
    )
    summary_clean = clean_grid.run_clean_grid(cfg_clean)
    net_file = summary_clean.get("net_file")
    if not net_file or not os.path.exists(net_file):
        print("❌ Could not find net file from clean_grid stage; aborting.")
        sys.exit(1)

    # Detectors produced by clean_grid:
    baseline_detectors_file = os.path.join(DIR_SOURCE_NET, "detectors.add.xml")

    print("\n=== [2a] CLOSURE IMPACT (removed / full) ===")
    impact_cfg = closure_impact.ClosureImpactConfig(
        source_folder=DIR_SOURCE_NET,
        closure_parent=DIR_CLOSURES,
        out_dir=os.path.join(DIR_COMBINED_OUT, "impact"),
        impact_norm_max=2.0,
        save_excel=True,
        save_csv=True,
        save_png=True,
        show_plot=False,           # do not block the pipeline
    )
    _impact_paths = closure_impact.run_closure_impact(impact_cfg)
    PATH_EFFECT_MATRIX_CSV = _impact_paths.get("csv")

    # ===========================================
    # 2) MONTHLY/DAY RUNNER (optional, long-running)
    # ===========================================
    if RUN_MONTHLY:
        print("\n=== [2/7] MONTHLY/DAY RUNNER (portal-only) ===")

        # Build the runner config purely from globals above
        if SCHEDULE_MODE.lower() == "days":
            month_cfg = cmr.MonthRunnerConfig(
                run_mode="days",
                start_year=START_YEAR, start_month=START_MONTH, start_day=START_DAY,
                days_to_run=int(DAYS_TO_RUN),

                sumo_home=SUMO_HOME,
                use_gui=False,
                step_length=1.0,
                bin_sec=300,
                net_dir=DIR_SOURCE_NET,
                run_dir=DIR_MONTH_RUN,
                sim_seconds=24 * 3600,

                # Demand
                demand_mode="daily_target",
                base_spawn_rate_total=0.8,
                target_daily_veh=86400,
                scale_safety=1.00,
                max_veh_per_sec_total=2.0,
                diurnal_peak_clamp=None,

                # --- Closures (legacy + new controls) ---
                n_closures=5,                      # legacy, no longer used directly
                close_min_dur_s=30 * 60,
                close_max_dur_s=8 * 60 * 60,
                close_start_guard_s=10 * 60,
                exclude_portal_edges_from_closure=True,
                allow_cross_day=True,
                retarget_before_reroute=True,

                # New global closure controls:
                # - pick 5 edges once; only these can ever be closed
                # - every day is a closure day (period=1)
                # - each edge has 50% chance to be closed on a closure day
                closure_pool_size=5,
                closure_day_period=1,
                closure_edge_prob=0.5,

                # Late subset: 2 of those 5 edges only appear in the last 30% of the schedule
                late_closure_pool_size=2,
                late_closure_start_frac=0.7,

                # Date gates
                closure_global_start=(2025, 1, 1),    # (Y, M, D)
                deferred_closure_start=(2024, 3, 1),  # (Y, M, D)
                deferred_edge_ids=set(),
                deferred_edge_prefixes=[],

                # Portal weights
                weight_mode="uniform",
                custom_weights={"origin": {}, "sink": {}},

                seed=104,
            )
        else:
            month_cfg = cmr.MonthRunnerConfig(
                run_mode="months",
                start_year=MONTHS_START_YEAR, start_month=MONTHS_START_MONTH,
                num_months=int(MONTHS_TO_RUN),

                sumo_home=SUMO_HOME,
                use_gui=False,
                step_length=1.0,
                bin_sec=300,
                net_dir=DIR_SOURCE_NET,
                run_dir=DIR_MONTH_RUN,
                sim_seconds=24 * 3600,

                # Demand
                demand_mode="daily_target",
                base_spawn_rate_total=0.8,
                target_daily_veh=86400,
                scale_safety=1.00,
                max_veh_per_sec_total=2.0,
                diurnal_peak_clamp=None,

                # --- Closures (legacy + new controls) ---
                n_closures=5,                      # legacy, no longer used directly
                close_min_dur_s=2 * 60 * 60,
                close_max_dur_s=14 * 60 * 60,
                close_start_guard_s=10 * 60,
                exclude_portal_edges_from_closure=True,
                allow_cross_day=True,
                retarget_before_reroute=True,

                # New global closure controls:
                # For long multi-month runs, maybe a slightly larger pool:
                closure_pool_size=10,              # 10 edges can ever be closed
                closure_day_period=1,              # closures considered every day
                closure_edge_prob=0.3,             # per-edge closure prob on closure days

                # Late subset: a few edges that only start appearing in later part of the run
                late_closure_pool_size=3,
                late_closure_start_frac=0.7,

                # Date gates
                closure_global_start=(2025, 2, 11),   # (Y, M, D)
                deferred_closure_start=(2024, 3, 1),  # (Y, M, D)
                deferred_edge_ids=set(),
                deferred_edge_prefixes=[],

                # Portal weights
                weight_mode="uniform",
                custom_weights={"origin": {}, "sink": {}},

                seed=104,
            )

        month_summary = cmr.run_closure_month_runner(month_cfg)

        # Use the runner’s months dir for labeling/merging
        months_base_dir_for_labels = month_summary["months_dir"]       # == os.path.join(DIR_MONTH_RUN, "months")
        dets_for_labeling = os.path.join(DIR_MONTH_CONFIG, "detectors.add.xml")
        if not os.path.exists(dets_for_labeling):
            dets_for_labeling = baseline_detectors_file
        merged_out_dir = DIR_MONTH_MERGED
    else:
        print("\n=== [2/7] MONTHLY/DAY RUNNER: skipped (set RUN_MONTHLY=True to enable) ===")
        months_base_dir_for_labels = DIR_EXISTING_MONTHS
        dets_for_labeling = os.path.join(os.path.dirname(DIR_EXISTING_MONTHS), "config", "detectors.add.xml")
        if not os.path.exists(dets_for_labeling):
            dets_for_labeling = baseline_detectors_file
        merged_out_dir = DIR_EXISTING_MERGED

    # ===========================================
    # 3) DISTANCE MATRIX (SUMO routing between sensors)
    # ===========================================
    print("\n=== [3/7] DISTANCE MATRIX ===")
    os.makedirs(os.path.dirname(PATH_DISTANCE_CSV) or ".", exist_ok=True)
    distance_matrix.run_distance_matrix(
        sumo_home=SUMO_HOME,
        net_file=net_file,
        detectors_file=baseline_detectors_file,
        out_csv=PATH_DISTANCE_CSV,
        vtype_id=VTYPE_ID,
        assume_symmetric=False,
        gui=False,
        cap_sensors=None,
        progress_every=1,
    )

    # ========================================================
    # 4) CLOSURE LABELS (per-day, per-month tree)
    # ========================================================
    print("\n=== [4/7] CLOSURE LABELS ===")
    if os.path.isdir(months_base_dir_for_labels):
        yyyy_mm_dirs = [
            d for d in os.listdir(months_base_dir_for_labels)
            if len(d) == 7 and d[4] == "-" and os.path.isdir(os.path.join(months_base_dir_for_labels, d))
        ]
    else:
        yyyy_mm_dirs = []

    if yyyy_mm_dirs:
        for yyyy_mm in sorted(yyyy_mm_dirs):
            month_out_dir = os.path.join(months_base_dir_for_labels, yyyy_mm)
            print(f"\n→ Labeling month: {yyyy_mm}")
            closure_labeling.run_closure_labeling(
                month_out_dir=month_out_dir,
                dets_file=dets_for_labeling,
                raw_closure_pattern=None,     # accepts both YYYYMMDD and day_## patterns
                out_suffix=".labels.csv",
                bin_min=5,
                day_min=24*60,
                skip_existing=True,
            )
    else:
        print(f"(skip) No YYYY-MM folders under '{months_base_dir_for_labels}'. "
              f"If you have closure CSVs to label, create this structure and re-run.")

    # ========================================================
    # 4a) SENSOR READINGS: XML → per-day CSVs (detectors_*.xml → detectors_*.csv)
    # ========================================================
    print("\n=== [4a/7] SENSOR READINGS: XML → per-day CSVs ===")
    xml_csv_converter.run_xml_to_csv(
        months_base_dir=months_base_dir_for_labels,  # the YYYY-MM/day_## tree
        root_fallback=months_base_dir_for_labels,
        skip_existing=True,
    )

    # ======================================================
    # 5A) MERGE SENSOR READINGS
    #     (filters detector CSVs only; avoids picking *.labels.csv)
    # ======================================================
    print("\n=== [5A/7] MERGE SENSOR READINGS ===")
    os.makedirs(merged_out_dir, exist_ok=True)
    overall_readings_csv = os.path.join(merged_out_dir, "readings.csv")
    csv_merger.run_csv_merger(
        months_base_dir=months_base_dir_for_labels,
        out_dir=merged_out_dir,
        file_name_must_contain="detectors_",    # detector CSVs are named detectors_YYYYMMDD.csv
        write_overall_combined=True,
        overall_out_csv=overall_readings_csv,
    )

    # ======================================================
    # 5B) MERGE CLOSURE LABELS (per-month & all-months)
    # ======================================================
    print("\n=== [5B/7] MERGE LABELS ===")
    overall_labels_csv = os.path.join(merged_out_dir, "clabels.csv")
    csv_merger.run_csv_merger(
        months_base_dir=months_base_dir_for_labels,
        out_dir=merged_out_dir,
        file_name_must_contain="labels",   # merges only *.labels.csv
        write_overall_combined=True,
        overall_out_csv=overall_labels_csv,
    )

    # ===========================================================
    # 6) DESPT PREP (HDF5 + adjacency + optional extras)
    # ===========================================================
    print("\n=== [6/7] DESPT PREP ===")
    input_wide_csv = overall_readings_csv if os.path.exists(overall_readings_csv) else overall_labels_csv
    labels_csv = overall_labels_csv if os.path.exists(overall_labels_csv) else None

    despt_cfg = create_DESPT_input.DesptConfig(
        input_csv=input_wide_csv,                # primary timeseries (sensor readings if available)
        dist_csv=PATH_DISTANCE_CSV,
        labels_csv=labels_csv,                   # merged labels get stored under a separate key
        effect_matrix_csv=PATH_EFFECT_MATRIX_CSV,
        out_dir=DIR_DESPT_OUT,
        h5_key_timeseries="sensor_readings",
        h5_key_labels="closure_labels",
        adj_basename="adj_mx",
        effect_matrix_pkl="closure_effect.pkl",
        start_date="2025-01-01 00:00:00",
        step_min=5,
        threshold=0.1,
        write_adj_csv=True,
    )
    create_DESPT_input.run_despt_prep(despt_cfg)

    # ===========================================================
    # 7) SUMMARY
    # ===========================================================
    print("\n🎉 Pipeline finished.")
    print(f"  Months tree used: {months_base_dir_for_labels}")
    print(f"  Merged readings:  {overall_readings_csv}")
    print(f"  Merged labels:    {overall_labels_csv}")
    print(f"  Distance CSV:     {PATH_DISTANCE_CSV}")
    print(f"  DESPT out dir:    {DIR_DESPT_OUT}")


if __name__ == "__main__":
    run_all()
