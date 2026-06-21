# prototype_v11_1_1_clean_real_dem

Version: v11.1.1

This package is a cleaned v11.1.1 revision for the direction-dependent off-road trafficability model.

## What changed from v11.0

1. Real DEM outputs now use `outputs_v11_1_1_real_full` or user-provided `--out`; they are no longer written into `outputs_v10_real_full` by default.
2. The artificial built-up square patch is disabled by default. It appears only if `--add-demo-barrier` is explicitly used.
3. Main v11.1.1 output file and folder names are ASCII-safe to avoid Windows zip filename mojibake.
4. `J_plan` is consistently described as relative accumulated path cost, not Joule energy.
5. A clean formula note is provided in `docs/01_model_formulas_and_experiment_logic.md`.

## How to run

Install dependencies:

```bat
python -m pip install -r requirements.txt
```

Synthetic smoke test:

```bat
run_v11_1_1_synthetic_test.bat
```

Quick real DEM test:

```bat
run_v11_1_1_my_dem_quick.bat
```

Full real DEM run:

```bat
run_v11_1_1_my_dem_full.bat
```

The real DEM path is read from:

```text
my_dem_path.txt
```

Optional rasters can still be provided through command line arguments:

```bat
python run_v11_experiments.py --dem your_dem.tif --landcover your_landcover.tif --soil your_soil.tif --water your_water_mask.tif --out outputs_v11_1_1_real_full
```
