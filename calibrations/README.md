# Shared calibration library

Owner-curated calibration profiles served from GitHub Pages and offered by the
dashboard's **CALIBRATOR → CAL I/O → LOAD FROM GITHUB** button.

This folder is copied verbatim into the `gh-pages` branch by `tools/release.sh`,
so it is published at `https://toldxls.github.io/AutoFOV-V12/calibrations/`.

## Adding a calibration

1. On the dashboard, open **CALIBRATOR → CAL I/O → SAVE TO LIBRARY** and type a
   name (e.g. `objective-20x`). A `<name>.json` file downloads.
2. Drop that file into this folder.
3. Add an entry to `index.json`:
   ```json
   { "calibrations": [
     { "name": "Objective 20x", "file": "objective-20x.json", "by": "travis" }
   ] }
   ```
   - `name` — label shown in the dashboard list.
   - `file` — the JSON filename in this folder.
   - `by`  — optional attribution shown after the name.
4. Commit, then `bash tools/release.sh` to publish.

## File format (`autofov-calib`, ver 1)

```json
{ "fmt": "autofov-calib", "ver": 1, "name": "objective-20x",
  "calWidth": 6960, "demarcDist": 0.40,
  "points": [ { "dist": 12.3, "fov": 4.56 } ],
  "fit": { "slope": 0.000123, "intercept": 1.23, "r2": 0.999 } }
```

Only `calWidth`, `demarcDist`, and `points[]` are used on import — the device
re-runs its own least-squares fit, so the stored `fit` is informational.
