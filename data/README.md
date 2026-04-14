# Data

This project uses one public dataset, the lower-limb biomechanics dataset of Camargo et al. (2021), and the datasheet of one real motor. No data file is committed: everything downloaded goes into `data/raw/`, which is git-ignored.

## Camargo et al. (2021) lower-limb biomechanics dataset

**What it is.** Gait data from 22 able-bodied adults (subjects AB06 to AB30, with gaps in the numbering) walking in four modes: treadmill (28 speeds from 0.5 to 1.85 m/s), level ground (slow, normal and fast, in clockwise and counterclockwise circuits), ramps (6 inclinations from 5.2° to 18°, ascent and descent) and stairs (step heights of 4, 5, 6 and 7 inches, ascent and descent), including the transitions between modes. For each trial it has OpenSim inverse kinematics and inverse dynamics, ground reaction forces, gait-cycle segmentation, and wearable sensor signals (IMU, EMG, goniometers).

**Citation.** J. Camargo, A. Ramanathan, W. Flanagan, A. Young, "A comprehensive, open-source dataset of lower limb biomechanics in multiple conditions of stairs, ramps, and level-ground ambulation and transitions", *Journal of Biomechanics* 119 (2021) 110320. <https://doi.org/10.1016/j.jbiomech.2021.110320>

**Links.**
- EPIC Lab page (description and the link to the public Dropbox folder): <https://www.epic.gatech.edu/opensource-biomechanics-camargo-et-al/>
- Dropbox folder: <https://www.dropbox.com/sh/lhurwcy0znonh56/AAAPmVdrxh7M6FW-UYHyPHyza?dl=0>
- Mendeley Data copies, in three parts: <https://doi.org/10.17632/fcgm3chfff.2>, <https://doi.org/10.17632/k9kvm5tn3f.2>, <https://doi.org/10.17632/jj3r5f9pnf.2>
- Author's tutorial (MATLAB): <https://blog.jcamargo.co/jbiomechanics_dataset/>

**License.** CC BY 4.0. This is the license in the DataCite records of all three Mendeley Data parts (checked on 2026-10-05). The Mendeley pages themselves did not load that day (HTTP 502 from the site). The authors ask to be cited in any paper or project that uses the data (`README.txt` in the dataset). Raw files are not redistributed here; derived figures and aggregate tables (for example mean ankle torque per mode) may be committed with the citation.

**Size.** About 1 GB per subject for everything (EPIC Lab page). The folders this project needs come to about 163 MB for subject AB06 (492 files, measured with `--dry-run`), so roughly 3.5 GB for all 22 subjects. Start with one or two subjects.

### Layout and the files this project needs

The data are MATLAB `.mat` files laid out as `<subject>/<date>/<mode>/<sensor>/<trial>.mat`, for example `AB06/10_09_18/stair/id/stair_1_l_01_01.mat`. The same trial name appears in every sensor folder. Sampling rates are from the dataset's `README.txt`.

| File or folder | Used for |
|---|---|
| `README.txt` | The dataset's own description of the layout and sensors |
| `SubjectInfo.mat` | Age, gender, height (m) and body mass (kg) of each subject, to scale torques to the prosthesis user |
| `<mode>/id/` | Inverse dynamics, 200 Hz. Ankle moment: `ankle_angle_r_moment`, `ankle_angle_l_moment` |
| `<mode>/ik/` | Inverse kinematics, 200 Hz, angles in degrees. Ankle angle: `ankle_angle_r`, `ankle_angle_l` |
| `<mode>/gcRight/`, `<mode>/gcLeft/` | Gait cycle per leg, 200 Hz: columns `HeelStrike` and `ToeOff`, which from their values look like a 0 to 100 % phase that restarts at that event (confirm on a plot). Used to cut strides and as a phase reference |
| `<mode>/conditions/` | Locomotion mode labels per sample and the trial condition (treadmill speed, ramp incline or stair height), 1000 Hz |

Modes: `treadmill`, `levelground`, `ramp`, `stair`. Not needed: `emg`, `fp`, `gon`, `imu`, `jp`, `markers`, `ik_offset`, `static` and the per-subject `osimxml/` OpenSim models.

### What was checked in the files (AB06, on 2026-10-05)

- **Format.** Most files hold MATLAB `table` objects. `scipy.io.loadmat` cannot read them (it returns an opaque `MatlabOpaque` object). The `mat-io` package (version 1.0.0, BSD-3-Clause, `pip install mat-io`, imported as `matio`) reads them into pandas DataFrames with `matio.load_from_mat(path)`; this worked for `id`, `ik`, `gcRight`, `conditions` and `SubjectInfo.mat`. The `conditions` files mix plain arrays (readable by SciPy) with a `labels` table.
- **Inverse dynamics table.** 24 columns: `Header` (time in s) and the joint moments of the OpenSim model, including `ankle_angle_r_moment`. Values are in N·m and are not normalized by body mass. In the stair trial `stair_1_l_01_01`, `ankle_angle_r_moment` ranges from about -137 to 7 N·m. The first samples of the trial are NaN (no force plate under the foot), so strides must be cut with the gait events and checked for NaN.
- **NaN is any stance off a force plate (confirmed on AB06 and AB07, 2026-10-06).** In level-ground, ramp and stair trials the ankle moment is NaN for about 35 to 43 % of the samples: every stance whose foot is not on a force plate, from heel strike to toe-off, plus the idle periods. Only one or two right-leg strides per level-ground pass land on a plate, so level-ground and stair profiles rest on far fewer strides than the treadmill (where the instrumented belt gives a moment for every stride and there is no NaN). Strides on a plate often have one or two NaN samples at the edge of contact; the stride code interpolates gaps up to 50 ms and drops longer ones. See [`results/inventory.md`](../results/inventory.md).
- **Sign convention (confirmed).** The ankle angle is positive in dorsiflexion and the moment is the generalized force along the same coordinate, so plantarflexion moments are negative and joint power is moment times angular velocity. Check: at 45 to 60 % of the gait cycle (late stance) in level walking, treadmill walking and ramp ascent, the right ankle moment is negative, the angle is moving toward plantarflexion and the power is positive (push-off) in 59/60, 17/17 and 60/60 trials of AB06 and AB07. Stair trials are less clear-cut per trial (few late-stance samples land on a plate) and are checked on the stride-averaged profiles instead.
- **Coverage.** AB06 and AB07 have ramp inclines of 5.2, 7.8 and 9.2 degrees only (the `rampIncline` field reads 7.79 and 9.21) and stair heights of 4, 5 and 6 in only. Their treadmill trials cover 32 speeds from 0.5 to 2.05 m/s (8 trials of 4 speeds; AB07 has a repeated trial named `treadmill_04_2_01`), more than the 28 speeds up to 1.85 m/s listed for the dataset. Each treadmill trial holds four speed plateaus of about 32 s with ramps between them; the `conditions` file gives the belt speed at 1000 Hz.
- **Stair heights.** In the `conditions` files, `stairHeight` is in inches, and the trial name encodes it: `stair_1` is 4 in, `stair_2` 5 in, `stair_3` 6 in. AB06 has no `stair_4` (7 in) trials, so not every subject covers every height. Check the coverage per subject before pooling.
- **Mode labels.** The `labels` table of a stair trial uses `idle`, `stairascent`, `stairdescent`, `walk-stairascent`, `stairascent-walk`, `walk-stairdescent` and `stairdescent-walk`.

### How to download

No login and no browser are needed for the script. It uses only the Python standard library:

```bash
python scripts/fetch_data.py --dry-run                 # list files and total size for AB06
python scripts/fetch_data.py                           # AB06: conditions, ik, id, gcRight, gcLeft in all four modes
python scripts/fetch_data.py --subjects AB06 AB07 AB08
python scripts/fetch_data.py --subjects all            # about 3.5 GB
python scripts/fetch_data.py --manual                  # print the manual steps
```

Files land in `data/raw/camargo/` with the dataset's own layout. Files already present with the right size are skipped. Every downloaded `.mat` file must start with the MATLAB header: when many requests run in parallel, Dropbox throttles them and answers with a web page instead of the file (seen on 2026-10-06 with 16 parallel downloads), so the script then waits and retries with a growing delay. Keep parallel downloads to a few at a time. Dropbox has no documented anonymous API for shared folders, so the script uses the listing request the Dropbox web page makes. If that stops working, the script prints the manual steps: open the Dropbox link from the EPIC Lab page in a browser (no login needed) and download the same folders by hand, or use the Mendeley Data copies.

## Motor datasheet

The actuator model needs the constants of a real motor and gearbox (torque constant, winding resistance, rotor inertia, rated and peak current, mass, gearbox efficiency). Choose a motor from a manufacturer's published datasheet, save the values used in a small file in this folder (for example `data/motors/<motor>.yaml`) with the datasheet title, version or date, URL and retrieval date, and cite it in the README. Do not commit the datasheet PDF itself unless its terms allow it; link to it.

## Not used here

No patient data and no data from Parmida's thesis (IMU and Vicon recordings at Sharif) are used or stored in this repository.
