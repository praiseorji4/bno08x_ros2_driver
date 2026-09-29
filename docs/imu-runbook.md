# BNO085 runbook: long still recording (E2) and analysis

Step-by-step commands, in the order they are run, with what each one does and the mistakes we hit along the way. The Pi runs ROS 2 **Jazzy**. Commands marked **[Pi]** run on the Pi (over `ssh tony@tony.local`); **[WSL]** run in the Ubuntu/WSL terminal on the laptop (prompt `praise@Ozioma:~$`, not the Windows `C:\Users\pator>` prompt).

---

## Part A: one-time setup

### A1. Laptop Python environment [WSL]
```
sudo apt update && sudo apt install -y python3-pip python3-venv
python3 -m venv ~/imuenv
source ~/imuenv/bin/activate
pip install mcap numpy matplotlib zstandard lz4
```
- WSL's Python ships without pip, and new Ubuntu versions refuse system-wide pip installs, so a virtual environment (`~/imuenv`) holds the packages.
- In every new terminal, run `source ~/imuenv/bin/activate` before using the tool.

### A2. Get or update the analysis tool [WSL]
```
source ~/imuenv/bin/activate && cd ~
curl -LO https://raw.githubusercontent.com/praiseorji4/bno08x_ros2_driver/characterization-mode/tools/e2_allan_analysis.py
```
Re-run the `curl` line whenever the tool is updated. It overwrites `~/e2_allan_analysis.py`.

### A3. Update the driver on the Pi [Pi]
```
cd ~/ros2_ws/src/bno08x_ros2_driver && git pull && git log -1 --oneline
cd ~/ros2_ws && colcon build --packages-select bno08x_driver && source install/setup.bash
```
After this, any running launch must be restarted to use the new build. As of 2026-09-28, the lean Allan profile still isn't taking effect on the Pi (see Part E), so check with the 60 s test in B3.

---

## Part B: start a long still recording [Pi]

### B1. Physical setup
- The IMU sits on its own heavy, still surface. It must not sit on the Pi or its case (fan vibration), and the wires need slack.
- Keep it away from motors, metal and appliances that cycle on and off (fridge, AC, chargers).
- Don't touch the table during the run. Write down the start time and room temperature.

### B2. Start the driver inside tmux
tmux keeps programs running after SSH closes or the laptop sleeps. **Both the driver and the recorder must run inside tmux.** A launch started in a plain SSH terminal dies when that terminal closes, and the recording goes silent.
```
tmux new -s imu
source ~/ros2_ws/install/setup.bash
ros2 launch bno08x_driver bno085_i2c_characterization.launch.py config:=$(ros2 pkg prefix --share bno08x_driver)/config/bno085_i2c_allan.yaml
```
This is window 0. The driver must keep running for the whole recording, because the recorder only saves what the driver publishes.

### B3. 60-second check (second tmux window)
Press **Ctrl-b then c** to open window 1, then:
```
source ~/ros2_ws/install/setup.bash && df -h ~
T="/bno08x/raw/accelerometer /bno08x/raw/gyroscope /bno08x/raw/magnetometer /bno08x/gyroscope_uncalibrated /bno08x/report_info"
rm -rf ~/imu_data/check_lean
timeout -s INT 60 ros2 bag record -s mcap -o ~/imu_data/check_lean $T && ros2 bag info ~/imu_data/check_lean
```
- `T` holds the list of topics to record. Set it again in any new window.
- `rm -rf` is needed because the recorder won't write into a folder that already exists.
- `timeout -s INT 60` stops the recorder after 60 s with the same signal as Ctrl-c, so the bag closes properly.

**Read the `ros2 bag info` output under "Topic information":**

| Line | Expected in 60 s | Meaning if not |
|---|---|---|
| `/bno08x/raw/gyroscope` Count | about 6000 (100 Hz) | Far less: bus too slow, or the driver isn't running |
| `/bno08x/raw/accelerometer` Count | about 7500 (126 Hz, the sensor rounds its rate up) | |
| `/bno08x/raw/magnetometer` Count | about 3000 (50 Hz in the lean profile) or 6000 (full profile) | |
| `/bno08x/report_info` Count | about 22,500 (lean profile: the four topics above added together) | About 62,000: the full profile is running (fine, just a bigger bag) |
| Message count 0, no topics | | The driver isn't publishing: check window 0 for errors, `ros2 topic list \| grep bno08x`, `ros2 topic hz /bno08x/raw/gyroscope` |

### B4. Start the long recording (window 1)
```
df -h ~        # need at least 3 GB free for 9 h
timeout -s INT 9h ros2 bag record -s mcap --storage-preset-profile zstd_fast -o ~/imu_data/$(date +%Y%m%d_%H%M)_e2_allan_9h $T
```
- `timeout -s INT 9h` is what sets the duration. The `_9h` in the folder name is only a label.
- `--storage-preset-profile zstd_fast` compresses the bag: 3.5 h came to 782 MB.
- `$(date +%Y%m%d_%H%M)` puts the start time in the folder name.

### B5. Detach and check later
- Detach: **Ctrl-b then d**. Both windows keep running on the Pi. You can close SSH, and the laptop can sleep.
- Check on it: `ssh tony@tony.local` then `tmux attach -t imu`. Switch windows with Ctrl-b then 0 or 1.
- When the recorder stops itself, stop the driver with **Ctrl-c** in window 0.
- If `tmux attach` says no session exists, create one with `tmux new -s imu`.

**If the launch was started outside tmux by mistake:** stop the recorder (Ctrl-c), stop the plain-terminal launch (Ctrl-c; only one driver can use the sensor), delete the partial bag (`rm -rf ~/imu_data/*_e2_allan_9h`), then redo B2 to B5.

**Losing internet during the run** doesn't stop the recording, because the driver and recorder talk locally. The Pi clock can't sync without internet, though, and may jump when it reconnects. The analysis tool's health table shows any jump as a large gap or backwards steps.

---

## Part C: analyse on the laptop [WSL]

### C1. Look at the bag and copy it
```
ssh tony@tony.local 'source /opt/ros/jazzy/setup.bash && ls ~/imu_data && ros2 bag info ~/imu_data/<folder> | head -20'
mkdir -p ~/imu_data && scp -r tony@tony.local:~/imu_data/<folder> ~/imu_data/
```
A one-off `ssh ... 'command'` doesn't load ROS (`ros2: command not found`), hence the `source /opt/ros/jazzy/setup.bash` first.

### C2. Run the tool
```
source ~/imuenv/bin/activate && cd ~
python3 e2_allan_analysis.py ~/imu_data/<folder> --skip-start 1800
```
- `--skip-start 1800` drops the first 30 min, while the sensor warms up.
- The first run decodes the bag in a few minutes and caches it in `<folder>_report/extracted.npz`. Re-runs are near instant. Add `--reread` to force decoding again.
- `--max-hours 1` does a quick look at just the first hour.

### C3. Get the report into Windows and upload it
```
cp -r ~/imu_data/<folder>_report /mnt/c/Users/pator/Downloads/
```
`explorer.exe ~/...` doesn't work, because Windows can't open a path written with `~`. Upload `summary.md`, `summary.json` and all the `.png` files from `Downloads\<folder>_report`.

---

## Part D: quick checks on `/imu` [Pi, driver running with the default characterization profile]

The lean Allan profile turns `/imu` off, so use `ros2 launch bno08x_driver bno085_i2c_characterization.launch.py` with no `config:=`.

### D1. Gyro bias at rest (IMU still, 60 s)
```
PYTHONUNBUFFERED=1 timeout -s INT 60 ros2 topic echo /imu --field angular_velocity.z --csv | awk '{s+=$1;n++} END{if(n) print s/n*57.2958, "deg/s from", n; else print "no messages received"}'
```
It averages about 6000 yaw-rate readings. With the IMU still, the true rate is zero, so the average is the bias. A plain `timeout` (without `-s INT` and `PYTHONUNBUFFERED=1`) printed only "Terminated", because ros2 was killed before it flushed its output. **Result 2026-09-28: exactly 0.0.**

### D2. Value histogram (still, then rotate by hand)
```
timeout -s INT 20 ros2 topic echo /imu --field angular_velocity.z --csv > /tmp/gyro_z.csv
grep -vc '^---$' /tmp/gyro_z.csv                                  # number of samples (about 100 per second)
grep -v '^---$' /tmp/gyro_z.csv | sort | uniq -c | sort -rn | head  # how often each value occurred
```
**Result 2026-09-28:** exact zeros while still, real values when moving, down to one step of 1/512 rad/s. The firmware zeroes the gyro when it is still, so the raw bias (−0.081 °/s) doesn't reach the EKF at rest.

---

## Part E: open items in this procedure
- Lean profile: the 2026-09-28 9-hour run used it (report_info held only the 4 lean reports). The 60 s check before it still showed the full profile, most likely because that check ran against the earlier launch (outside tmux), which was then restarted. **Always restart the launch after rebuilding, and in a terminal that has sourced `install/setup.bash`.**
- Next experiments: E3 rotation test and a 6-position accelerometer calibration (with the robot), PR #4 robot checks, E1 `/imu` timing, E7 magnetometer near the motors.
