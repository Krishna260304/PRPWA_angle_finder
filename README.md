# Stepper Motor Pointer Angle Measurement Dashboard

A high-precision, production-grade Python desktop application designed to measure and calibrate the angular movement of a physical stepper-motor-driven pointer using a live camera feed and interactive protractor-style geometry.

> [!NOTE]
> **External Motor Drive**: The stepper motor is driven by your external controller (such as an external MCU, PLC, pulse generator, or driver apparatus). This software is dedicated **exclusively to visually measuring angular displacement from the live camera feed and recording measurements in degrees**.
> **Zero False Detection**: There is **no automatic computer-vision blob detection** or false noise triggers from colored motor wires. Position aiming is done manually and precisely with a full-frame crosshair.

---

## Key Features

1. **Exact 2-Click Initialization**:
   - **Click 1**: Set central shaft pivot (**Origin O**).
   - **Click 2**: Set pointer arm marker (**Point 1 / Baseline**).
   - **Crosshair Disappears Immediately**: Once Point 1 is clicked, the crosshair vanishes from the live camera feed so the view remains clean and unobstructed.
2. **Automatic Black Dot Tracking**:
   - The application automatically tracks the physical black dot on the pointer arm in real-time as it rotates along radius $R$.
   - **Zero Noise / No False Triggers**: Tracking is constrained along the circular path of the pointer arm, completely ignoring cables, shadows, and tripod legs.
   - **Auto-Settle Measurement**: When the motor finishes rotating and settles, the new angle is automatically calculated and recorded in the table without any measurement spam.
3. **Strictly TWO Lines on Screen (Sliding Window)**:
   - **Initial**: Line 1 (Baseline).
   - **Rotation 1**: Line 1 and Line 2 are shown. Angle measured between Line 1 and Line 2.
   - **Rotation 2**: **Line 1 disappears!** Line 2 and Line 3 are shown. Angle measured between Line 2 and Line 3.
   - **Rotation 3**: **Line 2 disappears!** Line 3 and Line 4 are shown. Angle measured between Line 3 and Line 4.
   - At all times, only the **previous position line** and **current position line** are displayed.
4. **Protractor-Style Visualization (Rendered on Live Video Feed)**:
   - **Origin Reticle**: Central pivot circle $\bigoplus$ at the motor shaft center.
   - **Red Radial Lines**: High-visibility red lines extending from Origin $O$ to previous and current positions.
   - **Numbered Position Dots**: Solid red circular dots labeled with position numbers `1`, `2`, `3`...
   - **Circular Angle Arcs**: Smooth arc spanning between the two active lines.
   - **Framed Degree Callout Badges**: Clean, framed badge boxes displaying the exact measured angle in degrees (e.g. `[ 29.52° ]`).
   - **No Protractor Graphics**: Only the lines, dots, arcs, and degree callouts are overlaid on the live feed.
5. **Interactive 4-Column Measurement Table**:
   - Columns: `S.No`, `Motion` (Clockwise / Anti-Clockwise), `Angle` (degrees, e.g. `29.52°`), `Steps`.
   - **Inline Cell Editing**: Double-click any cell in the **Steps** column to edit the step count directly. Measured angles remain strictly preserved.
   - **Export to CSV**: Save full experiment measurement data to `.csv` with one click.
6. **Flexible Reset Controls**:
   - **Refresh / Reset Experiment**: Completely resets the session to Step 1 (clears all points, arcs, and table).
   - **Clear Points (Keep Origin)**: Preserves the calibrated motor center $O$ and prompts to re-mark Point 1.

---

## Table of Contents
1. [Installation & Requirements](#1-installation--requirements)
2. [Workflow: How to Measure Angles](#2-workflow-how-to-measure-angles)
3. [Protractor Overlay Visual Elements](#3-protractor-overlay-visual-elements)
4. [Editing Steps in the Table](#4-editing-steps-in-the-table)
5. [Exporting Data to CSV](#5-exporting-data-to-csv)
6. [Simulation Mode (Testing Without Hardware)](#6-simulation-mode-testing-without-hardware)
7. [Troubleshooting Guide](#7-troubleshooting-guide)

---

## 1. Installation & Requirements

### Requirements
- **Python 3.11+**
- Packages: `opencv-python`, `numpy`, `Pillow`, `pyserial`

### Quick Start
```bash
# Clone or open the project folder
cd Servo

# (Optional) Activate virtual environment
python -m venv venv
.\venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Launch application
python main.py
```

---

## 2. Workflow: How to Measure Angles

### Step 1: Click the Motor Shaft Center (Origin O)
- When the app starts, the full-frame crosshair is active.
- Move your mouse to the center of the motor shaft and click once.
- A cyan origin reticle `Origin O (x,y)` will be locked at the vertex.

### Step 2: Click the Black Dot on the Pointer Arm (Point 1)
- The app automatically prompts you for Step 2.
- Move the crosshair to the black dot / screw on the pointer arm and click.
- A prominent red radial line (Line 1) and a solid red dot labeled `1` will be established.
- **The crosshair immediately disappears** from the camera view.

### Step 3: Rotate the Stepper Motor
- Drive your motor using your external controller/apparatus (e.g. 50 steps, 100 steps, or arbitrary rotation).

### Step 4: Record Position After Rotation
- Click the prominent green button: **"RECORD POSITION AFTER ROTATION (MEASURE ANGLE)"** (or click directly on the camera canvas).
- The crosshair reappears along with a live aiming preview line and dynamic angle arc.
- Move the crosshair to the black dot at its new position and click.
- **Instantly**:
  1. Radial Line 2 is drawn.
  2. Red dot labeled `2` is created.
  3. A circular arc connects Line 1 and Line 2 with a callout box showing the exact angle in degrees (e.g. `29.52°`).
  4. A row is added to the measurement table with `S.No`, `Motion`, `Angle`, and `Steps`.
  5. The crosshair **disappears immediately**.
- Repeat Step 3 and 4 for subsequent rotations (Point 3, Point 4...) to observe multi-step rotations.

---

## 3. Protractor Overlay Visual Elements

The visualization matches the protractor geometry shown in your reference diagram:

| Element | Description | Visual Appearance |
|---|---|---|
| **Origin O** | Center of rotation (motor shaft) | Cyan concentric circle reticle $\bigoplus$ with coordinate label |
| **Radial Lines** | Lines extending from Origin to each marked point | Solid bright red lines with subtle dark outline |
| **Numbered Dots** | Marked positions of the black dot | Solid red dots with white border, labeled `1`, `2`, `3`... above the dot |
| **Rotation Arcs** | Angular span between consecutive positions | Smooth circular arcs connecting previous and current line |
| **Degree Badges** | Measured angle value | Framed callout boxes displaying exact degrees (e.g. `[ 29.52° ]`) |
| **Live Preview** | Dynamic preview during mouse aiming | Yellow line from Origin to mouse + live dynamic arc showing live angle |
| **Aiming Crosshair** | Appears only when actively aiming | Full-frame crosshair with coordinate indicator (hidden once clicked) |

---

## 4. Editing Steps in the Table

The **Steps** column in the measurement table is editable:
1. Double-click directly on the **Steps** cell of any row.
2. An inline text entry field appears. Type the commanded steps (e.g. `100`, `200`) and press **Enter**.
3. The measured physical angle remains strictly unchanged.
4. You can also select a row and click **"Edit Selected Steps"**.

---

## 5. Exporting Data to CSV

1. Click the **"Export CSV"** button below the measurement table.
2. Select destination path and save.
3. The exported CSV file contains:
```csv
S.No,Motion,Angle,Steps
1,Clockwise,29.52°,100
2,Clockwise,49.40°,150
```

---

## 6. Simulation Mode (Testing Without Hardware)

If testing on a computer without an active webcam or physical stepper connected:
1. In the **Camera** dropdown, select **"Simulation Rig"** and click **"Start Camera"**.
2. A simulated laboratory camera view of a stepper motor and pointer arm will render.
3. Click the motor shaft center (Origin O).
4. Click the black dot on the pointer arm (Point 1).
5. In the **Simulation Rig** tab on the right side, select **CW** or **CCW** and click **"Test Rotate Motor"**.
6. The simulated motor rotates, and the aiming crosshair is automatically armed to let you click the new position and record the angle!

---

## 7. Troubleshooting Guide

| Issue | Cause | Solution |
|---|---|---|
| **Crosshair disappeared after clicking** | Normal operation | The crosshair intentionally disappears once a point is clicked to keep the camera view clean. Click "RECORD POSITION AFTER ROTATION" or click the video canvas to aim for the next point. |
| **Camera feed shows black or busy** | Webcam used by another application | Close other webcam programs (Zoom, Teams, etc.). Select a different camera index or use "Simulation Rig". |
| **Want to start over completely** | Need new origin and clean table | Click **"Refresh / Reset Experiment"** to reset all lines, dots, table rows, and start fresh from Step 1. |
| **Want to keep the motor origin but clear measurements** | Motor didn't move center | Click **"Clear Points (Keep Origin)"** to preserve Origin O and restart from Point 1. |
