# Power Monitoring PCB

A custom Texas-Instruments-based PCB that sits **in series between the battery and the rest of the robot**, measures voltage / current / power across a shunt, and reports State of Charge to the Jetson over I²C. Designed and maintained as a separate project (link at the bottom); this doc only covers how it hooks into AutoNav.

## How it physically connects

The board has four screw terminals split into two pairs:

| Terminal pair | What it connects to |
|---|---|
| **POWER + / −** | The **battery** pack (currently a Renogy RBT2425LFP LiFePO₄, ~25.6 V nominal) |
| **LOAD + / −** | The **rest of the robot** — motors, Jetson power input, sensors, everything downstream |

So the topology is:

```
[ Battery ]
     │
     ▼
POWER+   POWER−
   ╔════════════╗
   ║  PCB       ║   ← shunt + INA226 measure here
   ║  (INA226 + ║      between POWER side and LOAD side
   ║   BQ34Z100)║
   ╚════════════╝
LOAD+    LOAD−
     │
     ▼
[ Rest of robot ]
```

Every amp the robot draws flows through the PCB's shunt resistor, so current can be inferred from the tiny voltage drop across it. (See the *Power Monitoring PCB (brief synopsis)* section in the [main README](./HUMAN-WRITTEN-README.md#power-monitoring-pcb-brief-synopsis) for the math.)

## How it talks to the Jetson

Three wires, plus you can ignore power for the comm bus because the PCB powers itself off the battery side:

| Wire | Purpose |
|---|---|
| **SCL** | I²C clock |
| **SDA** | I²C data |
| **GND** | Common ground (shared with the Jetson) |

That's it — it's I²C. No USB, no UART, no Ethernet on the comm side.

| | |
|---|---|
| **Jetson I²C bus** | `/dev/i2c-1` (Jetson 40-pin header pins 27 = SDA, 28 = SCL) |
| **INA226 address** | `0x40` |
| **BQ34Z100-R2 address** | (separate; SOC gauge — see PCB repo for details) |

## How it shows up in ROS2

The Jetson-side software lives in the **`autonav_electrical_publisher`** package.

| | |
|---|---|
| **Bringup (GUI)** | Click the **Power PCB** button in the launch panel |
| **Bringup (manual)** | `./config/run-electrical.sh` |
| **Launch file** | `ros2 launch autonav_electrical_publisher electrical_publisher.launch.py` |
| **Node** | `electrical_publisher_node` |
| **Publishes** | `/electrical/voltage` (`std_msgs/msg/Float32`), `/electrical/current`, `/electrical/power` |

Per the package's own gotcha: the INA226 needs a **calibration register write at startup** (`0x05 = 0x0800` for the 10 mΩ shunt). The node retries in a timer loop on I²C init failure — don't trust readings until the log says "calibrated and ready."

## What the PCB does on its own (no Jetson required)

Even with the Jetson off, the PCB drives an LED bank showing approximate SOC (100% / 80% / 60% / 40% / 20% / critical). Useful for a quick "should I plug it in?" check while the robot is parked.

The on-board BQ34Z100-R2 fuel gauge integrates current vs. time and re-anchors against an empirical discharge curve when the battery is at rest — that's the same drift-correction logic described in the [Drift section](./HUMAN-WRITTEN-README.md#drift) of the main README.

## House rule — protective enclosure

After the **R2 board failure on the Bowser → Shogi transfer** (metal shavings from chassis work landed on the bare board, induced CMOS latch-up on the INA226's SCL pin, and cascaded into a dead DC-DC buck), the project mandates that **all future PCBs ship inside a protective enclosure before being mounted on a robot**. Bare boards near a metal chassis is a "when, not if" failure mode.

Concretely: if you see the PCB exposed on a robot, escalate before the next test session.

## Relevant ROS2 package

[`autonav_electrical_publisher`](../isaac_ros-dev/src/autonav_electrical_publisher/) — see the [PACKAGES.md entry](./PACKAGES.md#autonav_electrical_publisher) for build/topic specifics.

## Reference repo

The full PCB design (KiCad project, schematic, BOM/fab outputs, programming scripts, board bring-up procedure, datasheets) lives in a separate repository:

**<https://github.com/nfikes/AutoNav-Charge_Indicator-KiCad_Pcb>**

Read that repo's README for the schematic, the board bring-up procedure (10 V power-on test, voltage-rail verification, I²C comm tests, BQ34Z100 chemistry programming), and the full failure-mode write-ups.
