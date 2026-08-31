# RoboteQ USB Issue — Clearing a Latched MOSFET Fault

When the **USB/status LED on the RoboteQ is red** and the robot will not move, the
controller likely has a latched fault flag. Its output stage is disabled until the fault is
cleared. This is how to confirm it and reset it.

## STEPS

Ensure you are inside the JETSON when you are invoking these commands.

- The command below defines a variable to hold the RoboteQ serial ID, and then shows any process on that port.
- KEEP IN MIND: The serial ID will change between controllers.

```bash
DEV="/dev/serial/by-id/usb-RoboteQ_RoboteQ_FBLG2360T_IwBCAAlQMTlZMDIg_2074307C3931-if00"
fuser -v "$DEV"
```

BTW if fuser returns a PID owned by cat, kill it. There could be multiple.
What it looks like:

```text
                       USER        PID ACCESS COMMAND
  /dev/ttyACM1:        vtcro      2507 F.... bash
                       vtcro      2646 F.... cat             
```

```bash
kill {PID1} {PID2}
```

Set 115200 baud, 8 data bits, no parity, no stop bit, flow control off, raw, no echo:

- The command below sets up a serial connection to that serial device which is the DEV variable you defined inline.

```bash
stty -F "$DEV" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
```

Next we want to just ensure that the MOSFET fault is correct.

- The commands below send a little message to the DEV serial variable and what prints out is what the controller responds with.
- NOTE: You don't need to copy the little comments.

```bash
printf '?FF\r' > "$DEV"   # Fault flags   -> FF=64  means MOSFET failure (latched)
printf '?FS\r' > "$DEV"   # Status flags
printf '?FM 1\r' > "$DEV" # Motor 1 runtime flags
printf '?FM 2\r' > "$DEV" # Motor 2 runtime flags
printf '?V 2\r' > "$DEV"  # Main battery voltage in 0.1 V (e.g. 259 = 25.9 V)
```

If you see `FF=64` then you need to reset the MOSFETS. Sending this RESET along with the controller's PASSWORD will reset it.

```bash
printf '%RESET 321654987\r' > "$DEV"
sleep 3
```

## VERIFICATION

Re-read the fault flags — a successful reset returns everything to zero:

```bash
stty -F "$DEV" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
printf '?FF\r' > "$DEV"   # expect FF=0
printf '?FM 1\r' > "$DEV" # expect FM=0
printf '?FM 2\r' > "$DEV" # expect FM=0
```
