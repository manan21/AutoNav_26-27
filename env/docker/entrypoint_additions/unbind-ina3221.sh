#!/bin/bash

# Unbind the onboard INA3221 driver from I2C bus 1 address 0x40
# so the external INA226 (also at 0x40) can be accessed by userspace.
UNBIND_PATH="/sys/bus/i2c/drivers/ina3221/unbind"
DEVICE="1-0040"

if [ -e "$UNBIND_PATH" ]; then
    echo "$DEVICE" > "$UNBIND_PATH" 2>/dev/null
    tput setaf 5
    echo "INA3221 unbound from $DEVICE — INA226 ready on /dev/i2c-1"
    tput sgr0
else
    echo "INA3221 unbind path not found — skipping"
fi
