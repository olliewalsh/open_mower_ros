# Set default GPS antenna offset
export OM_ANTENNA_OFFSET_X=${OM_ANTENNA_OFFSET_X:--0.04}
export OM_ANTENNA_OFFSET_Y=${OM_ANTENNA_OFFSET_Y:-0.0}

# Set distance between wheels in m
export OM_WHEEL_DISTANCE_M=${WHEEL_DISTANCE_M:-0.368}

# Set default ticks/m
# 5m returned 4.7m
#export OM_WHEEL_TICKS_PER_M=${OM_WHEEL_TICKS_PER_M:-984}
# 612 ticks per rev / (0.190 wheel diameter * PI)
export OM_WHEEL_TICKS_PER_M=${OM_WHEEL_TICKS_PER_M:-1025}
