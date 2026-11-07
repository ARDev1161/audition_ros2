# M3j — bounded challenger confirmation for attention

The opt-in `attention_manager` can now require repeated accepted scene snapshots
before switching away from an eligible incumbent. The default
`switch_confirmations=1` preserves the previous immediate-switch behavior.
Set `switch_confirmations=3` to require three consecutive observations where
the same challenger exceeds the incumbent by `switch_margin`.
`switch_confirmation_window_ms=1000` limits the time between the first and
final confirming observations. An interrupted, expired, regressed or
generation/frame-changed candidate cannot carry confirmation forward.

These are **startup-only** ROS parameters. Valid ranges: confirmations 1–100,
confirmation window strictly positive milliseconds. Initial acquisition when
there is no eligible incumbent is immediate; a lost source is never kept alive
solely to complete confirmation. This mechanism reduces rapid switching, but
does not establish human identity, source reliability or a motion authorization.

Example:

```bash
ros2 run audition_behaviors attention_manager --ros-args \
  -p switch_confirmations:=3 \
  -p switch_confirmation_window_ms:=750 \
  -p switch_margin:=0.15
```

Four new pure-policy regression tests exercise confirmation, interruption,
time-window expiry, rejection of regressed snapshots, and configuration.
No Nav2 command, robot motion, or new default launch activation is introduced.
