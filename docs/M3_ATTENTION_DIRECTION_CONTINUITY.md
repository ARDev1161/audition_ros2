# M3m — optional attention direction continuity gate

The opt-in attention manager supports startup-only
`maximum_direction_jump_rad` (default 0, disabled; valid range [0, pi]).
When enabled, the currently selected source is ineligible if its new unit
bearing differs from the last published valid bearing by more than the
configured angular threshold. This avoids silently tracking a reused source
identity that abruptly jumps across the microphone frame.

The gate applies only to the same source ID and generation. A new source or
new scene generation is evaluated independently, so it cannot inherit the
old source's angular constraint. An invalidated candidate cannot remain
actionable through the timer: no eligible bearing produces an invalid target
and clears the previous target. This is deliberately a **fail-closed**
continuity policy; it does not estimate motion or perform source association.

Example:

```bash
ros2 run audition_behaviors attention_manager --ros-args \
  -p maximum_direction_jump_rad:=0.5 \
  -p maximum_source_age_ms:=250
```

Choose a threshold compatible with the spatial backend's update rate and
angular error. Fast-moving sources may legitimately exceed the threshold;
there is no velocity model. This remains a bearing-only advisory, never
a Nav2 goal or actuation command. Four regression tests cover jump rejection,
small changes, new identities/generations, and default/config validation.
