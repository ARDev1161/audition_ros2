# M3x: multi-source attention direction quarantine

With `maximum_direction_jump_rad > 0`, an accepted scene may reveal several
successive track identity discontinuities. The previous single-entry quarantine
could forget the first jumped identity when a second incumbent jumped, allowing
the first to become a valid attention target while it remained present.

The policy now maintains a set of quarantined source IDs for the current
generation and frame. Each jumped incumbent is added without evicting older
entries. On **accepted** snapshots, entries are removed only when the matching
identity is absent; regressed generations, replayed timestamps and malformed
snapshots do not release them. An accepted generation/frame transition or
explicit reset clears the set. Timer expiry does not clear it.

The set never retains IDs that are absent from the latest accepted scene, so
its size is bounded by that scene's source count. No unbounded historical
identity registry is introduced. The optional continuity gate is disabled by
default; this change does not enable it automatically.

Three C++ GTests cover two sequential discontinuities, independent release
with replay rejection, and generation reset. ROS 2 Humble CI compiles and
runs these tests. Native ODAS CI checks the native integration independently.
This is a bearing-only advisory and does not imply metric localization,
robot motion authority, or physical ReSpeaker calibration.
