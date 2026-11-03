# M3d — ASR observation-gap continuity fencing

M3c prevented stale **last** samples from generating a pressure alarm, but
did not prevent a timer that had stopped for an extended period from resuming
and interpreting the unobserved time as uninterrupted queue occupancy.

The speech workload observer now treats an interval strictly greater than
3000 ms between successful observations as a continuity break. A backward
observation clock also breaks continuity. Both queue and result-mailbox
residency clocks and contiguous sample counts restart at the next sample.
A gap of exactly 3000 ms remains continuous.

The existing pressure condition still requires >=4 pending segments,
>=10 seconds of observed nonempty queue, >=2 contiguous samples and a fresh
last sample. This is a diagnostic indication, not a native running-job
deadline, ASR quality metric or automatic restart mechanism. A brief queue
drain between samples may still be invisible to this polling-based observer.

Four unit tests cover long gaps, backwards time, boundary behavior and
repeated gaps. The threshold matches the M3c 3000-ms freshness bound.
