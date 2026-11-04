# M3ac: optional initial attention acquisition confirmation

The attention manager now accepts `acquisition_confirmations` (integer
1–100, default **1**) and `acquisition_confirmation_window_ms` (positive
integer, default **1000**). When confirmations exceed one, the policy will
not publish a valid new attention target until the same eligible source ID
and generation has been the highest-priority candidate in that many
consecutive accepted scenes within the time window.

During acquisition the advisory is invalid with reason
`acquisition_pending`. A different candidate, no eligible source, timeout,
generation/frame transition, or explicit reset restarts the evidence. A
regressed scene does not count. Once acquired, normal incumbent retention,
challenger hysteresis, switch confirmations, and direction quarantine
continue unchanged. No motion commands are emitted.

These parameters are startup-only and optional. They mitigate transient
single-snapshot false acquisition, but do not establish localization accuracy,
source identity certainty, or motion safety.
