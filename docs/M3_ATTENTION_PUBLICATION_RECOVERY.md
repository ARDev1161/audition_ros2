# M3am — advisory publication recovery

The attention manager always publishes the result of an incoming scene.
Rejected replay/out-of-order scenes may produce an invalid advisory while the
policy intentionally retains the previously accepted valid target and stream
watermark. Previously, the 100 ms timer noticed that the retained target was
valid but only changed an internal boolean. Subscribers could therefore remain
on an invalid advisory indefinitely until another scene arrived.

The timer now publishes **both validity transitions**: a single invalid
advisory on timeout, and a single valid advisory when a retained target becomes
visible again after an invalid scene result. It does not repeatedly publish
unchanged states. Scene callbacks still publish every result. The transition
logic lives in a small deterministic `AttentionPublicationGate` tested with
four GTests, including startup, timeout, rejected replay recovery and new
scene supersession.

This is advisory-only recovery. A timer does not create new acoustic evidence
or extend policy freshness. Downstream consumers must still enforce their own
freshness and safety requirements; no robot motion is issued.
