// flight.cpp
// The flight state machine (Pad, Armed, Boost, Coast, Apogee, Descent, Main deploy, Landed) and the event detection for launch, burnout, apogee, and landing.
// It also controls the pyro channels and enforces the safety rules: arming required, never fire on the pad, and backup timer and altitude failsafes.
// It takes filtered numbers in and decides what state we are in and whether to fire.
