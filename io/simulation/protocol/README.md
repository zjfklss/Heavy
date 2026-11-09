# RM simulation protocol SDK

This directory contains the algorithm-side protocol SDK for `rm-sim/1`, copied from
the simulation project's `modules/adapters/` on 2026-11-07. It contains only Linux TCP
framing, bounded JSON/BGR messages, session validation, telemetry and angle conventions.
It does not contain a detector, tracker, predictor, solver, aiming algorithm or simulator.

The bundled copy lets this repository build its optional simulation entry without a
sibling simulation checkout. It requires C++17 and nlohmann-json. CMake defaults to
`io/simulation/protocol/sdk/cpp`; `JLCV_SIM_SDK_DIR` can explicitly select another SDK.

To synchronize with the simulation project, copy these paths from `modules/adapters/`
while preserving their relative layout:

- `include/rm_sim/adapters/{profile,wire,server}.hpp`
- `src/{profile,wire}.cpp`
- `algorithms/{generic,sp_vision}/profile.cpp`
- `sdk/cpp/{CMakeLists.txt,server.cpp}`

Protocol changes must stay compatible on both sides and be checked with the actual
independent vision process. The source entry uses the `sp_vision` profile: radians,
yaw left positive, pitch up negative. See `docs/SIMULATION.md` for launch instructions.
