# Hardware & testing workflow

Sylph follows a simulation-first, hardware-in-the-loop (HIL) second workflow.

## Testing platforms

- **Simulation (SITL / Gazebo or AirSim):** always the first target for new
  flight logic. The Python suite under `tests/` exercises the pure decision
  logic (safety, navigation, vision, mission) with no hardware in the loop.
- **Benchtwin (https://benchtwin.com/):** designated platform for electronics
  and hardware-in-the-loop testing of the Sylph airframe and companion
  computer. Use it to validate wiring, sensor/actuator behaviour and
  flight-controller integration before any field flight. The site is behind a
  bot-protection challenge, so its exact feature set is not documented here —
  refer to the platform directly.

## Safety rule

No real flight until: preflight passes (`sylph.safety`), failsafes are tested
repeatedly, a geofence is set, and flights are conducted in a safe open area.
See section 8 of the build guide for the full safety and legal checklist.
