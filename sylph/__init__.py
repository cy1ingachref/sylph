"""Sylph: autonomous AI scout drone companion-computer software.

Layered architecture:

* :mod:`sylph.safety`     -- preflight / ground-safety gates (Phase 0)
* :mod:`sylph.link`       -- MAVLink-style telemetry link abstraction
* :mod:`sylph.navigation` -- waypoint runner, smart RTL energy logic
* :mod:`sylph.vision`     -- disease detection, geotagging, diagnosis
* :mod:`sylph.mission`    -- voice commands, grid search, live map
* :mod:`sylph.reporting`  -- mission reports, sensor fusion, profiles
"""

__version__ = "0.1.0"
