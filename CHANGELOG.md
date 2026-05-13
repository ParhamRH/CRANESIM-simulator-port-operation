# Changelog

All notable changes to CraneSim are documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).  
Versioning follows [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- Nothing yet — be the first to contribute!

---

## [1.0.0] — Initial Release

### Added
- Full 8-phase automatic crane loading cycle
- `IMUSensor` class — gyroscope, accelerometer, magnetometer, barometer with Gaussian noise
- `PhysicsController` state machine with force-based spreader height control and kinematic trolley
- `CameraManager` — five debug-viewer presets plus synthetic RGB/Depth/Segmentation camera
- `UIManager` — parameter sliders, live on-screen telemetry, coloured waypoint markers
- `DataManager` — per-step data collection, Excel (.xlsx) export with CSV fallback
- `config.py` — single source of truth for all simulation constants
- `crane_objects.py` — stateless PyBullet body factories
- Interactive keyboard controls (camera rotation, pan, save, quit)
- `.gitignore`, `requirements.txt`, `CONTRIBUTING.md`, `CHANGELOG.md`, `LICENSE`
- `docs/ARCHITECTURE.md` — module design rationale
- GitHub Actions CI workflow (lint + tests)
- Issue templates for bug reports and feature requests
