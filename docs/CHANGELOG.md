# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [0.0.2] - 2026-02-05

### Fixed
- **Wrist Camera Grasping Demo Texture Bug**: Fixed texture missing issue in wrist camera pick and place example by adding missing texture files (Image_Joint.png, Image_Link.png, Image_Logo.png) and updating USD asset path structure
- Improved robot asset loading by updating USD path from `gbt_c5a_camera_gripper` to `gbt-c5a_camera_gripper` with proper texture binding

### Changed
- **Documentation Links**: Fixed navigation links in wrist camera grasping example documentation to point to correct language versions (README_CN.md now correctly links to main Chinese documentation)
- **Code Organization**: Renamed main entry script from `pick_place_example.py` to `pick_place.py` for consistency
- **URDF Configuration**: Renamed URDF file from `gbt-c5a.urdf` to `gbt-c5a_camera_gripper.urdf` to better reflect robot configuration
- **Documentation Structure**: Improved formatting and organization in README files (added proper table structures, code formatting for dependencies, section hierarchy)
- **Code Comments**: Enhanced code comments in `tasks/pick_place.py` for better maintainability and clarity


### Added
- **VLA Model Notes**: Added camera field of view limitation notes for VLA model training in both English and Chinese documentation

---

## [Unreleased]

### Planned
- 2D grasping demo: Simulation grasping based on 2D vision
- 3D grasping demo: Simulation grasping based on 3D vision

---

## [0.0.1] - 2025-01-09

### Added
- Agilebot IsaacSim Integration Framework Initial Release
- Agilebot Robot Core Control Code
- RMPflow Motion Policy Configuration for gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a
- Example Demos:
  - Stacking Task
  - Pick and Place Task
  - Target Following Task
  - Target Following with Forward Kinematics Verification
- MoveIt 2 Integration with ROS 2 Humble
- Pick and Place with Wrist Camera Demo (complete workflow and video recording)
- Bilingual Documentation (English/Chinese)

### Changed
- Updated Isaac Sim environment setup guide for version 5.1:
  - Python version requirement changed to 3.11 (was 3.10)
  - System requirements updated (Ubuntu 22.04/Windows 11 recommended)
  - NVIDIA driver versions updated to 580+ series (Linux: 580.65.06+, Windows: 580.88+)
  - Added Isaac Lab asset caching configuration
- Updated demo scripts to remove `--help` commands (avoids Isaac interception)
- Synchronized Chinese and English documentation
- Updated README and README_CN.md to link to environment configuration guides
- Enhanced README with environment setup prerequisites

### Deprecated
- N/A

### Removed
- N/A

### Fixed
- N/A

### Security
- N/A

---

## Change Types

- `Added` for new features
- `Changed` for changes in existing functionality
- `Deprecated` for soon-to-be removed features
- `Removed` for now removed features
- `Fixed` for any bug fixes
- `Security` in case of vulnerabilities
- `Planned` for future roadmap items

## Configuration

To update this changelog, follow these steps:

1. Add your changes under the appropriate section (Added/Changed/Fixed/etc.)
2. Add planned features under the `### Planned` section in `[Unreleased]`
3. Update the version number and date when releasing
4. Push changes - GitHub Actions will automatically sync to both README files
5. For local testing, run:
   ```bash
   python .github/scripts/sync_changelog.py
   ```
6. Commit both CHANGELOG files together (if GitHub Actions fails or for manual sync)

## Automated Synchronization

The repository includes GitHub Actions workflows (`.github/workflows/sync-changelog.yml`) that:

- Automatically trigger when CHANGELOG.md or CHANGELOG_CN.md is pushed
- Reads the `### Planned` section from CHANGELOG files
- Updates "Upcoming Features" section in README.md
- Updates "即将推出的功能" section in README_CN.md
- Supports both English and Chinese versions
- Preserves existing formatting
- Auto-commits changes (skips CI to avoid recursive triggers)

For detailed information, see:
- [English Guide](./.github/workflows/README.md)
- [中文指南](./.github/workflows/README_CN.md)

### Manual Sync

If you need to manually sync (e.g., local testing):

```bash
python .github/scripts/sync_changelog.py
```
