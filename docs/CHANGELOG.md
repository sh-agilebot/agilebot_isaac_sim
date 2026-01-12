# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Planned
- 2D grasping demo
- 3D grasping demo

---

## [0.0.1] - 2025-01-09

### Added
- Initial release of Agilebot IsaacSim Integration
- Core integration code for Agilebot robots
- RMPflow motion control configurations for gbt-c5a, gbt-c7a, gbt-c12a, gbt-c16a
- Demo examples:
  - Stacking task
  - Pick and Place task
  - Follow Target task
  - Follow Target with FK Verification
- MoveIt 2 integration example with ROS 2 Humble
- Camera-based grasping task with video recording support
- Bilingual documentation (English/Chinese)
- Apache License 2.0
- English version of Isaac Sim environment configuration guide (`isaacsim_environment_setup.md`)
- Helpful tips section in README:
  - URDF import collision mesh loss issue (Isaac Sim 5.1)
  - Independent project setup guide with example reference
  - GUI unresponsive troubleshooting (network resource loading)
- Runtime requirements section in demos README
- Pre-run troubleshooting guide for demo scripts
- Project configuration files (`.gitignore`, `CHANGELOG.md`, `CHANGELOG_CN.md`)
- GitHub Actions workflow for CHANGELOG synchronization
- `.github` directory structure with documentation and automation scripts

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
