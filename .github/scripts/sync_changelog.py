#!/usr/bin/env python3
"""
Changelog Synchronization Script

This script automatically synchronizes 'Planned' section from CHANGELOG files
to README files (both English and Chinese versions).

Usage:
    python sync_changelog.py
"""

import re
import os
import sys


def read_file(filepath):
    """Read file content."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        print(f"❌ Error: File not found: {filepath}")
        return None
    except Exception as e:
        print(f"❌ Error reading {filepath}: {e}")
        return None


def write_file(filepath, content):
    """Write content to file."""
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✅ Successfully updated: {filepath}")
        return True
    except Exception as e:
        print(f"❌ Error writing to {filepath}: {e}")
        return False


def extract_planned_features(changelog_content):
    """Extract planned features from CHANGELOG.md.

    Looks for content under '### Planned' or '### 计划中' in the [Unreleased] section.
    Returns a list of planned items.
    """
    # Find Unreleased section (handle both English and Chinese)
    unreleased_pattern_en = r'\[Unreleased\](.*?)(?=\n## \[|\Z)'
    unreleased_pattern_cn = r'\[未发布\](.*?)(?=\n## \[|\Z)'
    
    unreleased_match = re.search(unreleased_pattern_en, changelog_content, re.DOTALL)
    if not unreleased_match:
        unreleased_match = re.search(unreleased_pattern_cn, changelog_content, re.DOTALL)

    if not unreleased_match:
        print("⚠️  Warning: [Unreleased] or [未发布] section not found in CHANGELOG files")
        return []

    unreleased_content = unreleased_match.group(1)

    # Find Planned section (handle both English and Chinese)
    planned_pattern_en = r'### Planned\s*\n(.*?)(?=\n### |\Z)'
    planned_pattern_cn = r'### 计划中\s*\n(.*?)(?=\n### |\Z)'
    
    planned_match = re.search(planned_pattern_en, unreleased_content, re.DOTALL)
    if not planned_match:
        planned_match = re.search(planned_pattern_cn, unreleased_content, re.DOTALL)

    if not planned_match:
        print("⚠️  Warning: '### Planned' or '### 计划中' section not found in [Unreleased]")
        return []

    # Extract list items
    planned_text = planned_match.group(1)
    planned_items = re.findall(r'^-\s+(.+)$', planned_text, re.MULTILINE)

    return planned_items


def extract_added_features(changelog_content):
    """Extract added features from the latest released version in CHANGELOG.

    Looks for content under '### Added' or '### 新增' in the first version section.
    Returns a tuple of (added_items, version_number).
    """
    version_pattern = r'## \[(\d+\.\d+\.\d+)\].*?(?=\n## \[|\Z)'
    version_match = re.search(version_pattern, changelog_content, re.DOTALL)

    if not version_match:
        print("⚠️  Warning: No version section found in CHANGELOG")
        return None

    version_number = version_match.group(1)
    version_content = version_match.group(0)

    added_pattern_en = r'### Added\s*\n(.*?)(?=\n### |\Z)'
    added_pattern_cn = r'### 新增\s*\n(.*?)(?=\n### |\Z)'

    added_match = re.search(added_pattern_en, version_content, re.DOTALL)
    if not added_match:
        added_match = re.search(added_pattern_cn, version_content, re.DOTALL)

    if not added_match:
        print(f"⚠️  Warning: '### Added' or '### 新增' section not found in [{version_number}]")
        return None

    added_text = added_match.group(1)
    added_items = re.findall(r'^-\s+(.+)$', added_text, re.MULTILINE)

    return added_items, version_number


def generate_roadmap_section_english(added_items, version):
    """Generate English roadmap section for README."""
    if not added_items:
        return ""

    section = f"\n### ✅ Released Features (v{version})\n\n"

    for item in added_items:
        section += f"- [x] {item.strip()}\n"

    return section


def generate_roadmap_section_chinese(added_items, version):
    """Generate Chinese roadmap section for README_cn."""
    if not added_items:
        return ""

    section = f"\n### ✅ 已发布功能 (v{version})\n\n"

    for item in added_items:
        section += f"- [x] {item.strip()}\n"

    return section


def update_roadmap(readme_path, added_items, version, is_chinese=False):
    """Update README file with released features in roadmap.

    Args:
        readme_path: Path to the README file
        added_items: List of added feature strings
        version: Version number string
        is_chinese: Whether the README is Chinese version
    """
    content = read_file(readme_path)
    if not content:
        return False

    # Find and replace existing roadmap section
    if is_chinese:
        # Chinese version
        pattern = rf'### ✅ 已发布功能 \(v{re.escape(version)}\).*?(?=\n### |\n## |\Z)'
        replacement = generate_roadmap_section_chinese(added_items, version) if added_items else ""
    else:
        # English version
        pattern = rf'### ✅ Released Features \(v{re.escape(version)}\).*?(?=\n### |\n## |\Z)'
        replacement = generate_roadmap_section_english(added_items, version) if added_items else ""

    # Remove trailing whitespace before next section
    if replacement:
        replacement = replacement.rstrip() + "\n"

    new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)

    # If no roadmap section exists, find where to insert it
    if new_content == content and added_items:
        # Find position to insert (after "License" section)
        if is_chinese:
            insert_marker = "## 🤝 如何贡献"
        else:
            insert_marker = "## 🤝 How to Contribute"

        insert_pos = content.find(insert_marker)
        if insert_pos != -1:
            section_to_insert = generate_roadmap_section_chinese(added_items, version) if is_chinese else generate_roadmap_section_english(added_items, version)
            new_content = content[:insert_pos] + "\n" + section_to_insert + content[insert_pos:]
        else:
            print(f"⚠️  Warning: Could not find roadmap insertion point in {readme_path}")
            return False
    elif not added_items:
        print("ℹ️  No added features found in CHANGELOG")
        return True

    return write_file(readme_path, new_content)


def update_roadmap_planned(readme_path, planned_items, is_chinese=False):
    """Update README file roadmap section with planned features from CHANGELOG.

    Args:
        readme_path: Path to the README file
        planned_items: List of planned feature strings from CHANGELOG
        is_chinese: Whether the README is Chinese version
    """
    content = read_file(readme_path)
    if not content:
        return False

    if is_chinese:
        # Chinese version - find ### 🚧 开发中 section in 开发路线图
        pattern = r'### 🚧 开发中\n(.*?)(?=\n### |\n## |\Z)'
        if planned_items:
            replacement = "### 🚧 开发中\n\n"
            for item in planned_items:
                replacement += f"- [ ] {item.strip()}\n"
            replacement = replacement.rstrip() + "\n"
        else:
            replacement = "### 🚧 开发中\n\n- （暂无计划）\n"
    else:
        # English version - find ### 🚧 In Development section in Development Roadmap
        pattern = r'### 🚧 In Development\n(.*?)(?=\n### |\n## |\Z)'
        if planned_items:
            replacement = "### 🚧 In Development\n\n"
            for item in planned_items:
                replacement += f"- [ ] {item.strip()}\n"
            replacement = replacement.rstrip() + "\n"
        else:
            replacement = "### 🚧 In Development\n\n- (No planned features)\n"

    new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)

    return write_file(readme_path, new_content)


def main():
    """Main synchronization function."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(os.path.dirname(script_dir))
    docs_dir = os.path.join(repo_root, 'docs')
    
    changelog_en_path = os.path.join(docs_dir, 'CHANGELOG.md')
    changelog_cn_path = os.path.join(docs_dir, 'CHANGELOG_CN.md')
    readme_en_path = os.path.join(repo_root, 'README.md')
    readme_cn_path = os.path.join(repo_root, 'README_CN.md')

    print("🚀 Starting Changelog Synchronization...")
    print(f"📁 Repository root: {repo_root}")
    print(f"📁 Docs directory: {docs_dir}\n")

    en_exists = os.path.exists(changelog_en_path)
    cn_exists = os.path.exists(changelog_cn_path)
    
    if not en_exists and not cn_exists:
        print("❌ Error: Neither CHANGELOG.md nor CHANGELOG_CN.md found")
        sys.exit(1)

    en_roadmap_success = True
    en_roadmap_planned_success = True
    cn_roadmap_success = True
    cn_roadmap_planned_success = True
    
    if en_exists:
        print("✅ Found CHANGELOG.md (English)")
        changelog_en_content = read_file(changelog_en_path)
        if changelog_en_content:
            added_result = extract_added_features(changelog_en_content)
            if added_result:
                added_items_en, version_en = added_result
                print(f"📋 Found {len(added_items_en)} released feature(s) in CHANGELOG.md v{version_en}:")
                for i, item in enumerate(added_items_en, 1):
                    print(f"   {i}. {item.strip()}")
                print(f"\n🔄 Updating Development Roadmap in README.md (English)...")
                en_roadmap_success = update_roadmap(readme_en_path, added_items_en, version_en, is_chinese=False)
            else:
                print("ℹ️  No released features found in CHANGELOG.md")

            planned_items_en = extract_planned_features(changelog_en_content)
            if planned_items_en:
                print(f"\n📋 Found {len(planned_items_en)} planned feature(s) in CHANGELOG.md:")
                for i, item in enumerate(planned_items_en, 1):
                    print(f"   {i}. {item.strip()}")
                print("\n🔄 Updating Development Roadmap - Planned section in README.md (English)...")
                en_roadmap_planned_success = update_roadmap_planned(readme_en_path, planned_items_en, is_chinese=False)
            else:
                print("ℹ️  No planned features found in CHANGELOG.md")
        else:
            print("⚠️  Could not read CHANGELOG.md")
            en_roadmap_success = False
            en_roadmap_planned_success = False
    else:
        print("⚠️  CHANGELOG.md not found")

    if cn_exists:
        print("\n✅ Found CHANGELOG_CN.md (Chinese)")
        changelog_cn_content = read_file(changelog_cn_path)
        if changelog_cn_content:
            added_result = extract_added_features(changelog_cn_content)
            if added_result:
                added_items_cn, version_cn = added_result
                print(f"📋 Found {len(added_items_cn)} released feature(s) in CHANGELOG_CN.md v{version_cn}:")
                for i, item in enumerate(added_items_cn, 1):
                    print(f"   {i}. {item.strip()}")
                print(f"\n🔄 Updating 开发路线图 in README_CN.md (Chinese)...")
                cn_roadmap_success = update_roadmap(readme_cn_path, added_items_cn, version_cn, is_chinese=True)
            else:
                print("ℹ️  No released features found in CHANGELOG_CN.md")

            planned_items_cn = extract_planned_features(changelog_cn_content)
            if planned_items_cn:
                print(f"\n📋 Found {len(planned_items_cn)} planned feature(s) in CHANGELOG_CN.md:")
                for i, item in enumerate(planned_items_cn, 1):
                    print(f"   {i}. {item.strip()}")
                print("\n🔄 Updating 开发路线图 - 计划中 section in README_CN.md (Chinese)...")
                cn_roadmap_planned_success = update_roadmap_planned(readme_cn_path, planned_items_cn, is_chinese=True)
            else:
                print("ℹ️  No planned features found in CHANGELOG_CN.md")
        else:
            print("⚠️  Could not read CHANGELOG_CN.md")
            cn_roadmap_success = False
            cn_roadmap_planned_success = False
    else:
        print("⚠️  CHANGELOG_CN.md not found")

    if not (en_roadmap_success and en_roadmap_planned_success and 
            cn_roadmap_success and cn_roadmap_planned_success):
        print("\n❌ Failed to update README files")
        sys.exit(1)

    print("\n✅ Synchronization completed successfully!")
    print("\n📝 Note: Please review the changes before committing:")
    print("   git diff README.md")
    print("   git diff README_CN.md")


if __name__ == "__main__":
    main()
