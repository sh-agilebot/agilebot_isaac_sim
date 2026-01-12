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


def generate_planned_section_english(planned_items):
    """Generate English planned section for README."""
    if not planned_items:
        return ""

    section = "\n## 🚀 Upcoming Features\n\n"
    section += "> This section is automatically synchronized from [CHANGELOG.md](./docs/CHANGELOG.md)\n\n"

    for item in planned_items:
        section += f"- {item.strip()}\n"

    return section


def generate_planned_section_chinese(planned_items):
    """Generate Chinese planned section for README_cn."""
    if not planned_items:
        return ""

    section = "\n## 🚀 即将推出的功能\n\n"
    section += "> 本部分从 [CHANGELOG_CN.md](./docs/CHANGELOG_CN.md) 自动同步\n\n"

    for item in planned_items:
        section += f"- {item.strip()}\n"

    return section


def update_readme(readme_path, planned_items, is_chinese=False):
    """Update README file with planned features.

    Args:
        readme_path: Path to the README file
        planned_items: List of planned feature strings
        is_chinese: Whether the README is Chinese version
    """
    content = read_file(readme_path)
    if not content:
        return False

    # Find and replace existing planned section
    if is_chinese:
        # Chinese version
        pattern = r'## 🚀 即将推出的功能.*?(?=\n## \w+|\Z)'
        replacement = generate_planned_section_chinese(planned_items) if planned_items else ""
    else:
        # English version
        pattern = r'## 🚀 Upcoming Features.*?(?=\n## \w+|\Z)'
        replacement = generate_planned_section_english(planned_items) if planned_items else ""

    # Remove trailing whitespace before next section
    if replacement:
        replacement = replacement.rstrip() + "\n\n"

    new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)

    # If no planned section exists, find where to insert it
    if new_content == content and planned_items:
        # Find position to insert (before "📁 项目结构" or "📁 Project Structure")
        if is_chinese:
            insert_marker = "## 📁 项目结构"
        else:
            insert_marker = "## 📁 Project Structure"

        insert_pos = content.find(insert_marker)
        if insert_pos != -1:
            section_to_insert = generate_planned_section_chinese(planned_items) if is_chinese else generate_planned_section_english(planned_items)
            new_content = content[:insert_pos] + section_to_insert + content[insert_pos:]
        else:
            print(f"⚠️  Warning: Could not find insertion point in {readme_path}")
            return False
    elif not planned_items:
        print("ℹ️  No planned features found in CHANGELOG files")
        return True

    return write_file(readme_path, new_content)


def main():
    """Main synchronization function."""
    # Get repository root directory (parent of .github)
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

    # Check which CHANGELOG files exist
    changelog_exists = False
    if os.path.exists(changelog_en_path):
        print("✅ Found CHANGELOG.md (English)")
        changelog_exists = True
    if os.path.exists(changelog_cn_path):
        print("✅ Found CHANGELOG_CN.md (Chinese)")
        changelog_exists = True
    
    if not changelog_exists:
        print("❌ Error: Neither CHANGELOG.md nor CHANGELOG_CN.md found")
        sys.exit(1)

    # Read CHANGELOG (prefer English, fallback to Chinese)
    changelog_content = read_file(changelog_en_path)
    if not changelog_content:
        changelog_content = read_file(changelog_cn_path)
        if not changelog_content:
            print("❌ Error: Could not read any CHANGELOG file")
            sys.exit(1)
        print("ℹ️  Using CHANGELOG_CN.md (Chinese version)")
        is_using_chinese = True
    else:
        is_using_chinese = False

    # Extract planned features
    planned_items = extract_planned_features(changelog_content)

    if not planned_items:
        print("ℹ️  No planned features found. Nothing to synchronize.")
        print("✅ Synchronization completed (no changes needed)")
        return

    print(f"📋 Found {len(planned_items)} planned feature(s) in CHANGELOG:")
    for i, item in enumerate(planned_items, 1):
        print(f"   {i}. {item.strip()}")

    # Update README files
    print("\n🔄 Updating README files...")
    en_success = update_readme(readme_en_path, planned_items, is_chinese=False)
    cn_success = update_readme(readme_cn_path, planned_items, is_chinese=True)

    if not (en_success and cn_success):
        print("❌ Failed to update README files")
        sys.exit(1)

    print("\n✅ Synchronization completed successfully!")
    print("\n📝 Note: Please review the changes before committing:")
    print("   git diff README.md")
    print("   git diff README_CN.md")


if __name__ == "__main__":
    main()
