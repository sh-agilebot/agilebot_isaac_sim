# GitHub Workflows

This directory contains GitHub Actions workflow files for the Agilebot Isaac Sim project.

## Workflows

### sync-changelog.yml

**Purpose**: Automatically synchronizes planned features from `CHANGELOG.md` and `CHANGELOG_CN.md` to `README.md` and `README_CN.md` files.

**Trigger**:
- Automatic: When `CHANGELOG.md` or `CHANGELOG_CN.md` is pushed to `main` or `master` branch
- Manual: Via GitHub Actions "workflow_dispatch" event

**How it works**:
1. Runs on Ubuntu latest
2. Checks out the repository
3. Sets up Python 3.11
4. Executes the sync script: `.github/scripts/sync_changelog.py`
   - Reads from CHANGELOG.md (English) or CHANGELOG_CN.md (Chinese)
5. Shows changed files
6. Commits changes if any (skips CI)

**Why skip CI**: The commit uses `[skip ci]` to avoid triggering the workflow recursively.

---

## Scripts

### sync_changelog.py

Located in `.github/scripts/` directory.

**Purpose**: Reads the `### Planned` section from `CHANGELOG.md` and updates both English and Chinese README files.

**Features**:
- ✅ Extracts planned features from CHANGELOG.md
- ✅ Generates English version for README.md
- ✅ Generates Chinese version for README_CN.md
- ✅ Preserves existing formatting
- ✅ Provides detailed progress and error messages
- ✅ Handles UTF-8 encoding properly

---

## Usage

### Manual Trigger

To manually trigger the workflow:

1. Go to GitHub repository page
2. Navigate to **Actions** tab
3. Select **Sync Changelog to README** workflow
4. Click **Run workflow** button
5. (Optional) Select branch if needed

### Automatic Trigger

Simply push changes to `CHANGELOG.md`:

```bash
git add CHANGELOG.md
git commit -m "feat: add new planned features"
git push
```

The workflow will automatically:
- Detect the CHANGELOG.md change
- Run the sync script
- Update both README files
- Commit the changes

---

## File Structure

```
.github/
├── workflows/
│   ├── sync-changelog.yml      # GitHub Action workflow definition
│   └── README.md             # This file
└── scripts/
    └── sync_changelog.py         # Synchronization script
```

---

## Troubleshooting

### Workflow doesn't trigger

**Check**:
- Are you pushing to `main` or `master` branch?
- Did the `CHANGELOG.md` file actually change?
- Is the workflow file valid YAML?

### Workflow fails with permissions error

**Check**:
- Repository has Actions enabled
- Workflow file has correct permissions
- Token has write access

### Script errors

**Check**:
- CHANGELOG.md has correct format with `### Planned` section
- Repository has both README.md and README_CN.md
- Python version is 3.11 (required)

---

## Best Practices

1. **Keep CHANGELOG.md up to date**: Always add planned features before other changes
2. **Review automatic commits**: The workflow will auto-commit, review before merging
3. **Follow changelog format**: Use standard CHANGELOG.md format
4. **Test locally first**: Run script locally if you want to test changes

---

## Additional Resources

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Workflow Syntax](https://docs.github.com/en/actions/reference/workflow-syntax-for-github-actions)
- [CHANGELOG.md](../CHANGELOG.md) - Main changelog file
