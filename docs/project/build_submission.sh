#!/usr/bin/env bash
# Build the submission zip per `docs/project/submission_checklist.md`.
#
# Usage:
#   cd /var/workspace/docker/isaac/workspace
#   bash docs/project/build_submission.sh [TEAM_NAME]
#
# Default team name is "XX队" (the placeholder). Pass a real team
# name to update the zip filename and the technical doc title.
set -euo pipefail

TEAM_NAME="${1:-XX队}"
REPO_ROOT="${REPO_ROOT:-/var/workspace/docker/isaac/workspace}"
DOC_DIR="${REPO_ROOT}/docs/project"
SUBMISSION_ROOT="/tmp/submission_${TEAM_NAME}"
ZIP_NAME="双臂-${TEAM_NAME}-预选赛.zip"

cd "${REPO_ROOT}"

# Rebuild the technical doc to PDF if pandoc + xelatex are present.
TECHNICAL_DOC_MD="${DOC_DIR}/技术文档-XX队-预选赛.md"
TECHNICAL_DOC_PDF="${DOC_DIR}/技术文档-${TEAM_NAME}-预选赛.pdf"
if command -v pandoc >/dev/null 2>&1 && command -v xelatex >/dev/null 2>&1; then
  echo "[$(date -Is)] rendering technical doc PDF"
  pandoc "${TECHNICAL_DOC_MD}" \
      -o "${TECHNICAL_DOC_PDF}" \
      --pdf-engine=xelatex \
      -V mainfont="Noto Sans CJK SC" \
      -V geometry:margin=2cm \
      --toc --toc-depth=2
else
  echo "[$(date -Is)] WARNING: pandoc/xelatex not available; PDF must be"
  echo "    rendered by hand on a host with TeX Live before submission."
  echo "    Source: ${TECHNICAL_DOC_MD}"
fi

# Stage the submission tree.
rm -rf "${SUBMISSION_ROOT}"
mkdir -p "${SUBMISSION_ROOT}/videos"
mkdir -p "${SUBMISSION_ROOT}/technical_doc"
mkdir -p "${SUBMISSION_ROOT}/工程文件/usd_scenes"
mkdir -p "${SUBMISSION_ROOT}/工程文件/action_graphs"
mkdir -p "${SUBMISSION_ROOT}/工程文件/python_scripts"
mkdir -p "${SUBMISSION_ROOT}/工程文件/configs"
mkdir -p "${SUBMISSION_ROOT}/工程文件/maps"

# Technical doc.
if [[ -f "${TECHNICAL_DOC_PDF}" ]]; then
  cp "${TECHNICAL_DOC_PDF}" "${SUBMISSION_ROOT}/technical_doc/"
fi
cp "${TECHNICAL_DOC_MD}" "${SUBMISSION_ROOT}/technical_doc/"

# Engineering files: every first-party package the team owns.
# 4 competition packages per `CLAUDE.md::Package boundaries`:
#   dual_arm_pkg, perception_competition_pkg, bridge_competition_pkg,
#   drone_navigation_pkg, competition_orchestrator_pkg
for pkg in dual_arm_pkg perception_competition_pkg bridge_competition_pkg \
          drone_navigation_pkg competition_orchestrator_pkg \
          grasp_demo_pkg grasp_demo_interfaces nav2_demo_pkg isaac_ros2_control; do
  if [[ -d "src/${pkg}" ]]; then
    mkdir -p "${SUBMISSION_ROOT}/工程文件/python_scripts/${pkg}"
    rsync -a --exclude='build/' --exclude='install/' --exclude='log/' \
        --exclude='__pycache__/' --exclude='*.pyc' --exclude='test/' \
        --exclude='.pytest_cache/' --exclude='weights/' \
        "src/${pkg}/" \
        "${SUBMISSION_ROOT}/工程文件/python_scripts/${pkg}/"
  fi
done

# Action-graph screenshots live in `docs/project/diagrams/`.
cp "${DOC_DIR}/diagrams"/*.png "${SUBMISSION_ROOT}/工程文件/action_graphs/" 2>/dev/null || true
mkdir -p "${SUBMISSION_ROOT}/工程文件/configs/competition_orchestrator_pkg"
cp -r src/perception_competition_pkg/config/* "${SUBMISSION_ROOT}/工程文件/configs/perception_competition_pkg/" 2>/dev/null || true
cp -r src/drone_navigation_pkg/config/*    "${SUBMISSION_ROOT}/工程文件/configs/drone_navigation_pkg/"    2>/dev/null || true
cp -r src/bridge_competition_pkg/config/*  "${SUBMISSION_ROOT}/工程文件/configs/bridge_competition_pkg/"  2>/dev/null || true
cp -r src/competition_orchestrator_pkg/config/* "${SUBMISSION_ROOT}/工程文件/configs/competition_orchestrator_pkg/" 2>/dev/null || true

# Maps (Nav2).
if [[ -d "src/nav2_demo_pkg/maps" ]]; then
  cp -r src/nav2_demo_pkg/maps/* "${SUBMISSION_ROOT}/工程文件/maps/"
fi

# USD scenes (workspace-local copies; the actual assets live outside
# the repo at /var/workspace/docker/isaac/scenes/active/).
# Copy the small ones for completeness; the full Isaac assets are
# deployed separately and are not in this repo.
if [[ -d "src/dual_arm_pkg/config" ]]; then
  cp src/dual_arm_pkg/config/* "${SUBMISSION_ROOT}/工程文件/configs/" 2>/dev/null || true
fi

# Top-level engineering README.
cp "${DOC_DIR}/diagrams/README.md" "${SUBMISSION_ROOT}/工程文件/README.md"

# Top-level videos: expected to be staged by the operator.
# The script does not auto-collect videos from VNC recordings.

# Build the zip.
ZIP_PATH="${REPO_ROOT}/${ZIP_NAME}"
echo "[$(date -Is)] building ${ZIP_PATH}"
rm -f "${ZIP_PATH}"
cd "${SUBMISSION_ROOT%/*}"
# Use zip if available, fall back to Python's zipfile.
if command -v zip >/dev/null 2>&1; then
  zip -qr "${ZIP_PATH}" "${SUBMISSION_ROOT##*/}"
else
  python3 -c "
import os, zipfile, sys
root = '${SUBMISSION_ROOT}'
out = '${ZIP_PATH}'
with zipfile.ZipFile(out, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
    for dirpath, dirnames, filenames in os.walk(root):
        for fn in filenames:
            full = os.path.join(dirpath, fn)
            arc = os.path.relpath(full, os.path.dirname(root))
            zf.write(full, arc)
print('wrote', out)
"
fi

echo "[$(date -Is)] md5:"
md5sum "${ZIP_PATH}" | tee "${ZIP_PATH}.md5"
