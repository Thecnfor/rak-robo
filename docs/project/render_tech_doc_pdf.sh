#!/usr/bin/env bash
# Render the technical doc to PDF using pandoc + xelatex (TeX Live).
#
# Requirements (run on a TeX Live host):
#   sudo apt-get install pandoc texlive-xetex texlive-lang-chinese \
#                        texlive-fonts-recommended fonts-noto-cjk
#
# Usage:
#   cd /var/workspace/docker/isaac/workspace
#   bash docs/project/render_tech_doc_pdf.sh [TEAM_NAME]
#
# Output:
#   docs/project/技术文档-${TEAM_NAME}-预选赛.pdf
set -euo pipefail

TEAM_NAME="${1:-XX队}"
REPO_ROOT="${REPO_ROOT:-/var/workspace/docker/isaac/workspace}"
DOC_DIR="${REPO_ROOT}/docs/project"
SRC_MD="${DOC_DIR}/技术文档-XX队-预选赛.md"
DST_PDF="${DOC_DIR}/技术文档-${TEAM_NAME}-预选赛.pdf"

if ! command -v pandoc >/dev/null 2>&1; then
  echo "[$(date -Is)] pandoc not installed; install via:"
  echo "    sudo apt-get install pandoc"
  exit 1
fi
if ! command -v xelatex >/dev/null 2>&1; then
  echo "[$(date -Is)] xelatex not installed; install TeX Live:"
  echo "    sudo apt-get install texlive-xetex texlive-fonts-recommended"
  exit 1
fi

if [[ ! -f "${SRC_MD}" ]]; then
  echo "[$(date -Is)] source markdown not found: ${SRC_MD}"
  exit 1
fi

# Stage the diagrams so the PDF embeds them at the right places.
# Markdown references images via relative paths; we keep the same paths.
DIAGRAM_DIR="${DOC_DIR}/diagrams"
if [[ -d "${DIAGRAM_DIR}" ]]; then
  cp -f "${DIAGRAM_DIR}"/*.png "${DOC_DIR}/" 2>/dev/null || true
fi

echo "[$(date -Is)] rendering ${SRC_MD} -> ${DST_PDF}"
pandoc "${SRC_MD}" \
    -o "${DST_PDF}" \
    --pdf-engine=xelatex \
    -V mainfont="Noto Sans CJK SC" \
    -V monofont="DejaVu Sans Mono" \
    -V geometry:margin=2cm \
    -V colorlinks=true -V linkcolor=blue!50!black -V urlcolor=blue!50!black \
    -V title-meta="技术文档-${TEAM_NAME}-预选赛" \
    -V author-meta="${TEAM_NAME}" \
    -V date-meta="$(date +%Y-%m-%d)" \
    -V lang="zh-CN" \
    --toc --toc-depth=2 \
    -M link-citations=true \
    -M figure-caption=true

if [[ -d "${DIAGRAM_DIR}" ]]; then
  # Clean up the staged PNGs in docs/project/ root; the canonical
  # source is docs/project/diagrams/.
  for f in "${DOC_DIR}"/*.png; do
    [[ -f "$f" ]] || continue
    bn=$(basename "$f")
    if [[ -f "${DIAGRAM_DIR}/${bn}" ]]; then
      rm -f "$f"
    fi
  done
fi

echo "[$(date -Is)] done: ${DST_PDF}"
ls -la "${DST_PDF}"
