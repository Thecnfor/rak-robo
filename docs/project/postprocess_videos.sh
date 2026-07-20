#!/usr/bin/env bash
# Post-process raw VNC recordings:
#   - re-encode to H.264 (libx264) yuv420p for compatibility
#   - burn a top-left timestamp + bottom-right team watermark
#   - clip to 3 minutes max
#   - normalise audio (loudnorm EBU R128)
#
# Usage:
#   cd /var/workspace/docker/isaac/workspace
#   bash docs/project/postprocess_videos.sh [TEAM_NAME]
#
# Reads from `videos_raw/`, writes to `videos/` with the exact
# filenames required by `docs/project/submission_checklist.md`.
set -euo pipefail

TEAM_NAME="${1:-XX队}"
REPO_ROOT="${REPO_ROOT:-/var/workspace/docker/isaac/workspace}"
RAW_DIR="${REPO_ROOT}/videos_raw"
OUT_DIR="${REPO_ROOT}/videos"

mkdir -p "${OUT_DIR}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "[$(date -Is)] ffmpeg not installed; install via:"
  echo "    sudo apt-get install ffmpeg libx264-dev"
  exit 1
fi

# Watermark filter chain: top-left timecode + bottom-right team watermark.
WATERMARK_FILTER="drawtext=text='${TEAM_NAME} 预选赛':fontsize=24:fontcolor=white:box=1:boxcolor=black@0.5:boxborderw=8:x=W-tw-20:y=H-th-20,drawtext=text='%{pts\\:hms}':fontsize=18:fontcolor=white:box=1:boxcolor=black@0.5:boxborderw=6:x=20:y=20"

for raw in "${RAW_DIR}"/*.mp4 "${RAW_DIR}"/*.mov; do
  [[ -f "${raw}" ]] || continue
  base=$(basename "${raw}")
  stripped=$(echo "${base}" | sed -E 's/^[0-9]{8}_[0-9]{6}-?//; s/_raw\././')
  out="${OUT_DIR}/${stripped}"
  echo "[$(date -Is)] ${raw} -> ${out}"
  ffmpeg -hide_banner -loglevel error -y \
      -i "${raw}" \
      -t 180 \
      -vf "${WATERMARK_FILTER},scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:black" \
      -c:v libx264 -preset slow -crf 20 -profile:v high -level 4.0 -pix_fmt yuv420p \
      -c:a aac -b:a 192k -ac 2 \
      -af loudnorm=I=-16:TP=-1.5:LRA=11 \
      -movflags +faststart \
      -metadata title="${stripped%.mp4}" \
      -metadata artist="${TEAM_NAME}" \
      -metadata description="ROBOTAC 2026 预选赛" \
      "${out}" || {
        rc=$?
        echo "[$(date -Is)] ffmpeg failed (rc=${rc}) for ${raw}; see logs"
        continue
      }
done

echo "[$(date -Is)] done; outputs in ${OUT_DIR}/"
du -h "${OUT_DIR}"/*.mp4 2>/dev/null | sort -h || true
