#!/bin/sh
# Build report/report.pdf from report/report.md (needs pandoc and a TeX distribution with
# xelatex; the Charter font is set in the front matter). Run scripts/run_all.py first so
# the figures in results/figures are current.
set -e
cd "$(dirname "$0")"
pandoc report.md -o report.pdf --pdf-engine=xelatex
