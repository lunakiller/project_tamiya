#!/bin/sh
# from https://forum.kicad.info/t/eeschema-export-plot-to-svg-and-crop-to-the-size-of-schematics/29348

find . -type f -iname "*.svg" -exec $SHELL -c '
  echo Processing file: "$0"
  inkscape "$0" --export-area-drawing \
                --export-plain-svg \
                --export-type=svg \
                --export-filename="$0" \
                --export-overwrite
' {} \;
