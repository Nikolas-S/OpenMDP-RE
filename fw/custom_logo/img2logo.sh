#!/bin/bash

# Image to 4-bit BMP Converter
# Requires: ImageMagick (imagemagick), NetPBM (netpbm)

set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 <input_image>"
    echo "Output: logo.bmp, logo.bin"
    exit 1
fi

INPUT="$1"
OUTPUT="logo.bmp"
OUTPUT_BIN="logo.bin"

if ! command -v magick &> /dev/null; then
    echo "Error: ImageMagick not found"
    echo "  install package imagemagick"
    exit 1
fi

if ! command -v ppmtobmp &> /dev/null; then
    echo "Error: NetPBM not found"
    echo "  install package netpbm"
    exit 1
fi

if [ ! -f "$INPUT" ]; then
    echo "Error: Input file not found: $INPUT"
    exit 1
fi

DIMS=$(magick identify -format "%wx%h" "$INPUT")
if [ "$DIMS" != "256x64" ]; then
    echo "Error: Image must be 256x64, got $DIMS"
    exit 1
fi

# ImageMagick uses less colors than you tell it sometimes
if ! magick "$INPUT" -dither FloydSteinberg -define dither:diffusion-amount=100% -colors 17 ppm:- 2>/dev/null | ppmtobmp -bpp 4 > "$OUTPUT" 2>/dev/null; then
    if ! magick "$INPUT" -dither FloydSteinberg -define dither:diffusion-amount=100% -colors 16 ppm:- 2>/dev/null | ppmtobmp -bpp 4 > "$OUTPUT" 2>/dev/null; then
        echo "Error: Conversion failed"
        exit 1
    fi
fi

SIZE=$(stat -f%z "$OUTPUT" 2>/dev/null || stat -c%s "$OUTPUT" 2>/dev/null)
if [ "$SIZE" -ne 8310 ]; then
    echo "Error: Output file size is $SIZE bytes, expected 8310"
    rm -f "$OUTPUT"
    exit 1
fi

cp "$OUTPUT" "$OUTPUT_BIN"
echo "Success: $OUTPUT, $OUTPUT_BIN ($SIZE bytes)"