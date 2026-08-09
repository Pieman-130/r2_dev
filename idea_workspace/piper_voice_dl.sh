#!/bin/bash
# Download all Piper en_US voices
# Each voice needs both the .onnx model and .onnx.json config

DEST=~/.local/share/piper
mkdir -p "$DEST"

BASE="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US"

download_voice() {
    local name=$1
    local quality=$2
    local stem="en_US-${name}-${quality}"
    echo "Downloading ${stem}..."
    wget -q --show-progress -O "$DEST/${stem}.onnx" \
        "${BASE}/${name}/${quality}/${stem}.onnx"
    wget -q --show-progress -O "$DEST/${stem}.onnx.json" \
        "${BASE}/${name}/${quality}/${stem}.onnx.json"
}

# --- Single-speaker voices ---
download_voice amy       medium
download_voice bryce     medium
download_voice danny     low
download_voice hfc_female medium
download_voice hfc_male  medium
download_voice joe       medium
download_voice john      medium
download_voice kathleen  low
download_voice kristin   medium
download_voice lessac    low
download_voice lessac    medium
download_voice lessac    high
download_voice ryan      low
download_voice ryan      medium
download_voice ryan      high

# --- Multi-speaker voices ---
download_voice arctic    medium   # 18 speakers
download_voice vctk      medium   # 109 speakers (large!)

echo "Done! Voices saved to $DEST"
