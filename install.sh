#!/usr/bin/env bash
set -e
echo "Installing MushIO host dependencies..."
pip install -r host/requirements.txt
echo ""
echo "All dependencies installed. Run with:"
echo "  python host/gui.py --demo"
