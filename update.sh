#!/usr/bin/env bash

set -e

echo "Updating main repository..."
git pull

echo "Updating submodules..."
git submodule update --init --remote --recursive

echo "Done!"
