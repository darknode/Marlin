#!/usr/bin/env bash

set -e

# Clean PIO chache
rm -rf ~/.platformio/.cache/*
# Clean PIO packages
rm -rf ~/.platformio/packages/*
# Clean PIO platforms
rm -rf ~/.platformio/platforms/*
rm -rf ./.pio