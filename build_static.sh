#!/bin/sh

odin build . -build-mode:static -out:geometry.lib -no-type-assert -no-bounds-check -no-rpath -o:speed -source-code-locations:none