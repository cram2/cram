#!/bin/bash
set -e
version="$1"
if [ -z "$version" ]; then
    exec >&2
    echo Usage: `basename "$0"` VERSION
    exit 64
fi

dir="cl-png-$version"
rm -rf "$dir"
mkdir "$dir"
tar cf - `cat MANIFEST` | tar -C "$dir" -xf -
tar cvzf "$dir.tar.gz" "$dir"
rm -rf "$dir"
