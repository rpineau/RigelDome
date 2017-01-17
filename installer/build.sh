#!/bin/bash

mkdir -p ROOT/tmp/NexDome_X2/
cp "../NexDome.ui" ROOT/tmp/NexDome_X2/
cp "../NexDome.png" ROOT/tmp/NexDome_X2/
cp "../domelist NexDome.txt" ROOT/tmp/NexDome_X2/
cp "../build/Release/libNexDome.dylib" ROOT/tmp/NexDome_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.NexDome_X2 --sign "$installer_signature" --scripts Scritps --version 1.0 NexDome_X2.pkg
pkgutil --check-signature ./NexDome_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.NexDome_X2 --scripts Scritps --version 1.0 NexDome_X2.pkg
fi

rm -rf ROOT
