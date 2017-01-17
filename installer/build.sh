#!/bin/bash

mkdir -p ROOT/tmp/RigelDomee_X2/
cp "../RigelDome.ui" ROOT/tmp/RigelDomee_X2/
cp "../RigelDome.png" ROOT/tmp/RigelDomee_X2/
cp "../domelist RigelDome.txt" ROOT/tmp/RigelDomee_X2/
cp "../build/Release/libRigelDome.dylib" ROOT/tmp/RigelDomee_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.RigelDome_X2 --sign "$installer_signature" --scripts Scritps --version 1.0 RigelDome_X2.pkg
pkgutil --check-signature ./RigelDome_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.RigelDome_X2 --scripts Scritps --version 1.0 RigelDome_X2.pkg
fi

rm -rf ROOT
