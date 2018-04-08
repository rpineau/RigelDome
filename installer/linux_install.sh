#!/bin/bash

TheSkyX_Install=`/usr/bin/find ~/Library/Application\ Support/Software\ Bisque/ -name TheSkyXInstallPath.txt`
echo "TheSkyX_Install = $TheSkyX_Install"

if [ ! -f "$TheSkyX_Install" ]; then
    echo TheSkyXInstallPath.txt not found
    TheSkyX_Path=`/usr/bin/find ~/ -maxdepth 3 -name TheSkyX`
    if [ -d "$TheSkyX_Path" ]; then
		TheSkyX_Path="${TheSkyX_Path}/Contents"
    else
	   echo TheSkyX application was not found.
    	exit 1
	 fi
else
	TheSkyX_Path=$(<"$TheSkyX_Install")
fi

echo "Installing to $TheSkyX_Path"


if [ ! -d "$TheSkyX_Path" ]; then
    echo TheSkyX Install dir not exist
    exit 1
fi

cp "./domelist RigelDome.txt" "$TheSkyX_Path/Resources/Common/Miscellaneous Files/"
cp "./RigelDome.ui" "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/"
cp "./RigelDome.png" "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/"
cp "./libRigelDome.so" "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/"

app_owner=`/usr/bin/stat -c "%u" "$TheSkyX_Path" | xargs id -n -u`
if [ ! -z "$app_owner" ]; then
	chown $app_owner "$TheSkyX_Path/Resources/Common/Miscellaneous Files/domelist RigelDome.txt"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/RigelDome.ui"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/RigelDome.png"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/libRigelDome.so"
fi
chmod  755 "$TheSkyX_Path/Resources/Common/PlugIns/DomePlugIns/libRigelDome.so"

