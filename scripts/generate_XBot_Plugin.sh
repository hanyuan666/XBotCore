#!/bin/sh

if [ $# -ne 1 ]; then
    echo "Error: one input parameter expected, the RT Plugin name"
    exit 1
fi


#create plugin's folder
mkdir -p $1
cd $1

# Local copy of XCM skeleton
cp -r $XBOT_ROOT/external/XBotCore/skeleton/control/* .

# find and replace 
find . -maxdepth 3 -type f -not -path '*/\.*' -exec sed -i -e "s/_MODULE_PREFIX_/$1/g" {} \;
# rename
find . -maxdepth 3 -type f -not -path '*/\.*' -not -name "CMakeLists.txt" -not -name "README.md" -not -name "FindXenomai.cmake" -not -name "MacroYCMInstallLibrary.cmake" -exec bash -c 'dir=$(dirname $0) && file=$(basename $0) && mv $0 "$dir/$1_$file"' {} $1 \;

mv "./include/plugin" "./include/$1"

echo "XBot RT Plugin skeleton is ready. Have fun!"
