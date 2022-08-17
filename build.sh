#!/usr/bin/env bash
# ======================================
# Name        : build.sh
# Author      : Arghadeep Mazumder
# Version     : 1.0
# Copyright   : -
# Description :
# ======================================

helpFunction()
{
   echo ""
   echo "Usage: $0 -b CMAKE_BUILD_TYPE -e BUILD_EXAMPLE"
   echo -e "\t-b CMAKE_BUILD_TYPE: Release | Debug"
   echo -e "\t-e BUILD_EXAMPLE: ON | OFF"
   exit 1 # Exit script after printing help
}
while getopts "b:e:" opt
do
   case "$opt" in
      b ) CMAKE_BUILD_TYPE="$OPTARG" ;;
      e ) BUILD_EXAMPLE="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

if [ -z "$CMAKE_BUILD_TYPE" ] || [ -z "$BUILD_EXAMPLE" ]
then
   echo "Some or all of the parameters are empty";
   helpFunction
fi


if [ -d "build" ]; then
  echo "Build Directory Exist"
  echo "Cleaning Build Directory"
  cd build && rm -rf *
else
  echo "Build Directory Doesn't Exist"
  echo "Creating Build Directory"
  mkdir build && cd build
fi

echo -e "\n---------- PARAMETERS ----------"
echo "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
echo "BUILD_EXAMPLE=${BUILD_EXAMPLE}"
echo -e "--------------------------------\n"
cmake .. -DBUILD_EXAMPLE=$BUILD_EXAMPLE -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE
make -j6