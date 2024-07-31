#!/bin/bash

# Define the executable name
EXECUTABLE=armgui.exe

# Verify if the executable is 64-bit
file $EXECUTABLE | grep "x86-64"
if [ $? -ne 0 ]; then
  echo "Error: The executable is not a 64-bit executable."
  exit 1
fi

# Create a directory for the output
OUTPUT_DIR=output
mkdir -p $OUTPUT_DIR

# Copy the executable to the output directory
cp $EXECUTABLE $OUTPUT_DIR

# Use ldd to find all the required DLLs and copy them to the output directory
ldd $EXECUTABLE | grep "=> /" | awk '{print $3}' | while read -r lib; do
  if [[ "$lib" == *"32"* ]]; then
    echo "Warning: Skipping 32-bit library $lib"
  else
    cp "$lib" $OUTPUT_DIR
  fi
done

# Ensure libwinpthread-1.dll is included
WINPTHREAD_DLL=$(find /mingw64/bin -name "libwinpthread-1.dll")
if [ -n "$WINPTHREAD_DLL" ]; then
  cp "$WINPTHREAD_DLL" $OUTPUT_DIR
else
  echo "Error: libwinpthread-1.dll not found in /mingw64/bin"
  exit 1
fi

echo "All required 64-bit DLLs have been copied to the $OUTPUT_DIR directory."
