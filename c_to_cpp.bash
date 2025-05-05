#!/bin/bash
echo "Moving file"
mv Core/Src/main.c Core/Src/main.cpp


# Path to the target file
FILE="./cmake/stm32cubemx/CMakeLists.txt"

# Check if the file exists
if [[ -f "$FILE" ]]; then
    # Use sed to replace all occurrences of 'main.c' with 'main.cpp'
    sed -i 's/main\.c/main.cpp/g' "$FILE"
    echo "Replaced 'main.c' with 'main.cpp' in $FILE"
else
    echo "File not found: $FILE"
    exit 1
fi