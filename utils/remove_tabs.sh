#!/bin/bash
# Replaces tabs per 4 spaces
find . -name \*.cpp -or -name \*.h -exec sed -i "s/\t/    /g" {} \;
