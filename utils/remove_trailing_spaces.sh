#!/bin/bash
# removes all trailing whitespaces from source files in spatosc
find . -name \*.cpp -or -name \*.h -exec sed -i 's/[ \t]*$//' {} \;
