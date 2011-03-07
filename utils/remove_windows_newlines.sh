#!/bin/bash
find . -name \*.cpp -or -name \*.h -exec perl -pi -e 's/\r\n?/\n/' {} \;
