#!/bin/bash
rm stats
while true; do make check; echo $? >> stats; done 
