#!/bin/bash
PACKAGE_DIR=$(rospack find ros_odrive)
cd $PACKAGE_DIR/nodejs
node main.js $@