#!/bin/bash

# Define paths
GAZEBO_MODELS_DIR="$HOME/.gazebo/models"
OMNI_STAGE_SRC="./omni_stage"

OMNI_STAGE_TARGET="$GAZEBO_MODELS_DIR/omni_stage"
if [ ! -d "$GAZEBO_MODELS_DIR" ]; then
    echo "Creating directory $GAZEBO_MODELS_DIR"
    mkdir -p "$GAZEBO_MODELS_DIR"
fi

# Copy the omni_stage model files to the Gazebo models folder
echo "Copying omni_stage model to $OMNI_STAGE_TARGET"
cp -r "$OMNI_STAGE_SRC" "$GAZEBO_MODELS_DIR"

# Verify the copy operation
if [ $? -eq 0 ]; then
    echo "omni_stage model copied successfully to ~/.gazebo/models."
else
    echo "Error occurred while copying the omni_stage model."
    exit 1
fi

echo "Model installed successfully"
