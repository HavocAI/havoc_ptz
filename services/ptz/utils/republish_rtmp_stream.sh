#!/bin/bash

# ##############################################################################
# FFmpeg RTMP/RTMPS Republishing Script
#
# This script continuously reads a video stream from a source RTSP URL and
# republishes it to a destination RTMPS URL using ffmpeg.
#
# It first checks if the source stream is available using ffprobe. Once the
# source is confirmed, it starts ffmpeg.
#
# It runs in an infinite loop to ensure that if ffmpeg exits for any
# reason (e.g., network drop, source stream unavailable), it will
# re-check the source and automatically attempt to restart.
# ##############################################################################

# --- Configuration ---
# Set the source and destination stream URLs here.
# The source stream with authentication credentials.
SOURCE_URL="rtsp://havoc-ptz1:1nspectR@192.168.168.106:554/stream1"

# The destination where the stream will be republished.
DESTINATION_URL="rtmps://media.stg.havocai.net:1936/havoc/vehicle/100/camera/ptz"

# --- Main Loop ---
# This loop will run forever, relaunching ffmpeg if it ever exits.
while true
do
  # --- Source Stream Check ---
  # Use ffprobe to check if the source stream is available before starting ffmpeg.
  # The '-v quiet' flag suppresses all output, we only care about the exit code.
  # The loop continues as long as ffprobe returns a non-zero exit code (failure).
  echo "Checking for available source stream at $SOURCE_URL..."
  while ! ffprobe -v quiet "$SOURCE_URL"
  do
    echo "Source stream not available. Retrying in 5 seconds..."
    sleep 5
  done

  echo "Source stream detected. Starting ffmpeg..."
  echo "Source: $SOURCE_URL"
  echo "Destination: $DESTINATION_URL"

  # Run ffmpeg with options to copy codecs and republish the stream.
  ffmpeg \
    -v error \
    -rtsp_transport tcp \
    -i "$SOURCE_URL" \
    -vf "transpose=2,transpose=2" \
    -c:v libx264 \
    -c:a copy \
    -f flv \
    "$DESTINATION_URL"

  # If ffmpeg exits, wait for 3 seconds before the main loop restarts the whole process.
  echo "ffmpeg exited. Restarting process in 3 seconds..."
  sleep 3
done
