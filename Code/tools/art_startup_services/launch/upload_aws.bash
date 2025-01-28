#!/bin/bash
directory="${ROSBAG_DIR}/"
bucket="${AWS_BUCKET}"
file=$(ls -t $directory | head -n1)
aws s3 cp "$directory/$file" s3://$bucket
