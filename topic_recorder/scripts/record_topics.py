#!/usr/bin/env python3

import yaml
import subprocess
import os
import rospy

def read_topics_from_yaml(yaml_file):
    try:
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)
        return config.get('topics', [])
    except Exception as e:
        rospy.logerr(f"Failed to read topics from YAML file: {e}")
        return []

def get_unique_bag_filename(output_directory, base_filename):
    """Generate a unique filename by appending an incrementing number."""
    i = 1
    filename = os.path.join(output_directory, base_filename)
    while os.path.exists(filename):
        filename = os.path.join(output_directory, f"{base_filename.rsplit('.', 1)[0]}_{i}.bag")
        i += 1
    return filename

def record_topics(topics, output_directory=".", base_bag_file_name="recorded_topics.bag"):
    if not topics:
        rospy.logerr("No topics to record. Please check your YAML file.")
        return

    topic_str = ' '.join(topics)
    output_path = get_unique_bag_filename(output_directory, base_bag_file_name)
    cmd = f'rosbag record -O {output_path} {topic_str}'
    rospy.loginfo(f'Starting rosbag record with topics: {topic_str} and saving to {output_path}')
    
    try:
        process = subprocess.Popen(cmd, shell=True)
        rospy.loginfo("Recording started. Press Ctrl+C to stop.")
        process.wait()  # Wait for the recording to finish
    except Exception as e:
        rospy.logerr(f"Failed to start rosbag record: {e}")

if __name__ == '__main__':
    rospy.init_node('record_topics_node')

    yaml_file = rospy.get_param('~yaml_file', 'config/topics.yaml')
    output_directory = rospy.get_param('~output_directory', '.')
    base_bag_file_name = rospy.get_param('~bag_file_name', 'recorded_topics.bag')

    topics = read_topics_from_yaml(yaml_file)
    if topics:
        record_topics(topics, output_directory, base_bag_file_name)
    else:
        rospy.logerr("No topics found in YAML file or file could not be read.")

