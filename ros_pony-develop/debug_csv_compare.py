#!/usr/bin/env python3

import csv

def analyze_csv_header(file_path):
    """CSVファイルの1行目を詳細に分析する."""
    print(f"\n=== Analyzing {file_path} ===")
    
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        header_row = next(reader)
        
        print(f"Raw header: {header_row}")
        print(f"Length: {len(header_row)}")
        
        if len(header_row) >= 2:
            message_type = header_row[0]
            topic_name = header_row[1]
            
            print(f"Message type: '{message_type}'")
            print(f"Topic name: '{topic_name}'")
            print(f"Message type length: {len(message_type)}")
            print(f"Message type ASCII: {[ord(c) for c in message_type]}")
            print(f"Message type hex: {[hex(ord(c)) for c in message_type]}")
            print(f"Message type stripped: '{message_type.strip()}'")
            print(f"Expected: 'sensor_msgs/JointState'")
            print(f"Match: {message_type.strip() == 'sensor_msgs/JointState'}")

# 両ファイルを分析
analyze_csv_header('test_data1.csv')
analyze_csv_header('csvPattern/joint_data_20250805_104712.csv') 