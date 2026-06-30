import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# Path to your database file
db_path = 'lidar_test_data/lidar_test_data_0.db3'

conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Fetch the serialized message data from the messages table
cursor.execute("SELECT data FROM messages LIMIT 1;")
row = cursor.fetchone()

if row:
    serialized_data = row[0]
    
    # Dynamically get the LidarSweep message type definition
    msg_type = get_message('lidar_msgs/msg/LidarSweep')
    
    # Deserialize the binary data into a readable object
    msg = deserialize_message(serialized_data, msg_type)
    
    print("--- HEADER ---")
    print(msg.header)
    print("\n--- FIRST 5 ANGLES & DISTANCES ---")
    for i in range(min(5, len(msg.angles))):
        print(f"Angle: {msg.angles[i]:.2f}° | Distance: {msg.distances[i]:.2f}mm")
else:
    print("No messages found in the database.")

conn.close()
