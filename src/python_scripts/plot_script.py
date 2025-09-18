import pandas as pd
import matplotlib.pyplot as plt

# Load CSVs
gt = pd.read_csv('ground_truth.csv')
odom = pd.read_csv('odom.csv')
scan = pd.read_csv('ekf_pose.csv')

# Ensure numeric
for df in [gt, odom, scan]:
    df['x'] = pd.to_numeric(df['x'], errors='coerce')
    df['y'] = pd.to_numeric(df['y'], errors='coerce')

# 1️⃣ Ground Truth
plt.figure(figsize=(6,6))
plt.plot(gt['x'].to_numpy(), gt['y'].to_numpy(), 'k-.', linewidth=2)
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Ground Truth Trajectory')
plt.grid(True)
plt.axis('equal')

# 2️⃣ Odometry
plt.figure(figsize=(6,6))
plt.plot(odom['x'].to_numpy(), odom['y'].to_numpy(), 'r--')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Odometry Trajectory')
plt.grid(True)
plt.axis('equal')

# 3️⃣ Scan Pose
plt.figure(figsize=(6,6))
plt.plot(scan['x'].to_numpy(), scan['y'].to_numpy(), 'b:')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Scan Pose Trajectory')
plt.grid(True)
plt.axis('equal')

plt.show()
