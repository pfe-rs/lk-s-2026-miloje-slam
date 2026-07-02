# Cilj:
### Da se ceo SLAM nalazi u folderu /src/my_robot_slam/my_robot_slam

## Slam testing se pokrece sa jednim fajlom i komandom ". ./rebuild.sh" nakon sto se taj fajl napravi executable-nim

# Folderi:

### Rviz
Rviz folder sluzi za vizuelizaciju skenova lidara u programu rviz koji je deo ROS-a.

### SLAM-Testing
Objedinjeni fajlovi koji ce se koristiti u konacnom SLAM-u, prelazni folder tokom sredjivanja gita.

### RPLidarA1Viz
Folder koji sluzi za vizuelizaciju i simulaciju lidara, prelazni folder tokom sredjivanja gita.

### src
Neophodni automatski generisani ros folder.

# TODO:
# Kako pokrenuti SLAM i ROS


# SLAM-Testing

SLAM logika u folderu SLAM-Testing/src/lidar_pkg/scripts

## 1. cvor - lidar_scan_node uzima sken sa lidara i objavljuje ga u LidarSweep formatu
## 2. cvor - slam_mapping_node uzima LidarSweep i objavljuje mapu na /map
## 3. cvor - path_planner, racuna gde MILOJE treba da ode, cita odometry i map, objavljuje na global_path
## 4. cvor - motion_planner, racuna kako MILOJE treba da ode kuda treba i sta se salje na arduino, na kraju poziva lidar_scan_node

### vector_deducer vise nije neophodan, racun je optimizovan.
### SLAM_mapping mozda bude invertovan, ispravka je sledeca linija koda, ako to bude slucaj:
icp(self._last_local_scan, current_local)

# TO DO:

## Path planner
### Zameni A* u path planeru sa BFS
### Kod nije proveren posle 130 linije

## Motion planner
### Nije proveren niti doradjen uopste