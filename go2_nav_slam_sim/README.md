# go2_nav_slam_sim

Pacchetto ROS 2 per la simulazione di SLAM e navigazione del robot Go2. Contiene i launch per eseguire lo stack Nav2 con SLAM Toolbox e un bridge per trasformare i punti del LiDAR in scan laser.

## Launch principali

- `ros2 launch go2_nav_slam_sim go2_nav_slam_sim.launch.py`
  - Esegue `ground_truth_to_tf`, `pointcloud_to_laserscan`, `slam_toolbox`, Nav2 e un nodo `rviz2` configurato.
  - Utilizza `sim_time` per la simulazione dei componenti Nav2.

- `ros2 launch go2_nav_slam_sim go2_nav_localization.launch.py`
  - Esegue localizzazione con `nav2_amcl` su una mappa statica.
  - Usa la mappa `warehouse.yaml` preconfigurata e un bridge di odometria dal ground truth.

- è possibile comunque teleoperare tramite comandi su rviz in entrambe le situazioni il robot (2d Goal Pose - Estimate)

## Componenti

- `ground_truth_to_tf`: converte messaggi `nav_msgs/Odometry` in trasformazioni TF tra `odom` e `base_link`.
- `pointcloud_to_laserscan`: converte i dati velodyne in scan laser su `/scan_raw`.
- `slam_toolbox`: esegue SLAM in modalità asincrona.
- `nav2_map_server`, `planner_server`, `controller_server`, `behavior_server`, `smoother_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`: componenti Nav2 per mappatura, pianificazione e controllo.
- `map_saver`: salva la mappa durante l'esecuzione.

## Configurazione

- File parametri: `config/go2_nav_slam_sim.yaml`, `config/go2_nav_localization.yaml`
- Mappe: `maps/house.*`, `maps/warehouse.*`
- RViz: `rviz/go2_nav_slam_sim.rviz`

## Note

Questo pacchetto è pensato per eseguire il comportamento di navigazione/SLAM in simulazione. La modalità di localizzazione usa `amcl` mentre la modalità di esplorazione usa `slam_toolbox`.
