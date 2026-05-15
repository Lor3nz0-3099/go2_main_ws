# go2_frontier_explorer

Questo pacchetto implementa una strategia di esplorazione basata sulle frontier del 
map in ROS 2. Il nodo analizza la mappa di occupazione, individua le frontiere tra spazio noto libero e spazio sconosciuto, e invia obiettivi di navigazione a Nav2.

## Componenti principali

- Eseguibile: `frontier_explorer`
- Launch: `ros2 launch go2_frontier_explorer frontier_explorer.launch.py`

## Funzionamento

- Sottoscrive una mappa `OccupancyGrid` dal topic definito da `map_topic` (default `/map`).
- Usa TF per leggere la posizione corrente del robot tra `global_frame` e `base_frame`.
- Individua celle di frontiera come celle sconosciute (-1) adiacenti a celle libere (0).
- Raggruppa le frontier in cluster e ne seleziona una per la navigazione.
- Pubblica lo stato di esplorazione su `/explore/request` e invia goal all'azione `navigate_to_pose`.

## Topic e azioni

- subscribe: `map_topic` (default `/map`)
- publish: `/explore/request` (std_msgs/Bool)
- action client: `navigate_to_pose` (nav2_msgs/NavigateToPose)

## Parametri principali

- `map_topic`: topic della mappa occupazione.
- `global_frame`: frame globale della mappa.
- `base_frame`: frame base del robot.
- `planner_period_sec`: periodo di pianificazione.
- `min_frontier_cluster`: dimensione minima della cluster di fronte.
- `goal_timeout_sec`: timeout dell'obiettivo di navigazione.
- `goal_min_distance`: distanza minima dal goal precedente.
- `goal_max_distance`: distanza massima per un nuovo goal.
- `goal_offset_from_frontier`: offset dal fronte per il goal.
- `min_goal_clearance`: clearance minima richiesta.
- `blacklist_radius`: raggio di blacklist per obiettivi falliti.
- `exploration_stop_ratio`: soglia oltre la quale l'esplorazione è considerata completata.

## Note

Questa componente deve essere usata insieme a un nodo Nav2 attivo (`bt_navigator` / `nav2_planner`) e a un gestore di modalità come `go2_supervisor` per osservare il flusso tra esplorazione, teleop e navigazione automatica.
