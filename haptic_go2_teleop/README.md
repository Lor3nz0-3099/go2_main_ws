# haptic_go2_teleop

Pacchetto ROS 2 per la teleoperazione del robot Go2 tramite Geomagic Touch.

## Scopo

Fornisce un bridge tra lo stato del dispositivo aptico e i comandi di velocità / corpo per il robot Go2. Convertendo la posizione del Phantom in comandi `Twist` e `Pose`, permette al supervisore di selezionare la sorgente di teleoperazione corretta.

## Nodo principale

- `haptic_go2_teleop_position_node`
  - Sottoscrive a `/phantom/state` e `/phantom/button`.
  - Pubblica `/teleop_haptic/cmd_vel` e `/teleop_haptic/body_pose`.
  - Usa `/phantom/force_feedback` per inviare feedback aptico.

## Launch

- `ros2 launch haptic_go2_teleop haptic_teleop.launch.py`

Questo launch include anche `omni_state.launch.py` da `omni_common`, quindi deve essere avviato insieme al driver del dispositivo Geomagic Touch.

## Funzionamento

- Mappa gli assi del Phantom ai comandi del robot Go2.
- Applica deadband, filtri e limiti di velocità e altezza.
- Mantiene il reporting della posa del corpo in modo che il supervisore possa commutare tra sorgenti `MYO` e `HAPTIC`.

## Topic chiave

- publish: `/teleop_haptic/cmd_vel`
- publish: `/teleop_haptic/body_pose`
- subscribe: `/phantom/state`
- subscribe: `/phantom/button`
- subscribe: `/phantom/force_feedback`

## Note

Questo pacchetto è pensato per l’uso con il supervisore `go2_supervisor`, che seleziona la sorgente di comando in base alla modalità attiva.
