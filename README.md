# Workspace `src`

Questa cartella contiene i pacchetti ROS 2 usati per teleoperare e gestire il robot Unitree Go2 con input Haptic, Myo e Nav2.

## Struttura generale

- `go2_supervisor`: coordinamento delle modalità operative, multiplexing dei comandi e gestione delle richieste di navigazione ed esplorazione.
- `haptic_go2_teleop`: teleoperazione tramite Geomagic Touch, con produzione di comandi `/teleop_haptic/cmd_vel` e `/teleop_haptic/body_pose`.
- `myo_control`: lettura del Myo armband, elaborazione EMG e creazione di comandi di teleoperazione `/teleop_myo/cmd_vel` e `/teleop_myo/body_pose`.
- `go2_frontier_explorer`: esplorazione autonoma basata sulle frontier map e invio di goal a Nav2.
- `go2_nav_slam_sim`: stack di simulazione per SLAM e navigazione del robot Go2.
- `unitree_api`, `unitree_go`: pacchetti di messaggistica per l’interfaccia Go2 e le definizioni dei tipi di dato.

## Build e setup

1. Costruisci il workspace:

   ```bash
   cd /home/lorenzo/go2_haptic_teleop_ws
   colcon build --packages-select go2_supervisor haptic_go2_teleop myo_control go2_frontier_explorer go2_nav_slam_sim
   ```

2. Sorgente dell’ambiente ROS 2:

   ```bash
   source install/setup.bash
   ```

## Flusso di esecuzione consigliato

Il modo di utilizzo tipico è avviare prima il supervisore e poi il launch specifico del sistema desiderato.

### Avviare il supervisore

```bash
ros2 launch go2_supervisor supervisor.launch.py #(aggiungere use_real_go2:=true a necessità)
```

Questo avvia i nodi che coordinano le modalità operative, selezionano il topic `cmd_vel` corretto e inoltrano la posa del corpo.

### Avviare l’Haptic teleoperation

```bash
ros2 launch haptic_go2_teleop haptic_teleop.launch.py
```

### Avviare il controllo Myo

```bash
ros2 launch myo_control myo_teleop.launch.py
```

### Avviare l’esplorazione frontier (solo dopo aver avviato SLAM)

```bash
ros2 launch go2_frontier_explorer frontier_explorer.launch.py
```

### Avviare SLAM / Nav2 in simulazione

```bash
ros2 launch go2_nav_slam_sim go2_nav_slam_sim.launch.py
```

Per localizzazione con mappa statica:

```bash
ros2 launch go2_nav_slam_sim go2_nav_localization.launch.py
```

## Note importanti

- Non usare `go2_supervisor/launch/sim_full.launch.py` quando si usa il robot reale se è incompleto o non è parte del tuo flusso attuale.
- `action_manager.py` nel package `go2_supervisor` è attualmente un mock di backend: stampa messaggi e invia uno stop al robot, ma non esegue un controllo reale sul Go2.
- Il supervisore è stato impostato come un comportamento basato su segnali e modalità, e consente di far convivere teleoperazione Myo, teleoperazione Haptic, navigazione autonoma ed esplorazione.

## Pubblicazione e multiplexing dei topic

- Comando principale: `/cmd_vel`
- Posa del corpo: `/body_pose`
- Modalità: `/robot_mode`, `/cmd_source`, `/body_source`
- Teleoperazione Myo: `/teleop_myo/cmd_vel`, `/teleop_myo/body_pose`, `/teleop_myo/active`, `/teleop_myo/body_active`
- Teleoperazione Haptic: `/teleop_haptic/cmd_vel`, `/teleop_haptic/body_pose`, `/teleop_haptic/enabled` (se supportato dal nodo)
- Esplorazione: `/explore/request`
- Navigazione: `/nav/request`
- Azioni speciali: `/special_action/request`, `/special_action/status`

## Consigli d’uso

- Avvia il supervisore prima di lanciare i moduli teleoperativi o di navigazione.
- Personalizza i parametri dei singoli launch file solo se conosci il comportamento desiderato di Myo, Haptic o Nav2.
- Utilizza i pacchetti di messaggistica `unitree_api` e `unitree_go` come dipendenze quando sviluppi bridge o servizi reali per il robot.
