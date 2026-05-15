# myo_control

Pacchetto ROS 2 per leggere dati dal Myo armband e generare comandi di teleoperazione per Go2.

## Scopo

- Acquisisce IMU ed EMG dal Myo.
- Estrae eventi EMG per comandi di reset, toggle modalità e stop.
- Converte i dati IMU in `Twist` e `Pose` per la teleoperazione del robot.

## Nodi principali

- `myo_reader_node`
  - Legge il dongle Myo dal seriale.
  - Pubblica `/myo/imu` e `/myo/emg`.
  - Gestisce comandi haptici ricevuti su `/myo/haptic_cmd`.

- `myo_emg_events_node`
  - Analizza EMG e genera eventi su `/myo/event`.
  - Supporta eventi `reset`, `toggle_mode`, `stop`.
  - Gli eventi corrispondo rispettivamente a: chiusura della mano una volta, chiusura della mano due volte separate da un breve intervallo di tempo, chiusura prolungata della mano.

- `myo_to_cmdvel_node`
  - Converte i dati IMU in comandi di movimento e pose.
  - Pubblica `/teleop_myo/cmd_vel`, `/teleop_myo/body_pose`, `/teleop_myo/active`, `/teleop_myo/body_active`.
  - Sottoscrive `/myo/imu`, `/myo/reset_reference`, `/myo/event`.

## Launch

- `ros2 launch myo_control myo_teleop.launch.py`

## Funzionamento

- `myo_reader_node` connette il dongle Myo e inoltra i pacchetti IMU/EMG.
- `myo_emg_events_node` riconosce attivazioni brevi e lunghe per controllare la modalità di teleoperazione.
- `myo_to_cmdvel_node` genera comandi di locomozione e di corpo in base alla rotazione del braccio.

## Parametri rilevanti

- `serial_port`: porta del dongle Myo.
- `activation_on_threshold`, `activation_off_threshold`: soglie EMG.
- `short_min_duration`, `short_max_duration`: durata degli eventi brevi.
- `long_activation_duration`: durata per evento `stop`.
- `cmd_topic`, `body_topic`: topic di uscita dei comandi.

## Note

Il pacchetto include anche `myo_emg_monitor_node`, ma il launch standard attiva solo i nodi principali per la lettura e la conversione dei dati.
