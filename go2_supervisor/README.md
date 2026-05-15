# go2_supervisor

Pacchetto ROS 2 che coordina modalità, comandi e sorgenti di teleoperazione, navigazione ed esplorazione per il robot Go2.

## Scopo

Il supervisore gestisce lo stato del robot e instrada i messaggi di comando e pose dai sistemi di teleoperazione Myo e Haptic verso il robot.

## Nodi principali

- `mode_manager`
  - Decide la modalità operativa del robot.
  - Pubblica `/robot_mode`, `/cmd_source`, `/body_source`.
  - Riceve richieste da `/mode_request`, `/special_action/request`, `/emergency_stop`, `/teleop_haptic/active`, `/teleop_haptic/body_active`, `/teleop_myo/active`, `/teleop_myo/body_active`, `/nav/request`, `/explore/request`, `/special_action/status`.

- `cmd_vel_mux`
  - Riceve `Twist` da `/teleop_myo/cmd_vel`, `/teleop_haptic/cmd_vel`, `/nav/cmd_vel`, `/explore/cmd_vel`.
  - In base al valore di `/cmd_source`, pubblica il comando selezionato su `/cmd_vel`.

- `body_pose_mux`
  - Riceve `Pose` da `/teleop_myo/body_pose` e `/teleop_haptic/body_pose`.
  - In base a `/body_source`, pubblica la posa selezionata su `/body_pose`.

- `nav_request_manager`
  - Monitora lo stato dell'azione di navigazione su `/navigate_to_pose/_action/status`.
  - Pubblica `/nav/request` come segnale di attivazione/disattivazione della navigazione.

- `action_manager`
  - Mock backend per comandi speciali `sit`, `stand`, `wave`.
  - Pubblica `/special_action/status` e invia stop su `/cmd_vel` durante l'esecuzione.
  - Attualmente non controlla il Go2 reale, ma può essere esteso in futuro per usare opzioni speciali del robot.

## Launch

- `ros2 launch go2_supervisor supervisor.launch.py`

Questo file avvia il comportamento supervisor completo, da usare insieme ad un altro launch (controllo con myo o aptico, navigazione e esplorazione). Al momento `sim_full.launch.py` che dovrebbe avviare tutti nodi è incompleto e non in uso quando si tratta di usare il robot reale. Quando invece si usa la simualazione può essere usato.

## Note sul comportamento

- Il package è stato impostato per funzionare come un comportamento a grafo di comportamento adeguato.
- `mode_manager` sceglie la modalità attiva in base allo stato dei teleop, alla navigazione e agli allarmi di emergenza.
- `cmd_vel_mux` e `body_pose_mux` isolano le sorgenti di comando e pose per evitare conflitti tra i sistemi di teleoperazione e la navigazione.
- se si usa il robot reale, e quindi si lancia `supervisor.launch.py` bisogna aggiungere al comando indicato anche `use_real_go2:=True`
