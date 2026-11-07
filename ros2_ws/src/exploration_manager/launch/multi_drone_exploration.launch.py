"""Multi-UAV C2-Explorer stack on top of PX4 SITL + Gazebo.

Brings up, for each drone i in 1..drone_num:
  * /drone_i/... : the three PX4<->ROS bridges (odometry, lidar cloud,
    offboard position command), pointed at PX4 SITL instance i-1
  * /drone_i/... : exploration_node, its two LKH solver nodes and traj_server
  * a ros_gz_bridge instance pulling that vehicle's lidar cloud out of Gazebo

plus one global c2x_px4_bridge/swarm_relay that forwards every drone's
"<topic>_send" to all the other drones' "<topic>_recv" - the ROS2 replacement
for ROS1's UDP swarm_bridge.

Drone i maps to PX4 SITL instance i-1, i.e. ROS namespace /px4_<i-1> (bare
/fmu for i=1) and MAV_SYS_ID i, and to the Gazebo model "x500_mid360_<i-1>".
The PX4/Gazebo processes themselves are started by scripts/run_exploration.sh.

Exploration is triggered globally: a single publication on
/move_base_simple/goal starts all drones (the trigger subscription is the one
absolute topic in the stack).

DRONE VERO (reale:=true, ottobre 2026, scripts/run_reale_exploration.sh): lo stesso stack
per un drone solo, a tempo reale e senza nulla di Gazebo. FAST-LIO legge il driver del
Livox Mid-360 (/livox/lidar, /livox/imu) con scripts/collaudo/fast_lio_mid360.yaml, e la
nuvola per la mappa e' quella di FAST-LIO stesso (vedi il ramo reale piu' sotto).
parte:=stima|missione divide lo stack in due lanci: prima stima e ponte verso PX4, poi,
dopo i controlli a terra e il consenso dell'operatore, il resto (il ponte dei comandi arma
e decolla appena parte).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, TimerAction
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import tempfile

import yaml

# Distinct rviz colours per drone, following the reference implementation's
# palette (red / brown / green / blue / ...).
DRONE_COLORS = [
    (1.0, 0.0, 0.0),
    (0.55, 0.27, 0.007),
    (0.0, 0.5, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.5, 0.0),
    (0.0, 0.8, 0.8),
]


def _lidar_gz_topic(world: str, instance: int) -> str:
    """Gazebo's default topic for this vehicle's lidar point cloud.

    c2x_description's x500_mid360 deliberately leaves <topic> unset so that gz
    derives a name containing the spawned model name, which PX4 makes unique
    per instance ("x500_mid360_<instance>").
    """
    return (f'/world/{world}/model/x500_mid360_{instance}'
            f'/link/lidar_sensor_link/sensor/mid360/scan/points')


def _imu_gz_topic(world: str, instance: int) -> str:
    """Argomento Gazebo della IMU co-locata col lidar.

    Stessa logica del lidar: nessun <topic> esplicito nel modello, cosi' gz
    deriva un nome che contiene gia' quello del modello ed e' unico per
    velivolo. La IMU sta dentro lidar_sensor_link, quindi il percorso e'
    identico a quello della nuvola fino al nome del sensore.
    """
    return (f'/world/{world}/model/x500_mid360_{instance}'
            f'/link/lidar_sensor_link/sensor/lidar_imu/imu')


def _parse_spawn_poses(raw: str, drone_num: int):
    """Parse "x,y,z,yaw;x,y,z,yaw;..." into one (x, y, z, yaw) tuple per drone.

    These must be the same poses the vehicles were spawned at
    (PX4_GZ_MODEL_POSE). PX4 reports local position relative to each vehicle's
    own EKF origin, which sits at its spawn point, so without this every drone
    would believe it is at (0, 0, 0) and the shared world frame the exploration
    stack assumes would collapse. Missing entries default to the origin.
    """
    poses = []
    for chunk in (c for c in raw.split(';') if c.strip()):
        parts = [float(v) for v in chunk.split(',')]
        parts += [0.0] * (4 - len(parts))
        poses.append(tuple(parts[:4]))
    while len(poses) < drone_num:
        poses.append((0.0, 0.0, 0.0, 0.0))
    return poses


# NODI CRITICI: se uno di questi termina (crash, eccezione), si spegne l'intero
# stack invece di lasciarlo zoppo. Senza, morto exploration_node, traj_server
# continuava a ripetere l'ultimo punto e il drone restava in hovering per
# sempre; morto il ponte dei comandi, nessuno lo avrebbe fatto atterrare.
# Spegnendo tutto, PX4 perde il collegamento offboard e dopo COM_OF_LOSS_T
# (1 s) esegue COM_OBL_RC_ACT = 4, cioe' ATTERRA (airframe 22000).
def critico():
    return {'on_exit': Shutdown(reason='nodo critico terminato: stack spento, PX4 atterra')}


# Nodi che restano con C2X_SOLO_STIMA=1: ponti di lidar e IMU, adattatore, stimatore, ponte
# della stima verso PX4.
SOLO_STIMA = {'parameter_bridge', 'lidar_lio_adapter', 'fastlio_mapping', 'fastlivo_mapping',
              'parameter_blackboard', 'lio_to_px4_bridge'}
# parte:=stima tiene questi nodi, parte:=missione tutti gli altri, parte:=tutto (di serie)
# l'intero stack in un lancio solo.
PARTI = ('tutto', 'stima', 'missione')

# Campo verticale del Mid-360 VERO: da -7 a +52 gradi (il simulato e' +-50). Va al modello
# di visibilita' del pianificatore: con +-50 cercherebbe di "vedere" il pavimento vicino,
# che il sensore vero non vede mai, e quelle frontiere non si chiuderebbero.
MID360_SU_RAD = 0.9075712110370514     # 52 gradi
MID360_GIU_RAD = 0.12217304763960307   # 7 gradi


def _parse_vettore(raw: str, nome: str):
    parti = [float(v) for v in raw.split(',') if v.strip()]
    if len(parti) != 3:
        raise RuntimeError(f'{nome} vuole "x,y,z", ricevuto "{raw}"')
    return parti


def _accelerazione_ricerca(config, max_vel):
    """search.max_acc compatibile con max_vel, o None se quello del file va gia' bene.

    RICERCA CINODINAMICA A BASSA VELOCITA' (19 ottobre 2026, prova generale del drone vero a
    0.3 m/s). KinodynamicAstar espande ogni nodo con accelerazioni a passi di max_acc/2 per
    max_tau secondi, cioe' con salti di velocita' di 0.5*max_acc*max_tau (0.8 m/s con 2.0 m/s2
    e 0.8 s), e scarta i nodi oltre max_vel + vel_margin. A 0.3 m/s (0.55 col margine) da
    fermo restava solo l'accelerazione nulla: "Kino replan: Can't find path" con 3 nodi
    usati, il rientro a casa non si pianificava mai e allo scadere del suo tempo il drone
    atterrava dov'era (2.9 m dalla partenza). Si abbassa l'accelerazione della sola ricerca
    quanto basta; ottimizzatore e traiettoria tengono la loro. Con 0.6 m/s o piu' e 2.0 m/s2
    la condizione e' gia' rispettata e non cambia nulla.
    """
    try:
        dati = yaml.safe_load(open(config)) or {}
    except (OSError, yaml.YAMLError):
        return None
    par = next((v['ros__parameters'] for v in dati.values()
                if isinstance(v, dict) and isinstance(v.get('ros__parameters'), dict)), {})
    try:
        max_acc = float(par['search.max_acc'])
        max_tau = float(par['search.max_tau'])
        margine = float(par.get('search.vel_margin', 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    tetto = (max_vel + margine) / (0.5 * max_tau)
    return None if max_acc <= tetto else round(0.95 * tetto, 3)


def _filtra_parte(nodi, parte):
    if parte == 'tutto':
        return nodi
    tieni = parte == 'stima'
    return [n for n in nodi if (getattr(n, 'node_executable', '') in SOLO_STIMA) == tieni]


def _fast_lio_mid360_predefinito():
    # Con --symlink-install questo file e' un collegamento ai sorgenti:
    # <ws>/src/exploration_manager/launch/ -> <ws>/scripts/collaudo/fast_lio_mid360.yaml.
    # Lo script di avvio passa comunque il percorso (fast_lio_config).
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__)))))
    return os.path.join(ws, 'scripts', 'collaudo', 'fast_lio_mid360.yaml')


def launch_setup(context, *args, **kwargs):
    drone_num = int(LaunchConfiguration('drone_num').perform(context))
    # Sorgente della posa: "px4" (EKF di bordo, con GPS simulato) oppure "lio"
    # (FAST-LIO su lidar+IMU, che e' cio' che funziona senza GPS).
    odom_source = LaunchConfiguration('odom_source').perform(context)
    # 'livo' = FAST-LIVO2 (lidar + IMU + telecamera) al posto di FAST-LIO. Per tutto
    # il resto dello stack e' identico a 'lio': la stima arriva sullo stesso topic
    # {ns}/lio/odometry, con la stessa origine (la posa di partenza).
    stimatore = 'fast_livo2' if odom_source == 'livo' else 'fast_lio'
    if odom_source == 'livo':
        odom_source = 'lio'
    config_name = LaunchConfiguration('config_name').perform(context)
    world = LaunchConfiguration('world').perform(context)
    max_vel = float(LaunchConfiguration('max_vel').perform(context))
    max_flight_z = float(LaunchConfiguration('max_flight_z').perform(context))
    # PAVIMENTO DI VOLO, in coordinate mondo (0 = terreno sotto la partenza): lo
    # stesso valore per il ponte (min_flight_z, il riferimento non scende sotto) e
    # per i percorsi del pianificatore (astar.quota_minima, vedi Astar::search).
    # Prima lo conosceva solo il ponte, e il pianificatore mandava il drone sotto
    # la piattaforma del parco giochi a 0.6 m (18 ottobre 2026). PAVIMENTO_PERCORSI=0
    # nell'ambiente toglie quello dei percorsi (per i confronti).
    quota_minima = 1.0
    quota_minima_percorsi = (quota_minima if os.environ.get('PAVIMENTO_PERCORSI', '1') != '0'
                             else -1e9)
    initial_spin_rate = float(LaunchConfiguration('initial_spin_rate').perform(context))
    photo_completion = LaunchConfiguration('photo_completion').perform(context).lower() in ('1', 'true')
    photo_yaw = LaunchConfiguration('photo_yaw').perform(context).lower() in ('1', 'true')
    visit_completion = LaunchConfiguration('visit_completion').perform(context).lower() in ('1', 'true')
    spawn_poses = _parse_spawn_poses(
        LaunchConfiguration('spawn_poses').perform(context), drone_num)

    # Drone vero o simulazione, e quale parte dello stack (vedi la nota in testa).
    reale = LaunchConfiguration('reale').perform(context).lower() in ('1', 'true')
    parte = LaunchConfiguration('parte').perform(context)
    # C2X_SOLO_STIMA=1 (collaudo in simulazione, run_multi_exploration.sh SOLO_STIMA=1) e'
    # la stessa cosa di parte:=stima.
    if os.environ.get('C2X_SOLO_STIMA') == '1':
        parte = 'stima'
    if parte not in PARTI:
        raise RuntimeError(f'parte:={parte} non valida (una fra {", ".join(PARTI)})')
    if reale:
        # Sul drone vero: un velivolo solo (driver Livox e PX4 senza namespace), e senza
        # GPS al chiuso la posa viene solo da FAST-LIO. FAST-LIVO2 e' stato provato e
        # scartato (LEGGIMI) e sul drone vero non ha ancora la telecamera.
        if drone_num != 1:
            raise RuntimeError('reale:=true vuole drone_num:=1')
        if odom_source != 'lio' or stimatore != 'fast_lio':
            raise RuntimeError('reale:=true vuole odom_source:=lio (FAST-LIO)')
        fast_lio_reale = (LaunchConfiguration('fast_lio_config').perform(context)
                          or _fast_lio_mid360_predefinito())
        if not os.path.isfile(fast_lio_reale):
            raise RuntimeError(f'configurazione di FAST-LIO non trovata: {fast_lio_reale}')
        lidar_offset = _parse_vettore(LaunchConfiguration('lidar_offset').perform(context),
                                      'lidar_offset')

    share = get_package_share_directory('exploration_manager')
    config = os.path.join(share, 'config', config_name)
    # Cartella di LAVORO del solver LKH: qui l'esplorazione scrive a ogni ciclo i
    # file .par/.atsp e il solver ci lascia il .tour. Prima era la cartella
    # resource/ del pacchetto, che con --symlink-install e' un collegamento ai
    # SORGENTI: ogni run riscriveva file del codice, con dentro percorsi
    # assoluti della macchina. Sono file temporanei, il loro posto e' /tmp.
    acc_ricerca = _accelerazione_ricerca(config, max_vel)
    mtsp_dir = os.path.join(tempfile.gettempdir(), 'c2x_lkh')
    os.makedirs(mtsp_dir, exist_ok=True)

    # TUTTO LO STACK A TEMPO SIMULATO (sciame, ottobre 2026). Fino a qui solo FAST-LIO
    # usava il tempo di Gazebo e il resto l'orologio di parete: con un drone il Jetson
    # tiene la simulazione a RTF 0.85-0.9 e lo scarto si notava poco, con tre droni
    # scende sotto 0.4 e traiettorie, tetti di tempo e ripianificazioni scorrevano 2-3
    # volte piu' in fretta della fisica (il riferimento scappava avanti al drone, il
    # tetto di 1200 s di esplorazione ne valeva ~400 di volo). LEGGIMI lo segnava come
    # "da sistemare prima di riprendere il multi-drone". A tempo simulato lo stack vede
    # il mondo come lo vedrebbe in volo vero, qualunque sia il RTF: con tre droni su un
    # solo Jetson e' come dare a ogni drone il proprio calcolatore, come nella realta'.
    # I timer restano a parete (create_wall_timer): ticchettano piu' spesso per secondo
    # simulato, e i tetti di tempo di calcolo (A*, ottimizzatore) restano di CPU.
    # tempo_sim:=false torna al comportamento di prima. Sul drone vero non c'e' un
    # orologio simulato: tutto gira a tempo reale.
    tempo_sim = (not reale and
                 LaunchConfiguration('tempo_sim').perform(context).lower() in ('1', 'true'))
    nodes = [
        *([SetParameter(name='use_sim_time', value=True)] if tempo_sim else []),
        # OROLOGIO DELLA SIMULAZIONE verso ROS: serve a FAST-LIO (lidar e IMU sono
        # marcati col tempo di Gazebo) e, con tempo_sim, a tutto lo stack. Il ponte
        # stesso resta sull'orologio di parete: e' lui a pubblicare /clock.
        *([] if reale else [Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='log',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': False}],
        )]),
        Node(
            package='c2x_px4_bridge',
            executable='swarm_relay',
            name='swarm_relay',
            output='screen',
            # Inoltra soltanto: nessun orologio (vedi i solutori LKH).
            parameters=[{'drone_num': drone_num, 'ns_prefix': '/drone_', 'use_sim_time': False}],
        ),
    ]

    # Ritardo di avvio per drone: i nodi del drone N partono N*stagger secondi
    # dopo gli altri.
    #
    # In ROS2 i nodi si trovano fra loro per scoperta dinamica: si annunciano
    # sulla rete e devono accoppiarsi a due a due. Facendone partire una
    # trentina nello stesso istante l'annuncio diventa un'ondata, e sotto carico
    # qualche coppia non si forma: circa una run su tre partiva con un drone che
    # non riceveva MAI l'odometria, mentre gli altri due volavano - con le tre
    # istanze PX4 tutte sane e tutti i ponti avviati. Scaglionare distribuisce
    # gli annunci nel tempo invece di concentrarli.
    stagger = float(LaunchConfiguration('startup_stagger').perform(context))
    nodes = _filtra_parte(nodes, parte)

    for drone_id in range(1, drone_num + 1):
        ns = f'/drone_{drone_id}'
        instance = drone_id - 1
        drone_nodes = []
        # PX4 leaves instance 0 unnamespaced and prefixes the rest with
        # /px4_<instance> (ROMFS/px4fmu_common/init.d-posix/rcS).
        px4_ns = '' if instance == 0 else f'/px4_{instance}'
        color_r, color_g, color_b = DRONE_COLORS[(drone_id - 1) % len(DRONE_COLORS)]
        sx, sy, sz, syaw = spawn_poses[drone_id - 1]
        spawn_params = {'spawn_x': sx, 'spawn_y': sy, 'spawn_z': sz, 'spawn_yaw': syaw}

        common_overrides = {
            'exploration.drone_id': drone_id,
            'exploration.drone_num': drone_num,
            # Partenze di TUTTI i droni [x1, y1, x2, y2, ...]: servono alla spartizione a
            # strisce (vedi findGlobalTourOfGrid), che ogni drone calcola uguale.
            'exploration.partenze': [float(v) for p in spawn_poses for v in p[:2]],
            # SPARTIZIONE_STRISCE=0 nell'ambiente torna alla ripartizione di prima.
            'partitioning.strisce': os.environ.get('SPARTIZIONE_STRISCE', '1') != '0',
            'exploration.mtsp_dir': mtsp_dir,
            'exploration.tsp_dir': mtsp_dir,
            'color_r': color_r,
            'color_g': color_g,
            'color_b': color_b,
            # One PCD per drone: /sdf_map/save_map is now namespaced too, and a
            # shared path would have the drones overwrite each other's map.
            'map_ros.save_pcd_path': f'/tmp/c2x_explored_map_drone{drone_id}.pcd',
            # Un solo tetto di velocita' per tutti i moduli del pianificatore
            # (argomento max_vel): con valori diversi il piu' basso comanda e
            # alzarne uno solo non cambia nulla.
            'manager.max_vel': max_vel,
            'search.max_vel': max_vel,
            'optimization.max_vel': max_vel,
            # Fase fotografica dopo l'esplorazione (argomento photo_completion,
            # FASE_FOTO nello script di lancio). Spenta di serie dal 6 ottobre
            # 2026: con l'imbardata da foto l'esplorazione da sola fotografa gia'
            # la maggior parte della mappa 3D, e la fase costava ~5 minuti su
            # cubicle_office.
            'fsm.photo_completion': photo_completion,
            # Imbardata da foto in esplorazione (argomento photo_yaw,
            # IMBARDATA_FOTO nello script): la telecamera si gira verso le
            # superfici non ancora inquadrate. Costa rotazione: ~2400 gradi/min
            # contro ~900 senza (V29, cubicle_office).
            'fsm.photo_yaw': photo_yaw,
            # Fase di visita dopo l'esplorazione (argomento visit_completion,
            # VISITA nello script): il drone va nelle zone di spazio
            # percorribile in cui non e' mai passato (vedi copertura_visita.h).
            'fsm.visit_completion': visit_completion,
            # Lentezza della CPU rispetto al PC su cui sono tarati i tetti di tempo
            # di A* e dell'ottimizzazione (vedi Astar::init). PROVATO 2 sul Jetson
            # (16 ottobre 2026, cubicle_office senza GPS, 1.0 m/s, solo esplorazione):
            # PEGGIO, esplorazione 566 s contro 496 s con 1, pianificazione totale
            # 282 s contro 247 - le ricerche che falliscono costano il doppio e la
            # matrice dei costi (seriale) e' passata da 104 a 183 s. Di serie 1;
            # FATTORE_TEMPO_CPU nell'ambiente lo cambia.
            'prestazioni.fattore_tempo_cpu': float(os.environ.get('FATTORE_TEMPO_CPU', 1.0)),
            # Thread del grafo di connettivita' (UniformGrid::updateConnectivityGraph).
            # Con 3 e fattore 2 il grafo costava come con 1 e fattore 1 (220 s per
            # volo). Da solo, insieme all'A* piu' rapido (17 ottobre 2026, Jetson,
            # cubicle_office senza GPS a 1.0 m/s): grafo in media 0.20-0.24 s contro
            # 0.39 con 1 thread, esplorazione 274-328 s. Di serie 3; CG_THREADS lo cambia.
            'partitioning.cg_threads': int(os.environ.get('CG_THREADS', 3)),
            # Pavimento dei percorsi (A* a griglia e cinodinamico): vedi quota_minima.
            'astar.quota_minima': quota_minima_percorsi,
        }
        if acc_ricerca is not None:
            common_overrides['search.max_acc'] = acc_ricerca
        # Collegamento radio fra i droni (RAGGIO_RADIO / LINEA_VISTA nello script di run):
        # vuoto = quello del file di configurazione.
        raggio_radio = LaunchConfiguration('raggio_radio').perform(context)
        if raggio_radio:
            common_overrides['communication.connection_threshold'] = float(raggio_radio)
        linea_vista = LaunchConfiguration('linea_vista').perform(context)
        if linea_vista:
            common_overrides['communication.line_of_sight'] = linea_vista.lower() in ('1', 'true')
        if reale:
            # Modello di visibilita' del pianificatore = campo del Mid-360 vero montato dritto
            # (vedi MID360_SU_RAD). Vince sul file di configurazione.
            common_overrides['perception_utils.lidar_top_angle'] = MID360_SU_RAD
            common_overrides['perception_utils.lidar_bottom_angle'] = MID360_GIU_RAD

        if odom_source == 'lio':
            # PROVATO E SCARTATO: pianificatore rallentato a 0.25 m/s in volo
            # cieco (con MPC_XY_VEL_MAX a 0.5). Su 4 run e' risultato PEGGIORE:
            # errore di posizione da 0.470 a 0.753 m, latenza da 0.309 a 0.434 s.
            # Il motivo e' che non ha rallentato il drone: velocita' vera 1.1-1.5
            # m/s con un piano da 0.25, perche' la velocita' la detta l'errore di
            # stima inseguito dal controllore, non il piano. Ha solo cambiato la
            # traiettoria, peggiorandola.
            # FAST-LIO gira a TEMPO DI SIMULAZIONE, perche' lidar e IMU sono
            # entrambi marcati da Gazebo. Il rientro all'orologio di parete
            # avviene in odometry_bridge, in un punto solo.
            if reale:
                # DRONE VERO: FAST-LIO sul driver del Mid-360 (CustomMsg su /livox/lidar,
                # IMU interna su /livox/imu: lo dice fast_lio_mid360.yaml), a tempo reale.
                # La nuvola per la mappa e per il filtro di sicurezza del ponte dei comandi
                # e' la scansione di FAST-LIO in terna corpo (/cloud_registered_body): il
                # driver pubblica CustomMsg, che cloud_transform_bridge non legge, e quella
                # nuvola e' gia' raddrizzata dal moto e marcata con lo STESSO istante della
                # stima (lidar_end_time), cioe' la coppia nuvola-posa che
                # cloud_transform_bridge cerca. E' sottocampionata come l'ingresso di
                # FAST-LIO (point_filter_num 3, punti oltre blind 0.5 m).
                drone_nodes.append(Node(
                    package='fast_lio',
                    executable='fastlio_mapping',
                    **critico(),
                    name='fast_lio',
                    namespace=ns,
                    output='log',
                    parameters=[
                        fast_lio_reale,
                        {'use_sim_time': False,
                         'publish.scan_publish_en': True,
                         'publish.scan_bodyframe_pub_en': True},
                    ],
                    remappings=[('/Odometry', f'{ns}/lio/odometry'),
                                ('/cloud_registered', f'{ns}/lio/cloud'),
                                ('/cloud_registered_body', f'{ns}/lidar/points'),
                                ('/path', f'{ns}/lio/path'),
                                ('/lio_salute', f'{ns}/lio/salute')],
                ))
            elif stimatore == 'fast_lio':
                drone_nodes.append(Node(
                    package='fast_lio',
                    executable='fastlio_mapping',
                    **critico(),
                    name='fast_lio',
                    namespace=ns,
                    output='log',
                    parameters=[
                        os.path.join(get_package_share_directory('fast_lio'),
                                     'config', 'c2x_sim.yaml'),
                        {'use_sim_time': True,
                         'common.lid_topic': f'{ns}/lidar/points_lio',
                         'common.imu_topic': f'{ns}/lidar/imu'},
                    ],
                    remappings=[('/Odometry', f'{ns}/lio/odometry'),
                                ('/cloud_registered', f'{ns}/lio/cloud'),
                                ('/path', f'{ns}/lio/path'),
                                ('/lio_salute', f'{ns}/lio/salute')],
                ))
            else:
                # FAST-LIVO2 (ottobre 2026, prova): stessa nuvola e stessa IMU di
                # FAST-LIO, piu' la telecamera frontale che registra_mappa.py porta
                # in ROS su {ns}/camera/image (serve FOTO=1). I parametri della
                # telecamera li legge da un nodo a parte, parameter_blackboard, nello
                # stesso namespace (vikit_ros, params_helper.h).
                cfg_livo = os.path.join(get_package_share_directory('fast_livo2'), 'config')
                camera_livo = os.path.join(cfg_livo, 'camera_c2x_sim.yaml')
                drone_nodes.append(Node(
                    package='demo_nodes_cpp',
                    executable='parameter_blackboard',
                    name='parameter_blackboard',
                    namespace=ns,
                    output='log',
                    parameters=[camera_livo],
                ))
                drone_nodes.append(Node(
                    package='fast_livo2',
                    executable='fastlivo_mapping',
                    **critico(),
                    name='fast_livo2',
                    namespace=ns,
                    output='log',
                    parameters=[
                        os.path.join(cfg_livo, 'c2x_sim.yaml'), camera_livo,
                        {'use_sim_time': True,
                         'common.lid_topic': f'{ns}/lidar/points_lio',
                         'common.imu_topic': f'{ns}/lidar/imu',
                         'common.img_topic': f'{ns}/camera/image'},
                    ],
                    remappings=[('/aft_mapped_to_init', f'{ns}/lio/odometry'),
                                ('/cloud_registered', f'{ns}/lio/cloud'),
                                ('/path', f'{ns}/lio/path'),
                                ('/lio_salute', f'{ns}/lio/salute')],
                ))
            # La stessa stima va anche DENTRO l'autopilota come visione esterna:
            # e' cio' che permette a EKF2 di funzionare con EKF2_GPS_CTRL a zero.
            # Si passa l'odometria GREZZA di FAST-LIO, la cui origine coincide
            # gia' con l'origine locale di EKF2 (entrambe la posa di partenza).
            drone_nodes.append(Node(
                package='c2x_px4_bridge',
                executable='lio_to_px4_bridge',
                **critico(),
                name='lio_to_px4_bridge',
                namespace=ns,
                output='log',
                # Salute della stima (punti agganciati da FAST-LIO, vedi il ponte):
                # SALUTE_MIN_AGGANCIATI=0 disattiva l'atterraggio per stima degradata.
                parameters=[{'px4_ns': px4_ns, 'lio_topic': f'{ns}/lio/odometry',
                             'salute_topic': f'{ns}/lio/salute',
                             'salute_min_agganciati':
                                 float(os.environ.get('SALUTE_MIN_AGGANCIATI', 500))}],
            ))

        # Sensori simulati: nuvola e IMU del lidar da Gazebo, piu' l'adattatore che prepara
        # la nuvola per FAST-LIO. Sul drone vero ci sono il driver Livox e FAST-LIO (sopra).
        if not reale:
            drone_nodes += [
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'lidar_bridge_{drone_id}',
                    output='log',
                    arguments=[
                        _lidar_gz_topic(world, instance) +
                        '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    ],
                    remappings=[(_lidar_gz_topic(world, instance), f'{ns}/lidar/points')],
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'imu_bridge_{drone_id}',
                    output='log',
                    arguments=[
                        _imu_gz_topic(world, instance) + '@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    ],
                    remappings=[(_imu_gz_topic(world, instance), f'{ns}/lidar/imu')],
                ),
                Node(
                    package='c2x_px4_bridge',
                    executable='lidar_lio_adapter',
                    name='lidar_lio_adapter',
                    namespace=ns,
                    output='log',
                    # Copia la nuvola con i suoi istanti: nessun orologio. drone_id/drone_num:
                    # toglie i compagni dalla nuvola di FAST-LIO (vedi l'adattatore).
                    parameters=[{'use_sim_time': False, 'drone_id': drone_id,
                                 'drone_num': drone_num}],
                ),
            ]
        drone_nodes += [
            Node(
                package='c2x_px4_bridge',
                executable='odometry_bridge',
                **critico(),
                name='odometry_bridge',
                namespace=ns,
                output='log',
                parameters=[dict(spawn_params, px4_ns=px4_ns,
                                 odom_source=odom_source,
                                 lio_topic=f'{ns}/lio/odometry')],
            ),
            Node(
                package='c2x_px4_bridge',
                executable='cloud_transform_bridge',
                name='cloud_transform_bridge',
                namespace=ns,
                output='log',
                # Senza GPS ogni nuvola viene posata con la posa di FAST-LIO della
                # STESSA scansione (vedi cloud_transform_bridge.cpp); col GPS la
                # posa di PX4 viene interpolata all'istante della scansione.
                # Sul drone vero la nuvola e' in terna corpo di FAST-LIO, cioe' dell'IMU del
                # Mid-360: lidar_offset e' dove sta rispetto al punto di riferimento del drone
                # (quello a spawn_z da terra), in x avanti, y sinistra, z su. In simulazione
                # resta quello del modello x500_mid360 (0, 0, 0.25), di serie nel ponte.
                # drone_id/drone_num: toglie dalla nuvola i punti sui compagni (vedi
                # peer_filter_radius nel ponte), che altrimenti finiscono in mappa.
                parameters=[dict(spawn_params, drone_id=drone_id, drone_num=drone_num,
                                 lio_topic=(f'{ns}/lio/odometry' if odom_source == 'lio' else ''),
                                 **({'lidar_offset_body': lidar_offset} if reale else {}))],
            ),
            Node(
                package='c2x_px4_bridge',
                executable='position_command_bridge',
                **critico(),
                name='position_command_bridge',
                namespace=ns,
                output='log',
                # MAV_SYS_ID of SITL instance i is i+1, so drone_id doubles as
                # the target system id.
                # drone_id also feeds the inter-drone separation layer: it needs
                # to know which DroneState is its own, and which side yields.
                #
                # enable_peer_hold era spento perche' fermare il velivolo non
                # fermava traj_server, che continuava a valutare la B-spline sul
                # tempo di parete: ogni fermata lasciava il drone indietro
                # rispetto al proprio piano, e un drone che trascina invalida
                # proprio la separazione fra traiettorie che deve tenerli
                # distanti. Quel presupposto non vale piu': traj_server ora
                # riscala il tempo della traiettoria sull'errore di inseguimento
                # (traj_server.cpp, track_err_free/track_err_stop) e lo ferma del
                # tutto a 1 m di scarto, quindi il piano aspetta il velivolo
                # invece di scappargli via.
                # Serve perche' nei corridoi dell'ufficio il solo termine di
                # sciame dell'ottimizzatore - un costo morbido sulle traiettorie
                # pianificate - non basta: i droni si sono scontrati fra loro a
                # 0.74 m di distanza, con i ribaltamenti a coppie separate da
                # 0.32 s.
                # enable_lidar_hold riacceso per la stessa ragione: il motivo per
                # cui era spento (fermata -> piano che scappa) e' risolto dal
                # riscalamento temporale. Serve perche' in esplorazione si vola
                # verso il confine del noto, e la geometria non ancora mappata il
                # pianificatore non puo' evitarla: misurato sulla verita' del
                # simulatore, il margine mediano e' 1.1 m ma in alcuni voli
                # scende a 0.10-0.20 m, cioe' dentro l'ingombro delle eliche,
                # per lo 0.9-1.6% del tempo di volo. Sono quelle escursioni brevi
                # a distruggere i droni.
                # Le soglie 0.55/0.75 erano inservibili: la nuvola e' filtrata a
                # 0.5 m dal sensore, quindi la finestra di rilevamento era di 5 cm.
                #
                # ATTENZIONE: i valori qui sotto VINCONO sui default del sorgente
                # C++ (position_command_bridge.cpp dichiara 1.80/2.10). Quei
                # default non sono mai stati in volo: li avevo alzati per far
                # tornare il conto del tempo di collisione - il criterio chiedeva
                # 1.17 m di spazio di arresto e ne concedeva 0.80 - e per una
                # decina di campagne ho creduto fossero attivi. Il guadagno vero
                # sulle fermate anticipate viene dal filtro a banda verticale
                # (proximity_band_z), che invece non e' scavalcato qui.
                # Vanno provate a 1.80/2.10 da sole, non insieme ad altro.
                #
                # FILTRO DI SICUREZZA CONTINUO al posto della fermata su lidar
                # (enable_lidar_hold): la fermata congelava e rilasciava il
                # riferimento a soglie di distanza, e vicino ai muri si
                # accendeva e spegneva piu' volte al secondo - causa diretta
                # dell'urto nella run BASE_GPS 2 (vedi position_command_bridge,
                # enable_safety_filter). Il filtro limita solo la componente
                # del moto diretta verso l'ostacolo, senza mai fermare.
                # safety_dist 0.5 m resta sotto frontier.min_candidate_clearance
                # (0.6 m): i punti di vista restano raggiungibili.
                # max_flight_z: tetto di quota del riferimento. Senza, dopo una
                # divergenza della stima (BASE_LIO 3) il riferimento e' salito a
                # 5.8 m con il soffitto a 2.3.
                parameters=[
                    dict(spawn_params, px4_ns=px4_ns, target_system=drone_id, drone_id=drone_id,
                         enable_peer_hold=True, enable_lidar_hold=False,
                         enable_safety_filter=True, safety_dist=0.5, safety_slow_dist=0.9,
                         # Limite di frenata sul VELIVOLO (vedi enable_brake_governor
                         # nel ponte): con 1.5 m/s2 e 0.3 s di ritardo ci si deve
                         # poter fermare prima che un ostacolo sul percorso arrivi
                         # a 0.45 m dal centro (eliche a 0.35).
                         enable_brake_governor=True,
                         # PROVATO E SCARTATO: enable_proximity_speed (riferimento e
                         # anticipo a 0.5 m/s a 0.6 m dagli ostacoli, fino a 1.2 a
                         # 0.9 m). 5 run: margine minimo vero 0.35-0.55 m (mediana
                         # 0.40) e missione 284-364 s (mediana 333), contro 0.40-0.69
                         # (0.54) e 281-311 s (304) senza. Rallentare il riferimento
                         # vicino ai muri allarga il distacco dalla traiettoria e
                         # moltiplica le ripianificazioni proprio li'.
                         obstacle_stop_dist=0.80, obstacle_resume_dist=1.05,
                         # Corsie dell'incrocio fra droni (vedi cross_low_z nel ponte): 0.70 e
                         # 1.70 invece di 0.50 e 1.50, sempre un metro fra loro. Con la corsia
                         # bassa a 0.50 un drone dello sciame e' sceso a 0.29 m dal pavimento
                         # (office_small, ottobre 2026), inseguendola con lo scarto di PX4.
                         cross_low_z=0.70, cross_high_z=1.70,
                         max_flight_z=max_flight_z,
                         # Pavimento del riferimento (vedi quota_minima) e quota sotto
                         # cui nemmeno filtro di sicurezza e limite di frenata lo
                         # possono spingere (vedi min_flight_z_hard nel ponte).
                         min_flight_z=quota_minima, min_flight_z_hard=quota_minima - 0.2,
                         # Distanza massima del riferimento dal velivolo (vedi
                         # max_setpoint_jump nel ponte): 0.65 invece di 1.0 (17
                         # ottobre 2026, cubicle_office senza GPS, 5 run contro 8):
                         # scarto drone-riferimento al 99 percentile 0.59 m invece di
                         # 0.70, margine minimo vero 0.41-0.64 m invece di 0.35-0.69,
                         # missione invariata (media 306 s). Il limitatore interviene
                         # solo sugli episodi (0.40 con la rampa a 0.6 era scartato
                         # perche' restava sempre attivo). SALTO_RIFERIMENTO lo cambia.
                         max_setpoint_jump=float(os.environ.get('SALTO_RIFERIMENTO', 0.65)),
                         # Giro di 360 gradi a quota di decollo prima del via
                         # (rad/s, 0 = nessun giro): vedi position_command_bridge.
                         initial_spin_rate=initial_spin_rate,
                         # Senza GPS l'odometria arriva da FAST-LIO a 10 Hz, e
                         # 0.5 s (il default, tarato sui 100 Hz di PX4) sono
                         # solo 5 messaggi: nelle run BASE_LIO il riferimento
                         # veniva congelato fino a 105 volte per buchi di
                         # 0.50-0.52 s, cioe' normale variabilita' della cadenza.
                         odom_timeout=(1.0 if odom_source == 'lio' else 0.5))
                ],
            ),
            Node(
                package='exploration_manager',
                executable='exploration_node',
                **critico(),
                name='exploration_node',
                namespace=ns,
                output='screen',
                # Pronto al via solo a decollo e giro iniziale conclusi (topic
                # volo_pronto del ponte): prima il via poteva arrivare con il
                # drone ancora a terra o in salita.
                parameters=[config, common_overrides, {'fsm.wait_flight_ready': True}],
            ),
            Node(
                package='lkh_mtsp_solver',
                executable='mtsp_node',
                name=f'tsp_solver_{drone_id}',
                namespace=ns,
                output='log',
                # Solutore puro, non legge l'orologio: niente tempo simulato (ogni nodo con
                # use_sim_time riceve /clock a ogni passo di Gazebo).
                parameters=[{
                    'exploration.drone_id': drone_id,
                    'exploration.mtsp_dir': mtsp_dir,
                    'exploration.problem_id': 1,
                    'use_sim_time': False,
                }],
            ),
            Node(
                package='lkh_mtsp_solver',
                executable='mtsp_node',
                name=f'acvrp_solver_{drone_id}',
                namespace=ns,
                output='log',
                parameters=[{
                    'exploration.drone_id': drone_id,
                    'exploration.mtsp_dir': mtsp_dir,
                    'exploration.problem_id': 2,
                    'use_sim_time': False,
                }],
            ),
            Node(
                package='plan_manage',
                executable='traj_server',
                **critico(),
                name='traj_server',
                namespace=ns,
                output='log',
                # PROVATO E SCARTATO (campagna V18): passare la configurazione per
                # allineare fsm.replan_time a quello dell'FSM. Con anticipi lunghi
                # (0.45 s office_small, 2.0 s cubicle_office) il drone oscillava di
                # piu' (29 inversioni accelera/frena al minuto contro 7, scarto di
                # velocita' 0.45 contro 0.15 m/s): traj_server rallenta il proprio
                # orologio quando il drone resta indietro, l'FSM prevede lo stato di
                # partenza sull'orologio vero, e con un anticipo lungo il piano nuovo
                # parte piu' avanti del riferimento reale.
                parameters=[{
                    'traj_server.max_vel': max_vel,
                    'traj_server.drone_id': drone_id,
                    'color_r': color_r,
                    'color_g': color_g,
                    'color_b': color_b,
                }],
            ),
        ]

        # parte:=stima (o C2X_SOLO_STIMA=1, scripts/collaudo, ottobre 2026): solo sensori,
        # stimatore e ponte della stima verso PX4, come sul drone vero durante il collaudo.
        # Niente pianificatore ne' ponte dei comandi: a comandare PX4 e' solo
        # scripts/collaudo/collaudo.py. parte:=missione: tutto il resto, da avviare quando la
        # stima gira gia' (scripts/run_reale_exploration.sh).
        drone_nodes = _filtra_parte(drone_nodes, parte)

        # Il drone 1 parte subito, gli altri a scaglioni. TimerAction ritarda
        # l'avvio dell'intero gruppo di nodi di quel drone, non solo di uno.
        delay = stagger * (drone_id - 1)
        if delay > 0.0:
            nodes.append(TimerAction(period=delay, actions=drone_nodes))
        else:
            nodes += drone_nodes

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_num', default_value='3',
            description='Number of UAVs; drone i uses PX4 SITL instance i-1.'),
        DeclareLaunchArgument(
            'odom_source', default_value='px4',
            description="Sorgente della posa: 'px4' usa l'EKF di bordo, che in "
                        "simulazione riceve un GPS quasi perfetto; 'lio' usa FAST-LIO "
                        "su lidar+IMU, che e' cio' che funziona in un edificio senza GPS. "
                        "Con 'lio' viene avviato un nodo FAST-LIO per drone; con 'livo' "
                        "FAST-LIVO2 (lidar + IMU + telecamera) al suo posto."),
        DeclareLaunchArgument(
            'config_name', default_value='multi_drone_empty_room.yaml',
            description='Config file in exploration_manager/config/, one per scenario.'),
        DeclareLaunchArgument(
            'world', default_value='empty_room',
            description='Gazebo world name; used to build the per-vehicle lidar topic.'),
        DeclareLaunchArgument(
            'startup_stagger', default_value='3.0',
            description='Secondi di ritardo fra l\'avvio dei nodi di un drone e '
                        'quelli del successivo. 0 li avvia tutti insieme, che e\' '
                        'la condizione in cui circa una run su tre partiva con un '
                        'drone che non riceveva mai l\'odometria.'),
        DeclareLaunchArgument(
            'max_vel', default_value='0.6',
            description='traj_server velocity cap; keep in line with manager.max_vel. '
                        'Alzato a 1.0 con manager.max_vel: finche\' e\' rimasto a 0.6 il '
                        'velivolo volava a 0.63-0.73 m/s qualunque cosa dicesse la '
                        'configurazione del pianificatore, perche\' il taglio vero avviene qui.'),
        DeclareLaunchArgument(
            'initial_spin_rate', default_value='0.5',
            description='Velocita\' (rad/s) del giro di 360 gradi fatto a quota di decollo prima '
                        'di partire con l\'esplorazione; 0 = nessun giro. 0.5 = un giro in 12.6 s.'),
        DeclareLaunchArgument(
            'photo_completion', default_value='false',
            description='true = finite le frontiere, il drone va a fotografare le zone della mappa '
                        '3D non ancora inquadrate prima di rientrare (fino a fsm.photo_completion_time).'),
        DeclareLaunchArgument(
            'visit_completion', default_value='true',
            description='true = finite le frontiere, il drone visita lo spazio percorribile in cui '
                        'non e\' mai passato prima di rientrare; false = solo esplorazione.'),
        DeclareLaunchArgument(
            'photo_yaw', default_value='false',
            description='true = durante l\'esplorazione la telecamera si gira verso le superfici '
                        'della mappa 3D non ancora fotografate; false = guarda nella direzione di marcia.'),
        DeclareLaunchArgument(
            'max_flight_z', default_value='2.0',
            description='Quota massima (m, mondo) che il riferimento puo\' chiedere. Deve stare '
                        'sotto il soffitto della scena e sopra la quota di rientro (1.8 m).'),
        DeclareLaunchArgument(
            'spawn_poses', default_value='0,0,0,0;0,1.5,0,0;0,-1.5,0,0',
            description='"x,y,z,yaw" per drone, semicolon-separated. Must match the '
                        'PX4_GZ_MODEL_POSE each vehicle was spawned with - PX4 reports '
                        'position relative to its own spawn point.'),
        DeclareLaunchArgument(
            'raggio_radio', default_value='',
            description='Portata (m) del collegamento fra droni: vince su '
                        'communication.connection_threshold del file. Vuoto = quello del file.'),
        DeclareLaunchArgument(
            'linea_vista', default_value='',
            description='true/false: il collegamento fra droni richiede la linea di vista libera '
                        '(communication.line_of_sight). Vuoto = quello del file.'),
        DeclareLaunchArgument(
            'tempo_sim', default_value='true',
            description='true = tutti i nodi a tempo simulato (use_sim_time), indispensabile con '
                        'piu\' droni sul Jetson (RTF < 0.4); false = solo FAST-LIO, il resto '
                        'sull\'orologio di parete (comportamento fino a ottobre 2026). Ignorato '
                        'con reale:=true.'),
        DeclareLaunchArgument(
            'reale', default_value='false',
            description='true = drone VERO (un drone solo, odom_source:=lio): FAST-LIO sul driver '
                        'Livox Mid-360, tempo reale, nessun ponte Gazebo. Lo avvia '
                        'scripts/run_reale_exploration.sh.'),
        DeclareLaunchArgument(
            'parte', default_value='tutto',
            description="'tutto' = l'intero stack; 'stima' = sensori, stimatore e ponte della stima "
                        "verso PX4; 'missione' = tutto il resto (ponte dei comandi, che arma e "
                        "decolla appena parte, e pianificatore), con la stima gia' avviata."),
        DeclareLaunchArgument(
            'fast_lio_config', default_value='',
            description='Solo reale: configurazione di FAST-LIO per il Mid-360 (vuoto = '
                        'scripts/collaudo/fast_lio_mid360.yaml dello spazio di lavoro).'),
        DeclareLaunchArgument(
            'lidar_offset', default_value='0,0,0.10',
            description='Solo reale: posizione "x,y,z" (m; x avanti, y sinistra, z su) del Mid-360 '
                        'rispetto al punto di riferimento del drone, quello che spawn_poses mette '
                        'a quota z da terra. Va misurata sul drone.'),
        OpaqueFunction(function=launch_setup),
    ])
