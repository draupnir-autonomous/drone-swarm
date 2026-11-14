#!/usr/bin/env python3
"""Collaudo a fasi del drone vero (C2-Explorer, ottobre 2026), un passo alla volta.

Parla con PX4 via uXRCE-DDS (gli stessi topic /fmu/in e /fmu/out dei ponti del progetto) e
guida l'operatore dalle verifiche a terra fino a un volo automatico semplice. Ogni fase dice
cosa fare, misura, e dice SUPERATA o NON SUPERATA con i numeri; non si passa alla fase
successiva se la precedente non e' superata (si puo' forzare, ma resta scritto nel registro).

  0  Collegamento con PX4        stato, errori, batteria, comando manuale accettato
  1  Sensori                     frequenza del lidar, dell'IMU, della telecamera
  2  FAST-LIO a drone fermo      frequenza della stima, deriva, punti agganciati
  3  Assi e stima dentro PX4     drone mosso a mano: FAST-LIO e PX4 d'accordo su assetto e posizione
  4  Motori SENZA eliche         prova dei 4 motori, armamento, arresto d'emergenza
  5  Hover trattenuto            decollo a 0.5 m con il drone legato, 30 s fermo, atterraggio
  6  Volo manuale                spostamenti piccoli da tastiera o joystick in un recinto di 2 m
  7  Quadrato automatico         offboard: quadrato di 1 m a 0.3 m/s, poi atterraggio
  8  Missione completa           esplorazione autonoma di uno spazio libero di 5 x 5 m (drone al
                                 centro) con run_reale_exploration.sh, poi mappa 3D colorata con
                                 le foto della webcam (fino al 7 novembre 2026 solo istruzioni)
  9  FAST-LIO all'aperto         a piedi, motori spenti: giro del perimetro dell'area della fase 10
                                 col drone in mano; chiusura del giro, lato misurato, frequenza,
                                 punti agganciati -> soglia di atterraggio per la fase 10
 10  Missione all'aperto         area di 30 x 30 m a 1 m/s con recinto virtuale (frontiere che si
                                 chiudono anche senza pareti), poi mappa 3D (novembre 2026)

SENZA RADIOCOMANDO questo programma e' il radiocomando: manda a PX4 le levette virtuali su
/fmu/in/manual_control_input (PX4: COM_RC_IN_MODE = 1). Se il programma si ferma o il
collegamento cade, PX4 non riceve piu' levette e dopo COM_RC_LOSS_T applica NAV_RCL_ACT
(atterraggio, vedi collaudo_px4.params). In volo, sempre:
  SPAZIO = EMERGENZA: motori spenti subito (il drone CADE)      l = atterra      h = fermo
  frecce = avanti/indietro/sinistra/destra   w/s = su/giu'   a/d = ruota   t = decolla
t = decollo in offboard (quota che sale a 0.4 m/s), poi POSCTL fermo in quota.
Controlli automatici in volo (atterraggio da soli): stima di FAST-LIO ferma da 0.5 s, punti
agganciati troppo pochi, posizione di PX4 non valida, fuori dal recinto (raggio o quota),
batteria sotto il 15%. Se la sessione SSH cade (SIGHUP) il programma comanda l'atterraggio.

Uso (sul Jetson, con lo spazio di lavoro caricato, dopo avvia_collaudo.sh):
    python3 collaudo.py               drone vero
    python3 collaudo.py --sim         simulazione (avvia_collaudo.sh sim)
    python3 collaudo.py --fase 5      parte dalla fase 5 (le precedenti vanno superate prima)
    python3 collaudo.py --area 6      fase 8 in uno spazio di 6 x 6 m (--box per un decollo non
                                      al centro: recinto xmin,ymin,xmax,ymax dal decollo)
Registro di ogni sessione in collaudo_registri/<data_ora>_<sim|reale>/ accanto a questo file;
quali fasi sono superate in collaudo_registri/esiti_sim.json ed esiti_reale.json (separati);
le misure del drone chieste dalla fase 8 in collaudo_registri/misure_drone.json.
"""
import argparse
import curses
import json
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rosidl_runtime_py.utilities import get_message

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleAttitude, BatteryStatus,
                          FailsafeFlags, EstimatorStatusFlags, VehicleLandDetected,
                          VehicleCommand, VehicleCommandAck, ManualControlSetpoint,
                          OffboardControlMode, TrajectorySetpoint)

QUI = os.path.dirname(os.path.abspath(__file__))

# Modalita' PX4 (custom main mode di MAV_CMD_DO_SET_MODE) e stati di navigazione.
MODO_POSCTL, MODO_OFFBOARD = 3, 6
NOMI_NAV = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'MISSION', 4: 'LOITER', 5: 'RTL',
            12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'TAKEOFF',
            18: 'LAND'}
FORZA_DISARMO = 21196.0   # param2 di COMPONENT_ARM_DISARM: disarmo anche in volo


def eulero_ned(q):
    """(w, x, y, z) -> rollio, beccheggio, imbardata in gradi (convenzione PX4/NED-FRD)."""
    w, x, y, z = q
    r = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    p = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    yw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return r, p, yw


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def lio_in_ned(o):
    """Posa di FAST-LIO convertita come fa lio_to_px4_bridge.cpp: posizione ENU -> NED,
    assetto q_ned = Qmondo * q_enu * Qcorpo. Cosi' il confronto con PX4 usa la stessa
    trasformazione che PX4 riceve davvero."""
    p = o.pose.pose.position
    q = o.pose.pose.orientation
    s = 0.70710678118654752
    qn = qmul(qmul((0.0, s, s, 0.0), (q.w, q.x, q.y, q.z)), (0.0, 1.0, 0.0, 0.0))
    return (p.y, p.x, -p.z), qn


def diff_angolo(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


class Frequenza:
    """Messaggi al secondo su una finestra mobile di 3 s."""
    def __init__(self):
        self.t = []

    def tick(self):
        now = time.monotonic()
        self.t.append(now)
        while self.t and self.t[0] < now - 3.0:
            self.t.pop(0)

    def hz(self):
        now = time.monotonic()
        t = [x for x in self.t if x > now - 3.0]
        return len(t) / 3.0

    def eta(self):
        return time.monotonic() - self.t[-1] if self.t else float('inf')


class Joystick:
    """Gamepad Linux (/dev/input/js0) letto direttamente, senza dipendenze. Assi tipici:
    0/1 levetta sinistra (imbardata / su-giu'), 3/4 levetta destra (rollio / beccheggio)."""
    def __init__(self, dev, mappa):
        self.dev, self.mappa = dev, mappa
        self.assi, self.tasti = {}, {}
        self.vivo = False
        self.ultimo = 0.0
        try:
            self.f = open(dev, 'rb')
        except OSError:
            self.f = None
            return
        self.vivo = True
        threading.Thread(target=self._leggi, daemon=True).start()

    def _leggi(self):
        try:
            while True:
                ev = self.f.read(8)
                if len(ev) < 8:
                    break
                _t, val, tipo, num = struct.unpack('IhBB', ev)
                tipo &= ~0x80
                if tipo == 2:
                    self.assi[num] = val / 32767.0
                elif tipo == 1:
                    self.tasti[num] = val
                self.ultimo = time.monotonic()
        except OSError:
            pass
        self.vivo = False

    def levette(self):
        g = lambda i, s=1.0: s * self.assi.get(i, 0.0)
        m = self.mappa
        zona = lambda v: 0.0 if abs(v) < 0.08 else v
        return (zona(g(m['rollio'])), zona(-g(m['beccheggio'])), zona(g(m['imbardata'])),
                zona(-g(m['quota'])))


class Drone(Node):
    def __init__(self, a):
        # nome_nodo: sorveglia_missione.py usa questa classe e gira insieme a collaudo.py nella
        # fase 8; due nodi con lo stesso nome sono ammessi da ROS ma fonte di confusione.
        super().__init__(getattr(a, 'nome_nodo', 'collaudo_c2x'), parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, bool(a.sim))])
        self.a = a
        self.lock = threading.Lock()
        px4 = a.px4_ns
        qos_px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sens = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.m = {}            # ultimi messaggi
        self.f = {k: Frequenza() for k in ['status', 'lpos', 'att', 'lio', 'salute', 'lidar',
                                           'imu', 'cam', 'manual_px4']}
        self.acc_imu = []
        sub = lambda tipo, topic, chiave, qos: self.create_subscription(
            tipo, topic, lambda msg, k=chiave: self._ricevi(k, msg), qos)
        sub(VehicleStatus, px4 + '/fmu/out/vehicle_status_v1', 'status', qos_px4)
        sub(VehicleLocalPosition, px4 + '/fmu/out/vehicle_local_position_v1', 'lpos', qos_px4)
        sub(VehicleAttitude, px4 + '/fmu/out/vehicle_attitude', 'att', qos_px4)
        sub(BatteryStatus, px4 + '/fmu/out/battery_status_v1', 'batt', qos_px4)
        sub(FailsafeFlags, px4 + '/fmu/out/failsafe_flags', 'fs', qos_px4)
        sub(EstimatorStatusFlags, px4 + '/fmu/out/estimator_status_flags', 'ekf', qos_px4)
        sub(VehicleLandDetected, px4 + '/fmu/out/vehicle_land_detected', 'land', qos_px4)
        sub(VehicleCommandAck, px4 + '/fmu/out/vehicle_command_ack', 'ack', qos_px4)
        sub(ManualControlSetpoint, px4 + '/fmu/out/manual_control_setpoint', 'manual_px4',
            qos_px4)
        sub(Odometry, a.lio, 'lio', qos_sens)
        sub(Float32MultiArray, a.salute, 'salute', qos_sens)
        self.qos_sens = qos_sens
        self.pub_cmd = self.create_publisher(VehicleCommand, px4 + '/fmu/in/vehicle_command',
                                             qos_px4)
        self.pub_manual = self.create_publisher(ManualControlSetpoint,
                                                px4 + '/fmu/in/manual_control_input', qos_px4)
        self.pub_offb = self.create_publisher(OffboardControlMode,
                                              px4 + '/fmu/in/offboard_control_mode', qos_px4)
        self.pub_traj = self.create_publisher(TrajectorySetpoint,
                                              px4 + '/fmu/in/trajectory_setpoint', qos_px4)
        # Levette virtuali: [rollio, beccheggio, imbardata, quota] in [-1, 1]; quota 0 =
        # tieni la quota in POSCTL. manual_attivo: si pubblicano a 50 Hz (dead-man di PX4).
        self.levette = [0.0, 0.0, 0.0, 0.0]
        self.manual_attivo = False
        self.offboard = None          # dict con 'pos' e 'vel' NED quando attivo
        self.target_system = a.sysid
        self.create_timer(0.02, self._ciclo)

    # ------------------------------------------------------------------ ricezione
    def _ricevi(self, k, msg):
        with self.lock:
            self.m[k] = msg
            if k in self.f:
                self.f[k].tick()
            if k == 'imu' and hasattr(msg, 'linear_acceleration'):
                la = msg.linear_acceleration
                self.acc_imu.append(math.sqrt(la.x ** 2 + la.y ** 2 + la.z ** 2))
                del self.acc_imu[:-400]
            if k == 'status' and self.a.sysid == 0 and msg.system_id:
                self.target_system = msg.system_id

    def get(self, k):
        with self.lock:
            return self.m.get(k)

    def iscrivi_temporaneo(self, topic, chiave):
        """Iscrizione col tipo che il topic ha davvero (il driver Livox per FAST-LIO pubblica
        livox_ros_driver2/CustomMsg, non PointCloud2). None se il topic non c'e'."""
        tipi = dict(self.get_topic_names_and_types())
        if topic not in tipi:
            return None, None
        tipo = tipi[topic][0]
        # Lidar e immagini solo contati, senza decodificarli (raw): un CustomMsg da ~20 000
        # punti costa a Python ~0.1 s, e l'IMU a 200 Hz misurava 9 Hz (19 ottobre 2026).
        return self.create_subscription(get_message(tipo), topic,
                                        lambda msg, k=chiave: self._ricevi(k, msg),
                                        self.qos_sens, raw=chiave != 'imu'), tipo

    # ------------------------------------------------------------------ comandi
    def us(self):
        return self.get_clock().now().nanoseconds // 1000

    def comando(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        m = VehicleCommand()
        m.timestamp = self.us()
        m.command = cmd
        m.param1, m.param2, m.param3, m.param4 = float(p1), float(p2), float(p3), float(p4)
        m.param5, m.param6, m.param7 = float(p5), float(p6), float(p7)
        m.target_system = self.target_system or 1
        m.target_component = 1
        m.source_system = 255
        m.source_component = 190
        m.from_external = True
        self.pub_cmd.publish(m)

    def arma(self):
        self.comando(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarma(self, forza=False):
        self.comando(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0,
                     FORZA_DISARMO if forza else 0.0)

    def modo(self, principale):
        self.comando(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, float(principale))

    def atterra(self):
        self.offboard = None
        self.levette = [0.0, 0.0, 0.0, 0.0]
        self.comando(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def prova_motore(self, i, valore, durata):
        """ACTUATOR_TEST: param1 valore [0,1] (NaN = fermo), param2 durata [s] (0 = rilascia),
        param5 funzione. PX4 (Commander::handleCommandActuatorTest): da 1000 in su e' la sua
        funzione + 1000 (1101 = Motore 1, come lo script di prova che funzionava); sotto 1000
        la numerazione MAVLink (1 = Motore 1). Fino al 21 ottobre 2026 qui c'era 100 + i: per
        PX4 nessun motore, risposta UNSUPPORTED e motori fermi (anche in simulazione).
        Restituisce l'esito di PX4 (0 = accettato) o None se non risponde entro 1 s."""
        with self.lock:
            self.m.pop('ack', None)
        self.comando(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, valore, durata, 0, 0, 1100 + i)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            ack = self.get('ack')
            if ack is not None and ack.command == VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST:
                return ack.result
            time.sleep(0.02)
        return None

    def rilascia_motore(self, i):
        self.comando(VehicleCommand.VEHICLE_CMD_ACTUATOR_TEST, float('nan'), 0, 0, 0, 1100 + i)

    def _ciclo(self):
        now = self.us()
        if self.manual_attivo:
            m = ManualControlSetpoint()
            m.timestamp = now
            m.timestamp_sample = now
            m.valid = True
            m.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
            m.roll, m.pitch, m.yaw, m.throttle = [float(v) for v in self.levette]
            m.sticks_moving = any(abs(v) > 0.05 for v in self.levette)
            self.pub_manual.publish(m)
        if self.offboard is not None:
            o = OffboardControlMode()
            o.timestamp = now
            o.position = True
            o.velocity = True
            self.pub_offb.publish(o)
            t = TrajectorySetpoint()
            t.timestamp = now
            t.position = [float(v) for v in self.offboard['pos']]
            t.velocity = [float(v) for v in self.offboard['vel']]
            nan = float('nan')
            t.acceleration = [nan, nan, nan]
            t.jerk = [nan, nan, nan]
            t.yaw = float(self.offboard['yaw'])
            t.yawspeed = nan
            self.pub_traj.publish(t)

    # ------------------------------------------------------------------ stato
    def armato(self):
        s = self.get('status')
        return s is not None and s.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def nav(self):
        s = self.get('status')
        return NOMI_NAV.get(s.nav_state, str(s.nav_state)) if s else '?'

    def in_aria(self):
        land = self.get('land')
        return self.armato() and land is not None and not land.landed


# ====================================================================== registro
class Registro:
    def __init__(self, sim):
        modo = 'sim' if sim else 'reale'
        self.dir = os.path.join(QUI, 'collaudo_registri', time.strftime('%Y-%m-%d_%H%M%S_') + modo)
        os.makedirs(self.dir, exist_ok=True)
        self.f = open(os.path.join(self.dir, 'registro.txt'), 'a')
        # Esiti separati per simulazione e drone vero: le fasi superate in simulazione non
        # devono far saltare le verifiche sul drone vero.
        self.esiti_file = os.path.join(QUI, 'collaudo_registri', f'esiti_{modo}.json')
        try:
            self.esiti = json.load(open(self.esiti_file))
        except (OSError, ValueError):
            self.esiti = {}
        # Fino al 7 novembre 2026 la fase 8 dava solo istruzioni ed era sempre "superata": quel
        # vecchio esito non vale come missione fatta.
        if self.esiti.get('8', {}).get('dettagli') == 'solo istruzioni':
            del self.esiti['8']

    def scrivi(self, testo):
        riga = time.strftime('%H:%M:%S ') + testo
        self.f.write(riga + '\n')
        self.f.flush()
        return riga

    def esito(self, fase, ok, dettagli):
        self.esiti[str(fase)] = {'ok': bool(ok), 'quando': time.strftime('%Y-%m-%d %H:%M:%S'),
                                 'dettagli': dettagli}
        json.dump(self.esiti, open(self.esiti_file, 'w'), indent=1)
        self.scrivi(f'FASE {fase}: {"SUPERATA" if ok else "NON SUPERATA"} - {dettagli}')


def stampa(reg, testo=''):
    print(testo)
    if testo:
        reg.scrivi(testo)


def chiedi(reg, domanda):
    r = input(domanda + ' ').strip().lower()
    reg.scrivi(f'{domanda} -> {r}')
    return r


def attendi(cond, timeout, passo=0.1):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(passo)
    return cond()


# ====================================================================== fasi a terra
# Carica minima (frazione di PX4): sotto si atterra, e la fase 0 non lascia partire. 15% come
# BAT_LOW_THR di PX4 (25 ottobre 2026, prima 50% alla partenza e 25% in volo).
BATTERIA_MIN = 0.15


def fase0(d, reg, a):
    stampa(reg, '== FASE 0: collegamento con PX4 ==')
    stampa(reg, 'Attendo lo stato di PX4 (fino a 15 s)...')
    if not attendi(lambda: d.get('status') is not None, 15):
        stampa(reg, 'Nessun messaggio da PX4. Controlla: agente DDS avviato sulla porta giusta '
                    '(avvia_collaudo.sh), cavo USB, SYS_USB_AUTO = 0 e /etc/extras.txt '
                    'sulla scheda SD.')
        return False, 'nessun vehicle_status'
    s = d.get('status')
    stampa(reg, f'PX4 risponde: sistema {s.system_id}, modalita\' {d.nav()}, '
                f'{"ARMATO" if d.armato() else "disarmato"}, controlli pre-volo '
                f'{"OK" if s.pre_flight_checks_pass else "NON superati"}')
    b = d.get('batt')
    if b is not None:
        stampa(reg, f'Batteria: {b.voltage_v:.2f} V, {100 * b.remaining:.0f}%, '
                    f'{b.cell_count} celle')
        if b.remaining < 0:
            # -1 = carica sconosciuta (19 ottobre 2026: BAT1_N_CELLS = 0 con una 4S a 16 V)
            stampa(reg, 'PX4 non conosce la carica: se le celle sono 0 imposta BAT1_N_CELLS '
                        '(e controlla la tensione con un tester).')
    fs = d.get('fs')
    if fs is not None:
        attivi = [k for k, t in fs.get_fields_and_field_types().items()
                  if t == 'boolean' and getattr(fs, k)]
        stampa(reg, 'Condizioni di failsafe attive: ' + (', '.join(attivi) or 'nessuna'))
    att = d.get('att')
    if att is not None:
        r, p, _ = eulero_ned(tuple(att.q))
        stampa(reg, f'Assetto visto da PX4: rollio {r:.0f}, beccheggio {p:.0f} gradi')
        if abs(r) > 15 or abs(p) > 15:
            # fd_critical_failure fra i failsafe: armamento negato
            stampa(reg, 'PX4 vede il drone INCLINATO: se il drone e\' in piano, il Pixhawk non e\' '
                        'fissato in piano con la freccia verso il muso (o SENS_BOARD_ROT e\' sbagliato).')
    # Levette virtuali: PX4 le deve accettare come comando manuale (COM_RC_IN_MODE = 1).
    stampa(reg, 'Provo le levette virtuali per 3 s (motori fermi, drone disarmato)...')
    d.levette = [0.0, 0.0, 0.0, -1.0]
    d.manual_attivo = True
    ok_manual = attendi(lambda: d.get('manual_px4') is not None and d.get('manual_px4').valid
                        and d.f['manual_px4'].eta() < 0.5, 3.0)
    mp = d.get('manual_px4')
    d.manual_attivo = False
    d.levette = [0.0, 0.0, 0.0, 0.0]
    if ok_manual:
        stampa(reg, f'Comando manuale accettato da PX4 (sorgente {mp.data_source}).')
    else:
        stampa(reg, 'PX4 NON accetta le levette virtuali: imposta COM_RC_IN_MODE = 1 '
                    '(collaudo_px4.params) e riavvia PX4.')
    r = chiedi(reg, 'Hai caricato collaudo_px4.params con QGroundControl (e controllato '
                    'MPC_THR_HOVER)? [s/n]')
    ok = ok_manual and r == 's' and (b is None or b.remaining > BATTERIA_MIN)
    if b is not None and 0 <= b.remaining <= BATTERIA_MIN:
        stampa(reg, f'Batteria sotto il {100 * BATTERIA_MIN:.0f}%: caricala prima di volare.')
    return ok, f'manuale={ok_manual}, parametri={r == "s"}, prevolo={s.pre_flight_checks_pass}'


def fase1(d, reg, a):
    stampa(reg, '== FASE 1: sensori ==')
    subs = []
    for topic, k in [(a.lidar, 'lidar'), (a.imu, 'imu'), (a.camera, 'cam')]:
        s_, tipo = d.iscrivi_temporaneo(topic, k)
        stampa(reg, f'{topic}: ' + (tipo if s_ else 'NON PUBBLICATO'))
        if s_:
            subs.append(s_)
    stampa(reg, 'Misuro per 5 s...')
    time.sleep(5)
    for s_ in subs:
        d.destroy_subscription(s_)
    hz_l, hz_i, hz_c = d.f['lidar'].hz(), d.f['imu'].hz(), d.f['cam'].hz()
    acc = sorted(d.acc_imu)[len(d.acc_imu) // 2] if d.acc_imu else float('nan')
    stampa(reg, f'lidar {a.lidar}: {hz_l:.1f} Hz (atteso ~10)')
    stampa(reg, f'IMU {a.imu}: {hz_i:.0f} Hz (atteso >= 100), accelerazione a riposo '
                f'{acc:.2f} (1.00 se in g come il Mid-360, 9.81 se in m/s2)')
    stampa(reg, f'telecamera {a.camera}: {hz_c:.1f} Hz (serve solo alla mappa colorata)')
    acc_ok = 0.95 < acc < 1.05 or 9.6 < acc < 10.0
    ok = hz_l > 8.0 and hz_i > 100.0 and acc_ok
    if not ok:
        stampa(reg, 'Controlla: driver Livox avviato e IP del lidar (MID360s_config.json), '
                    'cavo Ethernet, topic giusti (--lidar/--imu).')
    return ok, f'lidar {hz_l:.1f} Hz, imu {hz_i:.0f} Hz, acc {acc:.2f}, camera {hz_c:.1f} Hz'


def fase2(d, reg, a):
    stampa(reg, '== FASE 2: FAST-LIO a drone fermo ==')
    stampa(reg, 'Drone appoggiato a terra in piano, NESSUNO lo tocca. Misuro per 30 s...')
    if not attendi(lambda: d.get('lio') is not None, 10):
        stampa(reg, f'Nessuna stima su {a.lio}: FAST-LIO non gira o non riceve il lidar.')
        return False, 'nessuna stima'
    pos, ag = [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30:
        o = d.get('lio')
        p = o.pose.pose.position
        pos.append((p.x, p.y, p.z))
        s = d.get('salute')
        if s is not None and len(s.data) >= 2:
            ag.append(s.data[1])
        time.sleep(0.1)
    hz = d.f['lio'].hz()
    deriva = math.dist(pos[0], pos[-1])
    escurs = max(math.dist(pos[0], q) for q in pos)
    ag_min = min(ag) if ag else float('nan')
    stampa(reg, f'stima a {hz:.1f} Hz, deriva in 30 s {100 * deriva:.1f} cm (massima '
                f'escursione {100 * escurs:.1f} cm), punti agganciati minimi {ag_min:.0f}')
    ok = hz > 8.0 and escurs < 0.05 and (not ag or ag_min > 500)
    return ok, f'{hz:.1f} Hz, escursione {100 * escurs:.1f} cm, agganciati min {ag_min:.0f}'


def fase3(d, reg, a):
    stampa(reg, '== FASE 3: assi e stima dentro PX4 (drone mosso A MANO, motori spenti) ==')
    if a.sim:
        stampa(reg, '(simulazione: il drone non si puo\' muovere a mano, controllo solo a riposo)')
    def letture():
        o, att, lp = d.get('lio'), d.get('att'), d.get('lpos')
        if o is None or att is None or lp is None:
            return None
        (pn, pe, pd), qn = lio_in_ned(o)
        return dict(lio_rpy=eulero_ned(qn), px4_rpy=eulero_ned(tuple(att.q)),
                    lio_pos=(pn, pe, pd), px4_pos=(lp.x, lp.y, lp.z), lp=lp)
    if not attendi(lambda: letture() is not None, 10):
        return False, 'mancano stima o assetto di PX4'
    ekf = d.get('ekf')
    flags = {k: bool(getattr(ekf, k)) for k in ['cs_ev_pos', 'cs_ev_hgt', 'cs_ev_yaw']} if ekf else {}
    lp = d.get('lpos')
    stampa(reg, f'EKF2 usa la visione: {flags}; posizione PX4 valida xy={lp.xy_valid} '
                f'z={lp.z_valid}')
    r0 = letture()
    scarto_pos = math.dist(r0['lio_pos'], r0['px4_pos'])
    d_r = abs(diff_angolo(r0['lio_rpy'][0], r0['px4_rpy'][0]))
    d_p = abs(diff_angolo(r0['lio_rpy'][1], r0['px4_rpy'][1]))
    stampa(reg, f'a riposo: posizione PX4 - FAST-LIO {100 * scarto_pos:.1f} cm, rollio '
                f'{d_r:.1f} gradi, beccheggio {d_p:.1f} gradi di differenza')
    ok_riposo = all(flags.values()) and lp.xy_valid and lp.z_valid and scarto_pos < 0.10 \
        and d_r < 2.0 and d_p < 2.0
    if a.sim:
        return ok_riposo, f'riposo: ev={flags}, scarto {100 * scarto_pos:.1f} cm'
    prove = [
        ('Inclina il drone IN AVANTI (muso giu\') di 10-20 gradi e tienilo', 'p', -1),
        ('Inclina il drone verso DESTRA (lato destro giu\') di 10-20 gradi e tienilo', 'r', +1),
    ]
    esiti = []
    for testo, asse, segno in prove:
        input(f'\n{testo}, poi premi INVIO (hai 10 s)...')
        i = 1 if asse == 'p' else 0
        migliore = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10:
            r = letture()
            if abs(r['px4_rpy'][i]) > 8 and (migliore is None or
                                              abs(r['px4_rpy'][i]) > abs(migliore['px4_rpy'][i])):
                migliore = r
            time.sleep(0.05)
        if migliore is None:
            stampa(reg, '  inclinazione sotto gli 8 gradi: ripeti la fase')
            esiti.append(False)
            continue
        vp, vl = migliore['px4_rpy'][i], migliore['lio_rpy'][i]
        ok = abs(diff_angolo(vp, vl)) < 3.0 and (vp * segno > 0) and (vl * segno > 0)
        stampa(reg, f'  PX4 {vp:+.1f} gradi, FAST-LIO {vl:+.1f} gradi -> {"OK" if ok else "SBAGLIATO"}')
        if not ok:
            stampa(reg, '  ASSI DEL LIDAR DIVERSI DA QUELLI DEL DRONE: NON VOLARE. Controlla il '
                        'montaggio del Mid-360 e la trasformazione in lio_to_px4_bridge.')
        esiti.append(ok)
    input('\nRimetti il drone a terra fermo, poi SPOSTALO A MANO IN AVANTI di circa 50 cm '
          '(dritto, senza ruotarlo) e premi INVIO quando e\' fermo...')
    r1 = letture()
    dp_px4 = [r1['px4_pos'][k] - r0['px4_pos'][k] for k in range(2)]
    dp_lio = [r1['lio_pos'][k] - r0['lio_pos'][k] for k in range(2)]
    psi = math.radians(r1['px4_rpy'][2])
    avanti_px4 = dp_px4[0] * math.cos(psi) + dp_px4[1] * math.sin(psi)
    avanti_lio = dp_lio[0] * math.cos(psi) + dp_lio[1] * math.sin(psi)
    ok_t = 0.3 < avanti_px4 < 0.8 and abs(avanti_px4 - avanti_lio) < 0.08
    stampa(reg, f'  in avanti: PX4 {100 * avanti_px4:.0f} cm, FAST-LIO {100 * avanti_lio:.0f} cm '
                f'-> {"OK" if ok_t else "SBAGLIATO"}')
    esiti.append(ok_t)
    input('\nRuota il drone A SINISTRA (antiorario visto dall\'alto) di circa 90 gradi, '
          'poi premi INVIO...')
    r2 = letture()
    dy_px4 = diff_angolo(r2['px4_rpy'][2], r1['px4_rpy'][2])
    dy_lio = diff_angolo(r2['lio_rpy'][2], r1['lio_rpy'][2])
    ok_y = -120 < dy_px4 < -60 and abs(diff_angolo(dy_px4, dy_lio)) < 5
    stampa(reg, f'  rotazione: PX4 {dy_px4:+.0f} gradi, FAST-LIO {dy_lio:+.0f} gradi (attesi '
                f'circa -90 in NED) -> {"OK" if ok_y else "SBAGLIATO"}')
    esiti.append(ok_y)
    ok = ok_riposo and all(esiti)
    return ok, f'riposo {ok_riposo}, prove {esiti}'


# Quad X di PX4 (CA_ROTOR0-3 nei parametri del drone, 19 ottobre 2026): angolo e verso visto
# da sopra di ogni motore. a = antiorario, o = orario.
ANGOLI = {1: 'avanti-destra', 2: 'dietro-sinistra', 3: 'avanti-sinistra', 4: 'dietro-destra'}
VERSO_GIUSTO = {1: 'a', 2: 'a', 3: 'o', 4: 'o'}
SCHEMA_QUAD_X = """        AVANTI
     3 (o)   1 (a)
         \\ /
         / \\
     2 (a)   4 (o)
   o = orario, a = antiorario, visti da SOPRA"""


def correzione_motori(oss):
    """oss[i] = (angolo che ha girato, verso) nella prova del Motore i (1-4); angolo 0 = nessuno.
    Uscite come nei parametri salvati il 19 ottobre: MAIN i -> Motore i (PWM_MAIN_FUNCi = 100+i).
    Restituisce (tutto giusto, righe da stampare, parametri da cambiare)."""
    righe, param = [], {}
    fermi = [i for i in range(1, 5) if oss[i][0] == 0]
    angoli = [oss[i][0] for i in range(1, 5) if oss[i][0]]
    for i in fermi:
        righe.append(f'Motore {i} (uscita MAIN {i}): non ha girato nessun motore. Controlla cavo del '
                     f'segnale su MAIN {i}, alimentazione dell\'ESC, calibrazione degli ESC.')
    if len(set(angoli)) != len(angoli):
        righe.append('Due prove hanno fatto girare lo stesso angolo: ripeti la fase guardando bene.')
    sposta = [i for i in range(1, 5) if oss[i][0] and oss[i][0] != i]
    if sposta and not fermi and len(set(angoli)) == 4:
        righe.append('POSIZIONI: ogni uscita comanda il motore di un altro angolo. Due modi, uno dei due:')
        righe.append('  a) sposta i connettori del segnale degli ESC sulle uscite MAIN del Pixhawk:')
        for i in sposta:
            righe.append(f'     il connettore ora su MAIN {i} (ESC {ANGOLI[oss[i][0]]}) va su MAIN {oss[i][0]}')
        righe.append('  b) oppure lascia i cavi e cambia i parametri (config.txt sulla scheda SD):')
        for i in sposta:
            param[f'PWM_MAIN_FUNC{i}'] = 100 + oss[i][0]
            righe.append(f'     PWM_MAIN_FUNC{i} = {100 + oss[i][0]} (Motore {oss[i][0]})')
    for i in range(1, 5):
        c, v = oss[i]
        if c and v in ('a', 'o') and v != VERSO_GIUSTO[c]:
            righe.append(f'VERSO: il motore {ANGOLI[c]} gira {"antiorario" if v == "a" else "orario"}, '
                         f'deve girare {"antiorario" if VERSO_GIUSTO[c] == "a" else "orario"}: con ESC PWM '
                         '(il tuo drone: PWM_MAIN_TIM = 400 Hz) scambia DUE dei tre fili fra ESC e motore.')
    giusto = all(oss[i][0] == i and oss[i][1] == VERSO_GIUSTO[i] for i in range(1, 5))
    return giusto, righe, param


def fase4(d, reg, a):
    stampa(reg, '== FASE 4: motori SENZA ELICHE ==')
    if chiedi(reg, 'Scrivi SENZA ELICHE per confermare che le eliche sono smontate:') != \
            'senza eliche':
        return False, 'eliche non confermate smontate'
    if d.armato():
        return False, 'drone gia\' armato'
    stampa(reg, 'Schema dei motori (Quad X di PX4), drone visto da SOPRA col muso in alto:')
    stampa(reg, SCHEMA_QUAD_X)
    oss = {}
    for i in range(1, 5):
        while True:
            input(f'\nProva Motore {i}: premi INVIO, gira per 1.5 s al 15%. Guarda QUALE angolo '
                  'gira e IN CHE VERSO (da sopra).')
            esito_px4 = d.prova_motore(i, 0.15, 1.5)
            if esito_px4 != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                stampa(reg, '  PX4 NON esegue la prova: ' + {
                    None: 'nessuna risposta (collegamento DDS?)',
                    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED:
                        'rifiutata (COM_MOT_TEST_EN = 1? drone disarmato? pulsante di sicurezza '
                        'del GPS premuto, se c\'e\'?)',
                    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'funzione non valida',
                }.get(esito_px4, f'esito {esito_px4}'))
                return False, f'prova motore {i} non eseguita da PX4 (esito {esito_px4})'
            time.sleep(2.0)
            d.rilascia_motore(i)
            c = chiedi(reg, '  Quale angolo ha girato? 1 avanti-destra, 2 dietro-sinistra, '
                            '3 avanti-sinistra, 4 dietro-destra, 0 nessuno, r = rifai la prova:')
            if c == 'r':
                continue
            if c not in ('0', '1', '2', '3', '4'):
                stampa(reg, '  rispondi con 0, 1, 2, 3, 4 o r')
                continue
            c = int(c)
            v = ''
            while c and v not in ('o', 'a', 'r'):
                v = chiedi(reg, '  In che verso, visto da sopra? o = orario, a = antiorario, '
                                'r = rifai la prova:')
            if v == 'r':
                continue
            oss[i] = (c, v)
            esito = 'GIUSTO' if (c == i and v == VERSO_GIUSTO[i]) else 'da correggere'
            stampa(reg, f'  Motore {i}: ' + (f'angolo {ANGOLI[c]}, ' + ('orario' if v == 'o' else
                        'antiorario') if c else 'non gira') + f' -> {esito}')
            break
    giusto, righe, param = correzione_motori(oss)
    descr = ' '.join(f'M{i}:{oss[i][0]}{oss[i][1]}' for i in range(1, 5))
    if not giusto:
        stampa(reg, '\nCOSA CORREGGERE:')
        for r in righe:
            stampa(reg, r)
        if param:
            f = os.path.join(reg.dir, 'config.txt')
            with open(f, 'w', newline='\n') as fp:
                for k, v in param.items():
                    fp.write(f'param set {k} {v}\n')
                fp.write('param save\n')
            stampa(reg, f'  config.txt per la strada b) pronto in {f}: va in /etc della scheda SD, poi un '
                        'riavvio VERO (stacca anche l\'USB) e poi toglilo.')
        stampa(reg, 'Dopo la correzione rifai la fase 4.')
        return False, f'motori da correggere ({descr})'
    stampa(reg, 'Armamento: i motori girano al minimo per 3 s, poi PROVA DELL\'EMERGENZA.')
    d.levette = [0.0, 0.0, 0.0, -1.0]
    d.manual_attivo = True
    time.sleep(0.5)
    d.modo(MODO_POSCTL)
    time.sleep(0.5)
    d.arma()
    if not attendi(d.armato, 3):
        d.manual_attivo = False
        ack = d.get('ack')
        stampa(reg, f'PX4 rifiuta l\'armamento (ultimo esito {ack.result if ack else "?"}). '
                    'Guarda i messaggi di PX4 (controlli pre-volo).')
        return False, 'armamento rifiutato'
    time.sleep(3)
    input('Ora premi INVIO: e\' come premere SPAZIO in volo (emergenza).')
    t0 = time.monotonic()
    d.disarma(forza=True)
    fermo = attendi(lambda: not d.armato(), 2.0, 0.01)
    dt = time.monotonic() - t0
    d.manual_attivo = False
    stampa(reg, f'emergenza: motori fermi in {1000 * dt:.0f} ms' if fermo else
                'EMERGENZA NON RIUSCITA: NON VOLARE')
    return fermo, f'motori ok, emergenza {1000 * dt:.0f} ms'


# ====================================================================== volo (curses)
class Volo:
    """Schermata di volo: levette da tastiera o joystick, sicurezze automatiche."""
    def __init__(self, d, reg, a, raggio, quota_max, quota_decollo, titolo, quadrato=False):
        self.d, self.reg, self.a = d, reg, a
        self.raggio, self.quota_max, self.quota_decollo = raggio, quota_max, quota_decollo
        self.titolo, self.quadrato = titolo, quadrato
        self.eventi = []
        self.tasti = {}          # asse -> (valore, istante)
        self.origine = None
        self.decollo = False
        self.dec = None          # decollo in corso (offboard con quota che sale)
        self.allarme = None
        self.massimi = dict(dist=0.0, quota=0.0)
        self.campioni = []
        self.js = Joystick(a.joystick, json.loads(a.mappa_joystick)) if a.joystick else None
        self.chiudi = False
        self.quadrato_finito = False

    def evento(self, testo):
        self.eventi.append(self.reg.scrivi(testo))
        del self.eventi[:-6]

    def quota(self):
        lp = self.d.get('lpos')
        return -lp.z if lp else float('nan')

    def dist(self):
        lp = self.d.get('lpos')
        if lp is None or self.origine is None:
            return 0.0
        return math.hypot(lp.x - self.origine[0], lp.y - self.origine[1])

    def sicurezze(self):
        """Ritorna il motivo per atterrare, o None."""
        d = self.d
        if not d.armato():
            return None
        lp = d.get('lpos')
        if d.f['lio'].eta() > 0.5:
            return f'stima di FAST-LIO ferma da {d.f["lio"].eta():.1f} s'
        s = d.get('salute')
        if s is not None and len(s.data) >= 2 and s.data[1] < 300 and d.f['salute'].eta() < 1:
            return f'FAST-LIO aggancia solo {s.data[1]:.0f} punti'
        if lp is None or not lp.xy_valid or not lp.z_valid:
            return 'posizione di PX4 non valida'
        if self.dist() > self.raggio:
            return f'fuori dal recinto: {self.dist():.2f} m > {self.raggio} m'
        if self.quota() > self.quota_max:
            return f'troppo alto: {self.quota():.2f} m > {self.quota_max} m'
        b = d.get('batt')
        if b is not None and 0 <= b.remaining < BATTERIA_MIN:
            return f'batteria al {100 * b.remaining:.0f}%'
        return None

    def avvia(self):
        d = self.d
        d.levette = [0.0, 0.0, 0.0, -1.0]
        d.manual_attivo = True
        lp = d.get('lpos')
        self.origine = (lp.x, lp.y) if lp else None

        def hup(*_):
            if d.armato():
                d.atterra()
            self.reg.scrivi('SIGHUP/SIGTERM: sessione persa, atterraggio comandato')
            self.chiudi = True
        signal.signal(signal.SIGHUP, hup)
        signal.signal(signal.SIGTERM, hup)
        try:
            try:
                curses.wrapper(self._schermo)
            except curses.error as e:
                self.allarme = f'schermata di volo non avviata ({e}): usa un terminale vero'
                self.reg.scrivi(self.allarme)
        finally:
            if d.in_aria():
                d.atterra()
                self.reg.scrivi('uscita dalla schermata in volo: atterraggio comandato')
                attendi(lambda: not d.armato(), 30)
            d.offboard = None
            d.manual_attivo = False
            signal.signal(signal.SIGHUP, signal.SIG_DFL)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
        return self.risultato()

    def risultato(self):
        cs = self.campioni
        if not cs:
            return False, 'nessun volo'
        # Oscillazione da fermo: per ogni tratto continuo con le levette al centro (da piu' di
        # 2 s) lo scarto massimo dalla posizione media del tratto; si riporta il peggiore.
        tratti, cur = [], []
        for c in cs:
            if c['fermo']:
                cur.append(c)
            elif cur:
                tratti.append(cur)
                cur = []
        if cur:
            tratti.append(cur)
        dev = float('nan')
        for tr in tratti:
            if len(tr) < 20:
                continue
            mx = sum(c['x'] for c in tr) / len(tr)
            my = sum(c['y'] for c in tr) / len(tr)
            d_tr = max(math.hypot(c['x'] - mx, c['y'] - my) for c in tr)
            dev = d_tr if math.isnan(dev) else max(dev, d_tr)
        txt = (f'volo {cs[-1]["t"] - cs[0]["t"]:.0f} s, distanza max {self.massimi["dist"]:.2f} m, '
               f'quota max {self.massimi["quota"]:.2f} m, oscillazione da fermo {100 * dev:.0f} cm, '
               f'allarme: {self.allarme or "nessuno"}'
               + (f', quadrato {"completato" if self.quadrato_finito else "NON fatto"}'
                  if self.quadrato else ''))
        # fase 7: superata solo col quadrato completato (prima bastava volare e atterrare)
        ok = self.allarme is None and self.decollo and not self.d.armato() and \
            (self.quadrato_finito or not self.quadrato)
        return ok, txt

    def _schermo(self, scr):
        d = self.d
        try:
            curses.set_escdelay(25)   # di serie 1 s: le frecce grezze arrivavano in ritardo
        except AttributeError:
            pass
        curses.curs_set(0)
        scr.nodelay(True)
        scr.timeout(30)
        frecce = {curses.KEY_UP: (1, +1), curses.KEY_DOWN: (1, -1), curses.KEY_RIGHT: (0, +1),
                  curses.KEY_LEFT: (0, -1), ord('w'): (3, +1), ord('s'): (3, -1),
                  ord('d'): (2, +1), ord('a'): (2, -1)}
        quad = None
        t_fermo = None
        while not self.chiudi:
            k = scr.getch()
            now = time.monotonic()
            while k != -1:
                if k == 27:
                    # Frecce come sequenze grezze: ESC [ A (modo normale, es. PuTTY) o ESC O A
                    # (modo applicazione); curses riconosce solo quella del suo terminfo.
                    k2, k3 = scr.getch(), scr.getch()
                    if k2 in (ord('['), ord('O')) and k3 in (65, 66, 67, 68):
                        k = {65: curses.KEY_UP, 66: curses.KEY_DOWN, 67: curses.KEY_RIGHT,
                             68: curses.KEY_LEFT}[k3]
                if k == ord(' '):
                    d.offboard = None
                    d.disarma(forza=True)
                    self.allarme = 'EMERGENZA (spazio)'
                    self.evento('EMERGENZA: motori spenti')
                elif k == ord('l'):
                    quad = None
                    d.atterra()
                    self.evento('atterraggio comandato (l)')
                elif k == ord('h'):
                    quad = None
                    if self.dec is not None:
                        self._fine_decollo('decollo interrotto (h)')
                    if d.offboard is not None:
                        d.offboard = None
                        d.modo(MODO_POSCTL)
                    self.tasti.clear()
                    self.evento('fermo (h)')
                elif k == ord('t') and not d.armato() and self.dec is None:
                    self._inizia_decollo()
                elif k == ord('o') and self.quadrato and quad is None:
                    # Solo con il drone DAVVERO alla quota di decollo: in simulazione un
                    # offboard partito da terra (PX4 lo dava gia' in aria) l'ha ribaltato.
                    if d.in_aria() and self.dec is None and \
                            self.quota() > self.quota_decollo - 0.15:
                        quad = self._inizia_quadrato()
                    else:
                        self.evento(f'quadrato rifiutato: quota {self.quota():.2f} m, '
                                    f'serve almeno {self.quota_decollo - 0.15:.2f} m e salita finita')
                elif k == ord('q') and not d.armato():
                    return
                elif k in frecce:
                    asse, segno = frecce[k]
                    self.tasti[asse] = (segno * self.a.stick, now)
                    if self.dec is not None:
                        self._fine_decollo('decollo interrotto dalle levette: comando manuale')
                    if d.offboard is not None or quad is not None:
                        quad = None
                        d.offboard = None
                        d.modo(MODO_POSCTL)
                        self.evento('levette: offboard interrotto, comando manuale')
                k = scr.getch()
            # levette: tastiera (tasto tenuto = ripetizione del terminale) o joystick
            lev = [0.0, 0.0, 0.0, 0.0]
            for asse, (v, t) in list(self.tasti.items()):
                if now - t < 0.25:
                    lev[asse] = v
                else:
                    del self.tasti[asse]
            if self.js is not None:
                if self.js.vivo:
                    jl = self.js.levette()
                    if any(abs(v) > 0 for v in jl):
                        lev = [self.a.stick * v for v in jl]
                elif d.in_aria() and self.allarme is None:
                    self.allarme = 'joystick scollegato'
                    d.atterra()
                    self.evento('joystick scollegato: atterraggio')
            q = self.quota()
            if self.dec is not None:
                self._passo_decollo(now, q)
            # Distacco mancato: armato, decollo chiesto da 8 s (12 in simulazione) e ancora sotto
            # 10 cm. E' a terra, quindi si spengono i motori (atterrare non servirebbe: PX4 lo crede
            # in aria). Se il drone non sale, PX4 alza la spinta fino al massimo in 1-2 s: il 21
            # ottobre 2026 il drone vero e' rimasto 12 s a terra coi motori quasi al massimo.
            # 8 s = armamento + avvio dei motori (COM_SPOOLUP_TIME) + rampa (MPC_TKO_RAMP_T) + margine.
            t_distacco = 12.0 if self.a.sim else 8.0
            if self.dec is not None and d.armato() and not math.isnan(q) and q < 0.10 \
                    and now - self.dec['t0'] > t_distacco and self.allarme is None:
                self.allarme = (f'non si stacca da terra in {t_distacco:.0f} s (corda tesa? eliche '
                                'montate al contrario? ESC calibrati?)')
                self.dec = None
                d.offboard = None
                d.disarma(forza=True)
                self.evento('SICUREZZA: ' + self.allarme + ' -> motori spenti')
            if not d.armato():
                lev[3] = -1.0                      # a terra: levetta del gas in basso
            if quad is not None:
                quad = self._passo_quadrato(quad)
            d.levette = lev
            # sicurezze
            motivo = self.sicurezze()
            if motivo and self.allarme is None and d.nav() not in ('LAND',):
                self.allarme = motivo
                quad = None
                d.atterra()
                self.evento(f'SICUREZZA: {motivo} -> atterraggio')
            # campioni e massimi
            lp = d.get('lpos')
            if lp is not None and d.armato():
                fermo = all(abs(v) < 0.05 for v in lev[:3]) and abs(lev[3]) < 0.05 and \
                    q > self.quota_decollo - 0.1 and quad is None
                t_fermo = (t_fermo or now) if fermo else None
                self.campioni.append(dict(t=now, x=lp.x, y=lp.y, z=lp.z,
                                          fermo=bool(t_fermo and now - t_fermo > 2.0)))
                self.massimi['dist'] = max(self.massimi['dist'], self.dist())
                self.massimi['quota'] = max(self.massimi['quota'], q)
            self._disegna(scr, lev, quad)

    # ------------------------------------------------------------- decollo
    # In OFFBOARD con la quota di riferimento che sale a VEL_DECOLLO, come lo stack di
    # esplorazione; arrivato in quota passa a POSCTL con le levette al centro (fermo). Prima si
    # decollava in POSCTL con la levetta del gas: in simulazione PX4 si dichiarava decollato ma
    # il drone restava appoggiato per 10-20 s, con la spinta che recuperava piano dopo la rampa
    # di decollo, e poi si staccava di colpo. Con la quota di riferimento che sale, se il drone
    # resta a terra lo scarto cresce e la spinta con lui.
    VEL_DECOLLO = 0.4

    def _inizia_decollo(self):
        d = self.d
        lp, att = d.get('lpos'), d.get('att')
        if lp is None or att is None:
            self.evento('decollo rifiutato: manca la posizione di PX4')
            return
        yaw = math.radians(eulero_ned(tuple(att.q))[2])
        self.origine = (lp.x, lp.y)
        now = time.monotonic()
        self.dec = dict(x=lp.x, y=lp.y, z=lp.z, zt=lp.z - self.quota_decollo, yaw=yaw, t0=now,
                        t=now, t_cmd=0.0)
        d.offboard = dict(pos=(lp.x, lp.y, lp.z), vel=(0.0, 0.0, 0.0), yaw=yaw)
        self.decollo = True
        self.evento(f'decollo verso {self.quota_decollo:.1f} m (offboard, {self.VEL_DECOLLO} m/s)')

    def _passo_decollo(self, now, q):
        d, dc = self.d, self.dec
        dt = now - dc['t']
        dc['t'] = now
        if not d.armato():
            # setpoint gia' in arrivo da 0.3 s, poi OFFBOARD + armamento, ripetuti ogni secondo
            if now - dc['t0'] > 0.3 and now - dc['t_cmd'] > 1.0:
                d.modo(MODO_OFFBOARD)
                d.arma()
                dc['t_cmd'] = now
            if now - dc['t0'] > 6.0:
                self._fine_decollo('armamento rifiutato da PX4 (controlli pre-volo?)')
            return
        if not math.isnan(q) and q >= self.quota_decollo - 0.05:
            self._fine_decollo(f'quota di decollo raggiunta ({q:.2f} m): fermo in POSCTL')
            if self.quadrato:
                self.evento('IN QUOTA: premi o per il quadrato automatico')
            return
        dc['z'] = max(dc['z'] - self.VEL_DECOLLO * dt, dc['zt'])     # NED: su = z piu' piccolo
        vz = -self.VEL_DECOLLO if dc['z'] > dc['zt'] else 0.0
        d.offboard = dict(pos=(dc['x'], dc['y'], dc['z']), vel=(0.0, 0.0, vz), yaw=dc['yaw'])

    def _fine_decollo(self, motivo):
        self.dec = None
        self.d.offboard = None
        if self.d.armato():
            self.d.modo(MODO_POSCTL)
        self.evento(motivo)

    def _inizia_quadrato(self):
        lp, att = self.d.get('lpos'), self.d.get('att')
        yaw = math.radians(eulero_ned(tuple(att.q))[2])
        c, s = math.cos(yaw), math.sin(yaw)
        x0, y0, z0 = lp.x, lp.y, lp.z
        lato = self.a.lato
        # quadrato davanti-destra rispetto al muso: avanti, destra, indietro, sinistra
        locali = [(lato, 0), (lato, lato), (0, lato), (0, 0)]
        punti = [(x0 + c * f - s * r, y0 + s * f + c * r) for f, r in locali]
        self.d.offboard = dict(pos=(x0, y0, z0), vel=(0.0, 0.0, 0.0), yaw=yaw)
        time.sleep(0.3)   # PX4 vuole setpoint offboard gia' in arrivo prima del cambio
        self.d.modo(MODO_OFFBOARD)
        self.evento(f'quadrato di {lato} m a {self.a.vel_auto} m/s (offboard)')
        return dict(punti=punti, i=0, p=(x0, y0), z=z0, yaw=yaw, t=time.monotonic(), sosta=None)

    def _passo_quadrato(self, qd):
        now = time.monotonic()
        dt = now - qd['t']
        qd['t'] = now
        if self.d.nav() != 'OFFBOARD' and now - qd.get('t0', now) > 2.0:
            self.evento('PX4 non e\' in OFFBOARD: quadrato annullato')
            self.d.offboard = None
            return None
        qd.setdefault('t0', now)
        if qd['i'] >= len(qd['punti']):
            self.d.offboard = None
            self.d.modo(MODO_POSCTL)
            self.evento('quadrato finito: fermo in POSCTL (l per atterrare)')
            self.quadrato_finito = True
            return None
        tx, ty = qd['punti'][qd['i']]
        px, py = qd['p']
        dx, dy = tx - px, ty - py
        dist = math.hypot(dx, dy)
        passo = self.a.vel_auto * dt
        if dist <= passo:
            qd['p'] = (tx, ty)
            if qd['sosta'] is None:
                qd['sosta'] = now
            if now - qd['sosta'] > 2.0:
                qd['i'] += 1
                qd['sosta'] = None
            vel = (0.0, 0.0, 0.0)
        else:
            qd['p'] = (px + dx / dist * passo, py + dy / dist * passo)
            vel = (dx / dist * self.a.vel_auto, dy / dist * self.a.vel_auto, 0.0)
        self.d.offboard = dict(pos=(qd['p'][0], qd['p'][1], qd['z']), vel=vel, yaw=qd['yaw'])
        return qd

    def _disegna(self, scr, lev, quad):
        d = self.d
        scr.erase()
        lp, b, s = d.get('lpos'), d.get('batt'), d.get('salute')
        righe = [
            f'{self.titolo}   [recinto {self.raggio} m, quota max {self.quota_max} m]',
            '',
            f'PX4: {d.nav():8s} {"ARMATO" if d.armato() else "disarmato":9s} '
            f'{"in aria" if d.in_aria() else "a terra"}',
            f'quota {self.quota():5.2f} m   distanza dal decollo {self.dist():4.2f} m   '
            f'batteria {b.voltage_v:5.2f} V ({b.voltage_v / max(b.cell_count, 1):4.2f} V/cella) '
            f'{100 * b.remaining:3.0f}%' if b and lp else
            f'quota {self.quota():5.2f} m   distanza {self.dist():4.2f} m',
            f'FAST-LIO {d.f["lio"].hz():4.1f} Hz (eta\' {d.f["lio"].eta():4.2f} s)  agganciati '
            f'{s.data[1]:.0f}' if s is not None and len(s.data) > 1 else
            f'FAST-LIO {d.f["lio"].hz():4.1f} Hz',
            # misure (cambiano se muovi il drone a mano) e comandi (cambiano solo coi tasti):
            # il 21 ottobre la riga dei comandi, con "quota -1.00", sembrava una misura bloccata
            ('assetto misurato da PX4: rollio {:+4.0f}  beccheggio {:+4.0f}  imbardata {:+4.0f} '
             'gradi'.format(*eulero_ned(tuple(d.get('att').q))) if d.get('att') is not None else
             'assetto misurato da PX4: non arriva'),
            f'comandi inviati (levette virtuali, -1..+1): rollio {lev[0]:+.2f}  beccheggio '
            f'{lev[1]:+.2f}  imbardata {lev[2]:+.2f}  gas {lev[3]:+.2f}'
            + ('   joystick' if self.js and self.js.vivo else ''),
            '',
            'SPAZIO = EMERGENZA (motori spenti, il drone cade)   l = atterra   h = fermo',
            'frecce = sposta   w/s = su/giu\'   a/d = ruota   t = decolla   q = esci (a terra)'
            + ('   o = quadrato' if self.quadrato else ''),
            '',
            f'ALLARME: {self.allarme}' if self.allarme else '',
            '' if quad is None else f'quadrato: tratto {min(quad["i"] + 1, 4)} di 4',
        ] + self.eventi
        h, w = scr.getmaxyx()
        for i, r in enumerate(righe[:h - 1]):
            try:
                scr.addstr(i, 0, r[:w - 1], curses.A_BOLD if r.startswith(('ALLARME', 'SPAZIO'))
                           else 0)
            except curses.error:
                pass
        scr.refresh()


def fase_volo(d, reg, a, n):
    conf = {5: dict(raggio=0.7, quota_max=0.9, quota_decollo=0.5,
                    titolo='FASE 5: hover TRATTENUTO (drone legato, eliche montate)'),
            6: dict(raggio=a.raggio, quota_max=a.quota_max, quota_decollo=1.0,
                    titolo='FASE 6: volo manuale'),
            7: dict(raggio=a.raggio, quota_max=a.quota_max, quota_decollo=1.0,
                    titolo='FASE 7: quadrato automatico (o per partire)', quadrato=True)}[n]
    stampa(reg, f'== {conf["titolo"]} ==')
    if n == 5:
        stampa(reg, 'Lega il drone a terra (corda corta, 30-50 cm di gioco) o fallo volare '
                    'dentro una rete. Nessuno entro 3 m.')
    if n == 7:
        # 25 ottobre 2026: con le istruzioni generiche il quadrato non e' mai stato chiesto (o)
        stampa(reg, 'Istruzioni: t per decollare (sale da solo a 1 m). Quando compare "IN QUOTA: '
                    'premi o", premi o: il drone fa DA SOLO un quadrato di '
                    f'{a.lato} m (avanti, destra, indietro, sinistra) e torna fermo sul punto di '
                    'partenza. Poi l per atterrare. Il quadrato NON parte da solo. SPAZIO = emergenza.')
    else:
        stampa(reg, 'Istruzioni: t per decollare (sale da solo alla quota indicata), resta fermo '
                    'almeno 30 s, poi l per atterrare. SPAZIO = emergenza.')
    if not a.sim and chiedi(reg, 'Area libera, eliche montate, persone lontane, batteria '
                                 'carica. Procedo? [s/n]') != 's':
        return False, 'annullata'
    valida = lambda: d.get('lpos') is not None and d.get('lpos').xy_valid and d.get('lpos').z_valid
    if not valida():
        stampa(reg, 'Attendo che la posizione di PX4 (dalla stima di FAST-LIO) sia valida, fino a 30 s...')
    if not attendi(valida, 30):
        stampa(reg, 'Posizione di PX4 non valida: ripeti le fasi 2 e 3.')
        return False, 'posizione non valida'
    return Volo(d, reg, a, **conf).avvia()


# ====================================================================== missione completa
# FASE 8 (novembre 2026; prima solo istruzioni): la missione di esplorazione vera,
# run_reale_exploration.sh, in uno spazio libero quadrato (di serie 5 x 5 m) col drone al
# centro, poi la mappa 3D colorata con le foto della webcam. Il collaudo prepara (recinto,
# misure del drone, webcam), ferma la propria stima e passa la mano allo script della missione,
# che fa i suoi controlli a terra, chiede il consenso (VOLA) e tiene in primo piano il
# sorvegliante (SPAZIO = emergenza, l = atterra).
# - Agente DDS e driver Livox restano quelli del collaudo (DAL_COLLAUDO=1): un agente riavviato
#   a volte non ritrova i topic di PX4 (sincronizzazione dell'ora, vedi avvia_collaudo.sh).
# - La stima invece riparte: FAST-LIO ha l'origine dove e' partito, e dopo la fase 3 (drone
#   spostato e ruotato a mano) e i voli delle fasi 5-7 non e' piu' il punto di decollo. I
#   controlli a terra della missione pretendono FAST-LIO nell'origine.
# - Recinto (BOX) = spazio libero meno MARGINE_SPAZIO per lato: il sorvegliante fa atterrare
#   0.5 m oltre il recinto, cioe' al bordo dello spazio. Pareti e ostacoli appena fuori dal
#   recinto restano nella mappa del pianificatore (larga 1.5 m oltre il recinto,
#   prepara_mondo.config_generica), che ne sta lontano.
# - Dopo la missione la stima del collaudo non c'e' piu': collaudo.py si chiude, e per altre
#   fasi si rilancia avvia_collaudo.sh.
SCRIPT_MISSIONE = os.path.join(os.path.dirname(QUI), 'run_reale_exploration.sh')
SCRIPT_RICOSTRUISCI = os.path.join(os.path.dirname(QUI), 'ricostruisci_mappa.py')
SCRIPT_WEBCAM = os.path.join(QUI, 'regola_webcam.py')
MISURE_FILE = os.path.join(QUI, 'collaudo_registri', 'misure_drone.json')
MARGINE_SPAZIO = 0.5    # m fra recinto della missione e bordo dello spazio libero
MARGINE_DECOLLO = 1.0   # m fra decollo e bordo del recinto (come prepara_reale.py)
LATO_MIN = 2.5          # m, lato minimo del recinto (come prepara_reale.py)
# Proposte della prima volta: i valori di serie di run_reale_exploration.sh. Vanno misurati.
MISURE_SERIE = {'partenza_z': 0.2, 'lidar_pos': '0,0,0.1', 'camera_pos': '0.1,0,0.05',
                'camera_giu': 0.0, 'camera_hfov': 50.0}


def scrivi(reg, testo=''):
    """Come stampa(), ma se il terminale non c'e' piu' (sessione SSH caduta durante la
    missione) si scrive solo il registro invece di fermarsi."""
    try:
        print(testo, flush=True)
    except OSError:
        pass
    if testo:
        reg.scrivi(testo)


def chiedi_valore(reg, domanda, proposto, limiti, vettore=False):
    """Numero, o x,y,z con vettore, con un valore proposto che INVIO tiene. limiti = (minimo,
    massimo) di ogni componente."""
    while True:
        r = input(f'{domanda} [INVIO = {proposto}]: ').strip()
        reg.scrivi(f'{domanda} -> {r or proposto}')
        try:
            v = [float(x) for x in (r or str(proposto)).replace(' ', '').split(',')]
        except ValueError:
            v = []
        if len(v) == (3 if vettore else 1) and all(limiti[0] <= x <= limiti[1] for x in v):
            return ','.join(f'{x:g}' for x in v) if vettore else v[0]
        print(f'  valore non valido: {"tre numeri separati da virgole, " if vettore else ""}'
              f'ciascuno fra {limiti[0]:g} e {limiti[1]:g}')


def recinto(reg, a, lato=None, margine=MARGINE_SPAZIO, nome='dello spazio libero'):
    """Recinto della missione "xmin,ymin,xmax,ymax" in m dal decollo (x avanti, y sinistra),
    o None se non va bene per prepara_reale.py. lato: quadrato col drone al centro (di serie
    --area); margine: quanto il recinto sta dentro al bordo (fase 8: 0.5, fase 10: 0, il recinto
    E' l'area da esplorare)."""
    lato = a.area if lato is None else lato
    if a.box:
        try:
            xmin, ymin, xmax, ymax = [float(v) for v in a.box.split(',')]
        except ValueError:
            stampa(reg, f'--box "{a.box}" non e\' "xmin,ymin,xmax,ymax"')
            return None
    else:
        mezzo = lato / 2.0
        dist = {k: mezzo for k in ('davanti', 'dietro', 'sinistra', 'destra')}
        if not a.sim and chiedi(reg, f'Il drone e\' al CENTRO {nome} di {lato:g} x {lato:g} m '
                                     f'({mezzo:g} m da ogni bordo), col muso verso un lato? [s/n]') != 's':
            stampa(reg, f'Allora misura la distanza dal centro del drone al bordo {nome} '
                        '(parete, ostacolo o segno a terra) nelle quattro direzioni:')
            for k in dist:
                dist[k] = chiedi_valore(reg, f'  bordo {k.upper()} [m]', f'{mezzo:g}', (0.0, 40.0))
        xmin, xmax = -(dist['dietro'] - margine), dist['davanti'] - margine
        ymin, ymax = -(dist['destra'] - margine), dist['sinistra'] - margine
    for lo, hi, asse in ((xmin, xmax, 'x (dietro-davanti)'), (ymin, ymax, 'y (destra-sinistra)')):
        if lo > -MARGINE_DECOLLO or hi < MARGINE_DECOLLO or hi - lo < LATO_MIN:
            stampa(reg, f'Recinto lungo {asse} da {lo:g} a {hi:g} m: servono almeno {LATO_MIN:g} m '
                        f'di lato e {MARGINE_DECOLLO:g} m dal decollo a ogni bordo del recinto, cioe\' '
                        f'{MARGINE_DECOLLO + margine:g} m dal bordo {nome}.')
            return None
    return f'{xmin:g},{ymin:g},{xmax:g},{ymax:g}'


def misure_drone(reg):
    """Misure che servono a missione e mappa 3D, chieste con l'ultimo valore dato come proposta
    e salvate in MISURE_FILE (versionato: valgono per il drone, su ogni computer)."""
    m = dict(MISURE_SERIE)
    try:
        with open(MISURE_FILE) as f:
            m.update(json.load(f))
    except (OSError, ValueError):
        pass
    stampa(reg, 'Misure del drone (INVIO = valore proposto; restano in '
                'collaudo_registri/misure_drone.json per le volte successive).')
    stampa(reg, 'Punto di riferimento: il punto del drone da cui misuri tutto, per esempio il centro '
                'del Pixhawk. Assi del drone: x avanti (muso), y a sinistra, z in su.')
    m['partenza_z'] = chiedi_valore(reg, 'Quota da terra del punto di riferimento, drone appoggiato [m]',
                                    m['partenza_z'], (0.0, 1.0))
    m['lidar_pos'] = chiedi_valore(reg, 'Centro del Mid-360 dal punto di riferimento, x,y,z [m]',
                                   m['lidar_pos'], (-1.0, 1.0), vettore=True)
    m['camera_pos'] = chiedi_valore(reg, 'Obiettivo della webcam dal punto di riferimento, x,y,z [m]',
                                    m['camera_pos'], (-1.0, 1.0), vettore=True)
    m['camera_giu'] = chiedi_valore(reg, 'Inclinazione della webcam verso il basso [gradi, 0 = '
                                         'orizzontale]', m['camera_giu'], (-30.0, 90.0))
    stampa(reg, 'Campo di vista orizzontale della webcam: a distanza D da una parete segna dove cadono '
                'i bordi sinistro e destro dell\'immagine, distanti L fra loro: campo = '
                '2*atan(L/(2*D)) (per esempio L = 1.9 m a D = 2 m: 51 gradi).')
    m['camera_hfov'] = chiedi_valore(reg, 'Campo di vista orizzontale della webcam [gradi]',
                                     m['camera_hfov'], (20.0, 150.0))
    m['quando'] = time.strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(MISURE_FILE, 'w') as f:
            json.dump(m, f, indent=1)
    except OSError as e:
        stampa(reg, f'ATTENZIONE: misure non salvate ({e})')
    return m


def processi(filtro):
    """PID dei processi vivi (non zombie) per cui filtro(pid, gruppo, comando) e' vero."""
    trovati = []
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            with open(f'/proc/{p}/stat') as f:
                stat = f.read()
            with open(f'/proc/{p}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        campi = stat[stat.rfind(')') + 2:].split()     # stato, genitore, gruppo, ...
        if campi[0] != 'Z' and filtro(int(p), int(campi[2]), cmd):
            trovati.append(int(p))
    return trovati


def ferma_gruppo(pgid, attesa=15.0):
    """SIGTERM a tutto il gruppo di processi, poi SIGKILL: come ferma() di avvia_collaudo.sh."""
    for sig, t in ((signal.SIGTERM, attesa), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        except PermissionError:
            return False
        if attendi(lambda: not processi(lambda p, g, c: g == pgid), t, 0.2):
            return True
    return False


def ferma_simulazione(pid):
    """Lo script della simulazione del collaudo fa da se' la sua pulizia (PX4, Gazebo, nodi)."""
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    attendi(lambda: not processi(lambda p, g, c: p == pid), 60, 0.5)
    return attendi(lambda: not processi(lambda p, g, c: 'bin/px4' in c or 'gz sim' in c), 30, 0.5)


def misura_webcam(d, topic, durata=3.0):
    """Fotogrammi al secondo della webcam (contati senza decodificarli, come nella fase 1), None
    se nessuno pubblica il topic. Lo usa anche sorveglia_missione.py nei controlli a terra."""
    sub = None
    for _ in range(10):      # il topic compare dopo la scoperta DDS
        sub, _tipo = d.iscrivi_temporaneo(topic, 'cam')
        if sub is not None:
            break
        time.sleep(0.5)
    if sub is None:
        return None
    time.sleep(durata)
    hz = d.f['cam'].hz()
    d.destroy_subscription(sub)
    return hz


def leggi_riepilogo(percorso):
    """Righe chiave=valore che run_reale_exploration.sh scrive in RIEPILOGO a fine run."""
    try:
        with open(percorso) as f:
            return dict(r.rstrip('\n').split('=', 1) for r in f if '=' in r)
    except OSError:
        return None


def esito_missione(reg, r, rc, reale, fase=8):
    if r is None:
        scrivi(reg, f'Nessun riepilogo della missione (uscita {rc}): si e\' fermata prima di partire, '
                    'vedi /tmp/c2x_run_reale/run.log.')
        return False, f'missione non partita (uscita {rc})'
    scrivi(reg, '')
    scrivi(reg, f'== FASE {fase}: risultato della missione ==')
    esito = r.get('esito', '?')
    if r.get('missione_avviata') != '1':
        scrivi(reg, f'Missione non partita: {esito}. Il perche\' e\' nelle ultime righe qui sopra o in '
                    '/tmp/c2x_run_reale/run.log.')
        return False, f'missione non partita: {esito}'
    problemi = []
    # "atterrato" e basta: i suffissi dicono chi ha fatto atterrare (sorvegliante, PX4)
    if esito != 'atterrato':
        problemi.append(f'volo: {esito}' + (f' ({r["atterraggio_sorvegliante"]})'
                                            if r.get('atterraggio_sorvegliante') else ''))
    fine = r.get('fine_esplorazione', '')
    if not fine:
        problemi.append('esplorazione non conclusa')
    foto, colore, mappa = int(r.get('foto') or 0), int(r.get('colore') or 0), r.get('mappa3d', '')
    if not mappa:
        problemi.append('mappa 3D non generata')
    elif foto == 0:
        problemi.append('nessuna foto dalla telecamera: mappa 3D grigia')
    elif colore == 0:
        problemi.append('foto mai proiettate sulla mappa (misure della webcam?)')
    scrivi(reg, f'volo: {esito}; esplorazione: {fine or "non conclusa"}'
                + (' (tetto di tempo raggiunto, rientro con la mappa raccolta)' if fine == 'tempo massimo' else ''))
    scrivi(reg, f'mappa 3D: {mappa or "non generata"}')
    if mappa:
        cartella = os.path.dirname(mappa)
        scrivi(reg, f'  {foto} foto, {colore}% della superficie colorata; anteprime in {cartella}/anteprima_*.png')
        if reale:
            scrivi(reg, '  Colori spostati rispetto alle forme? Si correggono le misure della webcam senza '
                        'rivolare, per esempio:')
            scrivi(reg, f'  python3 {SCRIPT_RICOSTRUISCI} {cartella} --cam_giu 10 --cam_hfov 48')
    if r.get('archivio'):
        scrivi(reg, f'archivio della run: {r["archivio"]}')
    if reale:
        scrivi(reg, 'Registro di volo di PX4 (.ulg): sulla scheda SD del Pixhawk.')
    if problemi:
        scrivi(reg, 'Da guardare: ' + '; '.join(problemi))
    scrivi(reg, 'La stima del collaudo e\' stata fermata per la missione: per altre fasi rilancia '
                'avvia_collaudo.sh.')
    return not problemi, (f'{esito}; esplorazione {fine or "non conclusa"}; mappa 3D '
                          f'{"si" if mappa else "no"}, {foto} foto, {colore}% colorato')


# ====================================================================== all'aperto
# FASE 9 (novembre 2026): FAST-LIO all'aperto, a piedi e motori spenti. L'8 novembre 2026 la
# prima missione su un prato con alberi e' atterrata da sola 1.8 s dopo il decollo: FAST-LIO
# agganciava ~340 punti in volo (al chiuso >= 1218) e lio_to_px4_bridge, con la soglia di 500
# tarata al chiuso e in simulazione, ha chiesto l'atterraggio. All'aperto non ci sono pareti che
# fermino il drone se la stima sbaglia, e senza radiocomando l'unica difesa e' sapere PRIMA che
# la stima in quel campo funziona. Col drone in mano si percorre il perimetro dell'area della
# missione e si torna al punto di partenza: la fase controlla che FAST-LIO veda davvero il
# percorso (lato dell'area misurato, chiusura del giro, frequenza), registra lidar e IMU per
# tarare FAST-LIO dopo, e ricava dai punti agganciati del giro la soglia di atterraggio per la
# fase 10 (collaudo_registri/aperto.json). Come la fase 3, ma all'aperto e su 100+ m.
FILE_APERTO = os.path.join(QUI, 'collaudo_registri', 'aperto.json')
CARTELLA_RISULTATI = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(QUI))), 'risultati_prova')
SOGLIA_APERTO_MIN, SOGLIA_APERTO_MAX = 150, 500   # punti agganciati, soglia per la fase 10
FRAZIONE_SOGLIA = 0.6         # soglia = 60% del 5o percentile del giro a piedi
VALIDITA_APERTO_H = 12.0      # ore: la prova a piedi vale per quel campo e quel giorno
PX4_VEL_APERTO = 1.2          # MPC_XY_VEL_MAX del profilo scheda_sd_aperto (1 m/s di missione)


def imbardata_lio(o):
    q = o.pose.pose.orientation
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def percentile(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))] if s else float('nan')


def fase9(d, reg, a):
    stampa(reg, '== FASE 9: FAST-LIO all\'aperto (giro a piedi, motori spenti) ==')
    if a.sim:
        return True, 'simulazione: niente giro a piedi (FAST-LIO sul lidar simulato)'
    if d.armato():
        return False, 'drone armato'
    config = os.path.basename(os.environ.get('FAST_LIO_CONFIG') or 'fast_lio_mid360.yaml')
    lato = a.area_aperto
    for r in [
        f'All\'aperto, nel campo della missione: area di {lato:g} x {lato:g} m segnata agli angoli, '
        'drone appoggiato al CENTRO (il punto di decollo) col muso verso un lato. Segna a terra '
        'posizione e direzione del muso: alla fine il drone torna li\' uguale.',
        f'FAST-LIO con {config}' + ('' if 'aperto' in config else
                                     ' (quella da interno; all\'aperto prova FAST_LIO_CONFIG='
                                     'fast_lio_mid360_aperto.yaml ./avvia_collaudo.sh reale ...)') + '.',
        'Durante il giro: drone a ~1 m da terra, dritto, muso in avanti; passo normale; nessuno '
        'davanti al lidar. Si registrano lidar e IMU per tarare FAST-LIO dopo.',
    ]:
        stampa(reg, r)
    if chiedi(reg, 'Pronto, drone appoggiato sul segno al centro dell\'area? [s/n]') != 's':
        return False, 'annullata'
    if not attendi(lambda: d.get('lio') is not None and d.f['lio'].eta() < 0.5, 10):
        return False, 'nessuna stima di FAST-LIO'
    o0 = d.get('lio')
    p0 = o0.pose.pose.position
    p0 = (p0.x, p0.y, p0.z)
    yaw0 = imbardata_lio(o0)
    # Registrazione di lidar, IMU e stima: fuori da git (risultati_prova ignora .mcap/.db3).
    bag = os.path.join(CARTELLA_RISULTATI, time.strftime('%Y-%m-%d_%H%M') + '_giro_a_piedi')
    reg_bag = None
    try:
        os.makedirs(CARTELLA_RISULTATI, exist_ok=True)
        reg_bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag, a.lidar, a.imu, a.lio,
                                    a.salute], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        stampa(reg, f'registrazione in {bag}')
    except OSError as e:
        stampa(reg, f'ATTENZIONE: registrazione non avviata ({e}): la prova vale lo stesso')
    campioni, agganciati, l1, hz = [], [], [], []
    fine = threading.Event()

    def raccogli():
        ultimo_sal = None
        t_hz = time.monotonic()
        while not fine.is_set():
            o = d.get('lio')
            if o is not None:
                p = o.pose.pose.position
                campioni.append((time.monotonic(), p.x, p.y, p.z))
            s = d.get('salute')
            if s is not None and s is not ultimo_sal and len(s.data) >= 3:
                ultimo_sal = s
                agganciati.append(s.data[1])
                l1.append(s.data[2])
            if time.monotonic() - t_hz > 1.0:
                t_hz = time.monotonic()
                hz.append(d.f['lio'].hz())
            time.sleep(0.05)
    input(f'\nSolleva il drone a ~1 m e premi INVIO quando parti. Poi vai al bordo dell\'area, '
          f'percorri TUTTO il perimetro ({4 * lato:g} m) e torna al centro...')
    th = threading.Thread(target=raccogli, daemon=True)
    th.start()
    reg.scrivi('giro a piedi iniziato')
    input('\nAppoggia il drone sul segno al centro, col muso come alla partenza, e premi INVIO...')
    time.sleep(2.0)                      # la stima si assesta a drone fermo
    fine.set()
    th.join(timeout=2.0)
    if reg_bag is not None:
        try:
            os.killpg(reg_bag.pid, signal.SIGINT)
            reg_bag.wait(timeout=15)
        except (OSError, subprocess.TimeoutExpired):
            reg_bag.kill()
    o1 = d.get('lio')
    if o1 is None or len(campioni) < 20:
        return False, 'stima persa durante il giro'
    p1 = o1.pose.pose.position
    # Percorso e dimensioni nel riferimento della partenza (x avanti, y sinistra), come il recinto.
    c, s = math.cos(math.radians(yaw0)), math.sin(math.radians(yaw0))
    loc = [((x - p0[0]) * c + (y - p0[1]) * s, -(x - p0[0]) * s + (y - p0[1]) * c) for _, x, y, _ in campioni]
    percorso, prec = 0.0, loc[0]
    for q in loc[1:]:
        dq = math.dist(q, prec)
        if dq > 0.05:                   # niente tremolio a drone fermo
            percorso += dq
            prec = q
    ex = max(q[0] for q in loc) - min(q[0] for q in loc)
    ey = max(q[1] for q in loc) - min(q[1] for q in loc)
    err_xy = math.hypot(p1.x - p0[0], p1.y - p0[1])
    err_z = abs(p1.z - p0[2])
    err_yaw = abs(diff_angolo(imbardata_lio(o1), yaw0))
    durata = campioni[-1][0] - campioni[0][0]
    hz_min = min(hz[2:]) if len(hz) > 3 else (min(hz) if hz else 0.0)
    ag_min, ag_p5, ag_med = min(agganciati or [0]), percentile(agganciati, 5), percentile(agganciati, 50)
    l1_min = min(l1 or [float('nan')])
    soglia = int(round(FRAZIONE_SOGLIA * ag_p5 / 10.0) * 10) if agganciati else 0
    soglia = max(SOGLIA_APERTO_MIN, min(SOGLIA_APERTO_MAX, soglia))
    toll_lato = 0.15 * lato
    toll_xy = max(0.5, 0.005 * percorso)
    for r in [
        f'giro di {durata:.0f} s: percorso stimato {percorso:.0f} m (atteso ~{4 * lato + lato:.0f} m), '
        f'area percorsa {ex:.1f} x {ey:.1f} m (attesa {lato:g} x {lato:g})',
        f'chiusura del giro: {100 * err_xy:.0f} cm in orizzontale (massimo {100 * toll_xy:.0f}), '
        f'{100 * err_z:.0f} cm in quota, {err_yaw:.1f} gradi di imbardata',
        f'FAST-LIO: almeno {hz_min:.1f} Hz; punti agganciati minimo {ag_min:.0f}, 5o percentile '
        f'{ag_p5:.0f}, mediana {ag_med:.0f}; l1 minimo {l1_min:.0f}',
    ]:
        stampa(reg, r)
    problemi = []
    if abs(ex - lato) > toll_lato or abs(ey - lato) > toll_lato:
        problemi.append(f'area percorsa {ex:.1f} x {ey:.1f} m invece di {lato:g} x {lato:g}: o il '
                        'giro non ha seguito il perimetro, o FAST-LIO non vede tutto lo spostamento '
                        '(campo troppo vuoto?)')
    if err_xy > toll_xy:
        problemi.append(f'il giro non si chiude: {100 * err_xy:.0f} cm (deriva della stima)')
    if err_z > 0.20:
        problemi.append(f'la quota non torna: {100 * err_z:.0f} cm')
    if err_yaw > 5.0:
        problemi.append(f'l\'imbardata non torna: {err_yaw:.1f} gradi (o il muso non era come alla partenza)')
    if hz_min < 8.0:
        problemi.append(f'FAST-LIO scende a {hz_min:.1f} Hz (troppi punti per il Jetson?)')
    if ag_p5 < SOGLIA_APERTO_MIN / FRAZIONE_SOGLIA:
        problemi.append(f'troppo pochi punti agganciati (5o percentile {ag_p5:.0f}, servono almeno '
                        f'{SOGLIA_APERTO_MIN / FRAZIONE_SOGLIA:.0f}): prova fast_lio_mid360_aperto.yaml '
                        'o un campo con piu\' struttura entro 40 m')
    ok = not problemi
    for p in problemi:
        stampa(reg, 'PROBLEMA: ' + p)
    if ok:
        stampa(reg, f'Soglia di atterraggio per la fase 10: {soglia} punti agganciati (il '
                    f'{100 * FRAZIONE_SOGLIA:.0f}% del 5o percentile del giro).')
    esito = {'ok': ok, 'quando': time.strftime('%Y-%m-%d %H:%M:%S'), 'epoca': time.time(),
             'config': config, 'lato': lato, 'percorso_m': round(percorso, 1),
             'area_percorsa_m': [round(ex, 1), round(ey, 1)], 'errore_xy_m': round(err_xy, 3),
             'errore_z_m': round(err_z, 3), 'errore_imbardata_gradi': round(err_yaw, 1),
             'hz_min': round(hz_min, 1), 'agganciati_min': ag_min, 'agganciati_p5': ag_p5,
             'agganciati_mediana': ag_med, 'l1_min': l1_min, 'soglia': soglia,
             'registrazione': bag if reg_bag is not None else '', 'problemi': problemi}
    try:
        with open(FILE_APERTO, 'w') as f:
            json.dump(esito, f, indent=1)
    except OSError as e:
        stampa(reg, f'ATTENZIONE: {FILE_APERTO} non scritto ({e})')
    return ok, (f'percorso {percorso:.0f} m, area {ex:.0f}x{ey:.0f} m, chiusura {100 * err_xy:.0f} cm, '
                f'agganciati p5 {ag_p5:.0f}, soglia {soglia}, {config}')


def prova_a_piedi(reg):
    """Esito della fase 9 utilizzabile dalla fase 10 (stesso giorno, stessa configurazione di
    FAST-LIO di questa sessione), o None col motivo gia' scritto."""
    try:
        with open(FILE_APERTO) as f:
            e = json.load(f)
    except (OSError, ValueError):
        stampa(reg, 'Manca la prova a piedi (fase 9) all\'aperto: falla in questo campo prima di volare.')
        return None
    config = os.path.basename(os.environ.get('FAST_LIO_CONFIG') or 'fast_lio_mid360.yaml')
    eta_h = (time.time() - e.get('epoca', 0)) / 3600.0
    if not e.get('ok'):
        stampa(reg, 'L\'ultima prova a piedi (fase 9) non e\' superata: rifalla.')
    elif eta_h > VALIDITA_APERTO_H:
        stampa(reg, f'La prova a piedi (fase 9) e\' di {eta_h:.0f} ore fa: rifalla in questo campo, oggi.')
    elif e.get('config') != config:
        stampa(reg, f'La prova a piedi e\' stata fatta con {e.get("config")}, questa sessione usa '
                    f'{config}: rifalla (o riavvia con FAST_LIO_CONFIG={e.get("config")}).')
    else:
        stampa(reg, f'Prova a piedi delle {e["quando"][11:16]}: {e["percorso_m"]:.0f} m, chiusura '
                    f'{100 * e["errore_xy_m"]:.0f} cm, soglia di atterraggio {e["soglia"]} punti agganciati.')
        return e
    return None


def regola_webcam(reg, dev):
    """Esposizione della webcam per il sole (regola_webcam.py), a webcam libera; mai bloccante."""
    try:
        r = subprocess.run([sys.executable, SCRIPT_WEBCAM, dev], capture_output=True, text=True,
                           timeout=60)
        for riga in (r.stdout or '').splitlines():
            stampa(reg, riga)
        if r.returncode not in (0, 2):
            ultima = (r.stderr or '').strip().splitlines()[-1:] or ['']
            stampa(reg, f'[webcam] regolazione fallita (uscita {r.returncode}: {ultima[0]}): resta com\'e\'')
    except (OSError, subprocess.TimeoutExpired) as e:
        stampa(reg, f'[webcam] regolazione dell\'esposizione non riuscita ({e}): resta com\'e\'')


def fase8(d, reg, a):
    return missione(d, reg, a, aperto=False)


def fase10(d, reg, a):
    return missione(d, reg, a, aperto=True)


def missione(d, reg, a, aperto):
    """Fase 8 (missione completa nello spazio di 5 x 5 m) o fase 10 (all'aperto, 30 x 30 m a
    1 m/s, recinto virtuale): stessa sequenza, parametri diversi."""
    n = 10 if aperto else 8
    lato = a.area_aperto if aperto else a.area
    vel = a.vel_aperto if aperto else a.vel_missione
    tempo = a.tempo_aperto if aperto else a.tempo_missione
    stampa(reg, f'== FASE {n}: ' + (f'missione all\'aperto ({lato:g} x {lato:g} m, {vel:g} m/s)' if aperto
                                    else 'missione completa con mappa 3D (webcam)') + ' ==')
    if d.armato():
        return False, 'drone armato: prima atterra e disarma'
    if not os.access(SCRIPT_MISSIONE, os.X_OK):
        return False, f'script della missione non eseguibile: {SCRIPT_MISSIONE}'
    dove = 'l\'area' if aperto else 'lo spazio'
    for r in [
        f'Il drone DECOLLA DA SOLO a 1 m, gira su se stesso, esplora {dove} '
        f'a {vel:g} m/s per al massimo {tempo:.0f} s, torna sul punto di decollo e atterra.',
        'Poi la mappa 3D: superficie dal lidar, colori dalle foto della webcam (qualche minuto).',
        'Il collaudo ferma la sua stima e passa la mano a run_reale_exploration.sh: controlli a '
        'terra, consenso (scrivi VOLA), sorvegliante con SPAZIO = EMERGENZA e l = atterra.',
    ] + ([
        'All\'aperto: recinto virtuale (le direzioni senza ritorni del lidar liberano spazio, come un '
        'recinto vero), niente persone nell\'area e attorno, vento debole, il drone sempre in vista. '
        'Nessun radiocomando: se si allontana dove non deve, SPAZIO.',
        'Niente sotto 1 m vicino al drone (il Mid-360 non vede in basso vicino); alberi e ostacoli '
        'alti li evita il pianificatore.',
    ] if aperto else [
        'Spazio libero: niente sotto 1 m vicino al drone (il Mid-360 non vede in basso vicino), '
        'soffitto ad almeno 2.8 m.',
    ]):
        stampa(reg, r)
    if aperto:
        box = recinto(reg, a, lato, 0.0, 'dell\'area da esplorare')
    else:
        box = recinto(reg, a)
    if box is None:
        return False, 'recinto non valido'
    if aperto:
        stampa(reg, f'Recinto della missione BOX="{box}" (m dal decollo, x avanti, y sinistra): il '
                    f'sorvegliante fa atterrare a {a.margine_aperto:g} m oltre. Tutto lo spazio fino a '
                    f'{a.margine_aperto:g} m fuori dall\'area deve essere libero e senza persone.')
    else:
        stampa(reg, f'Recinto della missione BOX="{box}" (m dal decollo, x avanti, y sinistra): il '
                    f'sorvegliante fa atterrare a {MARGINE_SPAZIO:g} m oltre, al bordo dello spazio.')
    # VEL_MAX come float ("1.0", non "1"): entra nel nome dell'archivio (..._lio_v1.0), come
    # nelle run lanciate a mano e in simulazione.
    env = dict(os.environ, BOX=box, VEL_MAX=str(float(vel)), TEMPO_MAX=f'{tempo:g}',
               MAPPA_REALISTICA='1', FOTO='1')
    # Sempre col percorso completo: avvia_collaudo.sh esporta il solo nome del file.
    env['FAST_LIO_CONFIG'] = os.path.join(QUI, os.path.basename(
        os.environ.get('FAST_LIO_CONFIG') or 'fast_lio_mid360.yaml'))
    if aperto:
        # Mappa 3D: il lidar vede fino a 40 m, si tiene l'area piu' 5 m a celle da 5 cm; i punti
        # lontani sono radi, quindi superficie tenuta fino a 25 cm da un punto misurato.
        # Quota d'allarme 2.6 invece di 2.3: a 1 m/s nella prova generale il drone ha toccato
        # 2.21 m col riferimento a 2.0 (2026-11-14_1139), e all'aperto c'e' il vento.
        env.update(RECINTO_VIRTUALE='1', MARGINE_RECINTO=f'{a.margine_aperto:g}', VOXEL_MAPPA='0.05',
                   RITAGLIO_MAPPA='5', TAGLIO_MAPPA='0.25', QUOTA_ALLARME='2.6')
    if a.sim:
        cmd = [SCRIPT_MISSIONE, 'sim', os.environ.get('C2X_COLLAUDO_MONDO', 'cubicle_office')]
        if aperto:
            # Prova generale: nessun recinto vero aggiunto al mondo, PX4 col profilo all'aperto.
            env.update(SIM_RECINTO='0', SIM_PX4_VEL_MAX=f'{PX4_VEL_APERTO:g}')
    else:
        if aperto:
            prova = prova_a_piedi(reg)
            if prova is None:
                return False, 'prova a piedi (fase 9) mancante, vecchia o con altra configurazione'
            env['SALUTE_MIN_AGGANCIATI'] = str(prova['soglia'])
            if chiedi(reg, f'Sul Pixhawk c\'e\' il profilo all\'aperto (scheda_sd_aperto/etc/config.txt: '
                           f'MPC_XY_VEL_MAX {PX4_VEL_APERTO:g})? Senza, PX4 resta a 0.6 m/s. [s/n]') != 's':
                return False, 'profilo PX4 all\'aperto non caricato'
        m = misure_drone(reg)
        hz = misura_webcam(d, a.camera)
        stampa(reg, f'webcam {a.camera}: ' + ('NON PUBBLICATA' if hz is None else f'{hz:.1f} fotogrammi/s'))
        if hz is None or hz < 3.0:
            stampa(reg, 'La webcam non arriva: controlla il cavo USB e ls /dev/video* (CAMERA_DEV, di '
                        'serie /dev/video0), poi rilancia avvia_collaudo.sh. Senza, mappa 3D grigia.')
            return False, 'webcam non pubblicata' if hz is None else f'webcam a {hz:.1f} fotogrammi/s'
        env.update(DAL_COLLAUDO='1', PARTENZA_Z=f'{m["partenza_z"]:g}', LIDAR_POS=m['lidar_pos'],
                   CAMERA_DEV=os.environ.get('CAMERA_DEV', '/dev/video0'), CAMERA_POS=m['camera_pos'],
                   CAMERA_GIU=f'{m["camera_giu"]:g}', CAMERA_HFOV=f'{m["camera_hfov"]:g}')
        cmd = [SCRIPT_MISSIONE, os.environ.get('C2X_COLLAUDO_PORTA', '/dev/ttyACM0')]
        if chiedi(reg, 'Drone appoggiato nel punto di decollo col muso lungo x, eliche montate, spazio '
                       'sgombro, nessuno entro 3 m, batteria carica. Passo alla missione? [s/n]') != 's':
            return False, 'annullata'
    riepilogo = os.path.join(reg.dir, 'missione_riepilogo.txt')
    env['RIEPILOGO'] = riepilogo
    # Da qui la stima del collaudo non c'e' piu': finita la fase collaudo.py si chiude.
    a.esci = True
    if a.sim:
        pid = os.environ.get('C2X_COLLAUDO_SIM')
        if pid:
            stampa(reg, 'Fermo la simulazione del collaudo (la missione ne avvia una sua)...')
            if not ferma_simulazione(int(pid)):
                return False, 'simulazione del collaudo non fermata (PX4 o Gazebo ancora attivi)'
    else:
        pg = os.environ.get('C2X_COLLAUDO_STIMA')
        if pg:
            stampa(reg, 'Fermo la stima del collaudo (FAST-LIO, ponte verso PX4, webcam); agente DDS e '
                        'driver del lidar restano accesi per la missione...')
            if not ferma_gruppo(int(pg)):
                return False, 'stima del collaudo non fermata'
        else:
            stampa(reg, 'ATTENZIONE: collaudo.py non e\' stato avviato da avvia_collaudo.sh: ferma tu '
                        'la stima del collaudo (la missione non parte con un altro FAST-LIO acceso).')
        if aperto:
            # Webcam libera (la stima del collaudo e' ferma): esposizione per il sole.
            regola_webcam(reg, env['CAMERA_DEV'])
    chiavi = ('BOX', 'VEL_MAX', 'TEMPO_MAX', 'FOTO', 'RECINTO_VIRTUALE', 'MARGINE_RECINTO', 'QUOTA_ALLARME',
              'SIM_RECINTO', 'SIM_PX4_VEL_MAX', 'DAL_COLLAUDO', 'SALUTE_MIN_AGGANCIATI',
              'FAST_LIO_CONFIG', 'PARTENZA_Z', 'LIDAR_POS', 'CAMERA_DEV', 'CAMERA_POS', 'CAMERA_GIU',
              'CAMERA_HFOV', 'VOXEL_MAPPA', 'RITAGLIO_MAPPA', 'TAGLIO_MAPPA')
    solo_sim = ('BOX', 'VEL_MAX', 'TEMPO_MAX', 'FOTO', 'RECINTO_VIRTUALE', 'MARGINE_RECINTO', 'QUOTA_ALLARME',
                'SIM_RECINTO', 'SIM_PX4_VEL_MAX', 'VOXEL_MAPPA', 'RITAGLIO_MAPPA', 'TAGLIO_MAPPA')
    stampa(reg, 'Missione: ' + ' '.join(f'{k}="{env[k]}"' for k in chiavi if k in env and
                                         (not a.sim or k in solo_sim)) + ' ' + ' '.join(cmd))
    stampa(reg)
    # Segnali: Ctrl+C e caduta dell'SSH li gestiscono lo script e il sorvegliante (atterraggio,
    # poi spegnimento con la stima per ultima); qui si aspetta e basta. Gestore vuoto e non
    # SIG_IGN: un segnale ignorato resta ignorato anche nello script lanciato, e bash non
    # potrebbe piu' intercettarlo con trap.
    vecchi = {s: signal.signal(s, lambda *_: None) for s in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM)}
    try:
        sys.stdout.flush()      # le righe qui sopra prima di quelle dello script (uscita su file)
        rc = subprocess.call(cmd, env=env)
    except OSError as e:
        rc = f'non avviata: {e}'
    finally:
        for s, h in vecchi.items():
            signal.signal(s, h)
    return esito_missione(reg, leggi_riepilogo(riepilogo), rc, not a.sim, n)


FASI = {0: fase0, 1: fase1, 2: fase2, 3: fase3, 4: fase4,
        5: lambda d, r, a: fase_volo(d, r, a, 5), 6: lambda d, r, a: fase_volo(d, r, a, 6),
        7: lambda d, r, a: fase_volo(d, r, a, 7), 8: fase8, 9: fase9, 10: fase10}
NOMI = {0: 'collegamento con PX4', 1: 'sensori', 2: 'FAST-LIO a drone fermo',
        3: 'assi e stima dentro PX4', 4: 'motori SENZA eliche', 5: 'hover trattenuto',
        6: 'volo manuale', 7: 'quadrato automatico', 8: 'missione completa con mappa 3D (webcam)',
        9: 'FAST-LIO all\'aperto (giro a piedi)', 10: 'missione all\'aperto'}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--sim', action='store_true', help='contro la simulazione (topic e tempo)')
    ap.add_argument('--fase', type=int, default=None, help='fase da cui partire')
    ap.add_argument('--px4-ns', default='')
    ap.add_argument('--sysid', type=int, default=0, help='0 = quello annunciato da PX4')
    ap.add_argument('--lio', default=None, help='odometria di FAST-LIO')
    ap.add_argument('--salute', default=None, help='salute di FAST-LIO (punti agganciati)')
    ap.add_argument('--lidar', default=None)
    ap.add_argument('--imu', default=None)
    ap.add_argument('--camera', default=None)
    ap.add_argument('--raggio', type=float, default=2.0, help='recinto fasi 6-7 [m]')
    ap.add_argument('--quota-max', type=float, default=1.5, help='quota massima fasi 6-7 [m]')
    ap.add_argument('--stick', type=float, default=0.6,
                    help='levetta virtuale per tasto (0-1). PX4 applica zona morta (MPC_HOLD_DZ) e '
                         'curva expo (MPC_XY_MAN_EXPO): 0.6 con MPC_VEL_MANUAL 0.5 = ~0.16 m/s')
    ap.add_argument('--lato', type=float, default=1.0, help='lato del quadrato [m]')
    ap.add_argument('--vel-auto', type=float, default=0.3, help='velocita\' del quadrato [m/s]')
    ap.add_argument('--joystick', default='', help='es. /dev/input/js0 (vuoto = tastiera)')
    ap.add_argument('--mappa-joystick', default='{"rollio": 3, "beccheggio": 4, '
                                                '"imbardata": 0, "quota": 1}')
    ap.add_argument('--forza', action='store_true',
                    help='permette di saltare fasi non superate (resta nel registro)')
    ap.add_argument('--area', type=float, default=5.0,
                    help='fase 8: lato dello spazio libero quadrato, drone al centro [m]')
    ap.add_argument('--box', default='',
                    help='fase 8: recinto xmin,ymin,xmax,ymax in m dal decollo (x avanti, y '
                         'sinistra), al posto di --area')
    ap.add_argument('--vel-missione', type=float, default=0.3, help='fase 8: velocita\' massima [m/s]')
    ap.add_argument('--tempo-missione', type=float, default=240.0,
                    help='fase 8: tetto al tempo di esplorazione [s], poi rientro e atterraggio')
    ap.add_argument('--area-aperto', type=float, default=30.0,
                    help='fasi 9-10: lato dell\'area da esplorare all\'aperto, drone al centro [m]')
    ap.add_argument('--vel-aperto', type=float, default=1.0, help='fase 10: velocita\' massima [m/s]')
    ap.add_argument('--tempo-aperto', type=float, default=420.0,
                    help='fase 10: tetto al tempo di esplorazione [s]')
    ap.add_argument('--margine-aperto', type=float, default=2.0,
                    help='fase 10: il sorvegliante fa atterrare oltre l\'area + questo margine [m]')
    a = ap.parse_args()
    a.esci = False      # la fase 8 ferma la stima del collaudo: dopo, si esce
    pre = '/drone_1' if a.sim else ''
    a.lio = a.lio or (pre + '/lio/odometry' if a.sim else '/drone_1/lio/odometry')
    a.salute = a.salute or '/drone_1/lio/salute'
    a.lidar = a.lidar or ('/drone_1/lidar/points_lio' if a.sim else '/livox/lidar')
    a.imu = a.imu or ('/drone_1/lidar/imu' if a.sim else '/livox/imu')
    a.camera = a.camera or ('/drone_1/camera/image' if a.sim else '/camera/image_raw')

    rclpy.init()
    d = Drone(a)
    ex = SingleThreadedExecutor()
    ex.add_node(d)
    # Esecutore in un thread che si puo' fermare: con ex.spin() ancora attivo, rclpy.shutdown()
    # all'uscita chiudeva il processo con "terminate called without an active exception" e un
    # core dump (visto alla fine della fase 8, 7 novembre 2026; stesso rimedio di
    # sorveglia_missione.py).
    fine_spin = threading.Event()

    def gira():
        while not fine_spin.is_set():
            try:
                ex.spin_once(timeout_sec=0.1)
            except Exception:       # contesto chiuso durante l'uscita
                break
    spin = threading.Thread(target=gira, daemon=True)
    spin.start()
    reg = Registro(a.sim)
    stampa(reg, f'Collaudo C2-Explorer {"(SIMULAZIONE)" if a.sim else "(DRONE VERO)"} - '
                f'registro in {reg.dir}')
    try:
        while True:
            print('\nFasi:')
            for n in sorted(FASI):
                e = reg.esiti.get(str(n))
                stato = '   ' if e is None else (' OK' if e['ok'] else ' NO')
                print(f' [{stato}] {n}  {NOMI[n]}')
            scelta = a.fase if a.fase is not None else None
            a.fase = None
            if scelta is None:
                r = input('\nFase da eseguire (numero, INVIO = la prossima, x = esci): ').strip()
                if r == 'x':
                    break
                if r == '':
                    scelta = next((n for n in sorted(FASI)
                                   if not reg.esiti.get(str(n), {}).get('ok')), None)
                    if scelta is None:
                        print('Tutte superate.')
                        continue
                else:
                    try:
                        scelta = int(r)
                    except ValueError:
                        continue
            if scelta not in FASI:
                continue
            mancanti = [n for n in range(scelta) if not reg.esiti.get(str(n), {}).get('ok')]
            if mancanti and not a.forza:
                print(f'Prima vanno superate le fasi {mancanti} (oppure --forza).')
                continue
            if mancanti:
                reg.scrivi(f'ATTENZIONE: fase {scelta} con fasi non superate {mancanti} (--forza)')
            ok, det = FASI[scelta](d, reg, a)
            reg.esito(scelta, ok, det)
            try:      # dopo la missione il terminale puo' non esserci piu' (SSH caduta)
                print(f'\n>>> FASE {scelta}: {"SUPERATA" if ok else "NON SUPERATA"} - {det}')
            except OSError:
                pass
            if a.esci:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if d.in_aria():
            d.atterra()
            print('Ctrl+C in volo: atterraggio comandato')
            attendi(lambda: not d.armato(), 30)
        d.manual_attivo = False
        d.offboard = None
        fine_spin.set()
        spin.join(timeout=2.0)
        ex.shutdown()
        d.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
