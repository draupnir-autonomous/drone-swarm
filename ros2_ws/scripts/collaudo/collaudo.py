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
  8  Verso la missione completa  cosa manca e come provarla (nessun volo da qui)

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
Registro di ogni sessione in collaudo_registri/<data_ora>_<sim|reale>/ accanto a questo file;
quali fasi sono superate in collaudo_registri/esiti_sim.json ed esiti_reale.json (separati).
"""
import argparse
import curses
import json
import math
import os
import signal
import struct
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
        super().__init__('collaudo_c2x', parameter_overrides=[
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


def fase8(d, reg, a):
    stampa(reg, '== FASE 8: verso la missione completa ==')
    for r in [
        'Le fasi 0-7 provano tutto cio\' che la missione usa sotto il pianificatore: stima di',
        'FAST-LIO dentro PX4, comando manuale, offboard. La missione di esplorazione si avvia',
        'con ros2_ws/scripts/run_reale_exploration.sh (recinto BOX, 0.3 m/s, sorvegliante con',
        'SPAZIO = emergenza e l = atterra); prima la prova generale con "... sim" a casa.',
        'Il campo del Mid-360 vero (-7/+52 gradi) e\' diverso dal simulato (+-50): ostacoli',
        'bassi vicini restano invisibili. Cosa resta da provare: LEGGIMI principale, "Missione',
        'di esplorazione sul drone vero".',
    ]:
        stampa(reg, r)
    return True, 'solo istruzioni'


FASI = {0: fase0, 1: fase1, 2: fase2, 3: fase3, 4: fase4,
        5: lambda d, r, a: fase_volo(d, r, a, 5), 6: lambda d, r, a: fase_volo(d, r, a, 6),
        7: lambda d, r, a: fase_volo(d, r, a, 7), 8: fase8}
NOMI = {0: 'collegamento con PX4', 1: 'sensori', 2: 'FAST-LIO a drone fermo',
        3: 'assi e stima dentro PX4', 4: 'motori SENZA eliche', 5: 'hover trattenuto',
        6: 'volo manuale', 7: 'quadrato automatico', 8: 'verso la missione completa'}


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
    a = ap.parse_args()
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
    threading.Thread(target=ex.spin, daemon=True).start()
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
            print(f'\n>>> FASE {scelta}: {"SUPERATA" if ok else "NON SUPERATA"} - {det}')
    except KeyboardInterrupt:
        pass
    finally:
        if d.in_aria():
            d.atterra()
            print('Ctrl+C in volo: atterraggio comandato')
            attendi(lambda: not d.armato(), 30)
        d.manual_attivo = False
        d.offboard = None
        ex.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
