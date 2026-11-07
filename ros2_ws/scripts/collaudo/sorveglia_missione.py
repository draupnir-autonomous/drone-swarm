#!/usr/bin/env python3
"""Sorvegliante della missione di esplorazione sul drone VERO (C2-Explorer, ottobre 2026).

Il drone non ha radiocomando: durante la missione autonoma l'unico modo di intervenire e'
questo programma, che scripts/run_reale_exploration.sh tiene in primo piano nel terminale.
Parla con PX4 come collaudo.py (ne usa la classe Drone: stessi topic, stessi comandi).

  controlli   controlli a terra prima del decollo; esce 0 solo se e' tutto a posto
  volo        sorveglianza della missione: tasti e sicurezze automatiche; esce a drone
              atterrato e disarmato (o se la missione finisce senza decollo)
  atterra     se il drone e' armato comanda l'atterraggio e aspetta il disarmo; lo usa la
              pulizia dello script prima di spegnere la stima

In volo:   SPAZIO = EMERGENZA (motori spenti subito, il drone CADE)    l = atterra
           q = esci (solo a drone disarmato)
Atterraggio automatico se: stima di FAST-LIO ferma da 1 s, meno di 300 punti agganciati,
posizione di PX4 non valida, drone fuori dal recinto (BOX + margine) o sopra la quota
massima, batteria sotto il 15%, esplorazione non partita entro un minuto dal decollo, volo
oltre il tempo massimo, stack della missione o della stima terminati. SIGHUP (sessione SSH
caduta), SIGTERM e Ctrl+C = atterraggio; il programma resta attivo fino al disarmo.
"""
import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

QUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, QUI)
import collaudo as C  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, Empty  # noqa: E402
from px4_msgs.msg import TrajectorySetpoint  # noqa: E402

NS = '/drone_1'
# Carica minima (frazione di PX4) per partire e per restare in volo: 15% come BAT_LOW_THR di PX4
# e come collaudo.py (25 ottobre 2026, prima 50% alla partenza e 25% in volo).
BATTERIA_MIN = 0.15


class Uscita:
    """Righe col tempo a terminale e nel registro. Se il terminale non c'e' piu' (sessione
    SSH caduta) si continua a scrivere solo nel registro: un errore di scrittura non deve
    fermare il programma con il drone in aria."""
    def __init__(self, percorso):
        self.f = open(percorso, 'a') if percorso else None

    def __call__(self, testo=''):
        riga = time.strftime('%H:%M:%S ') + testo if testo else ''
        if self.f:
            self.f.write(riga + '\n')
            self.f.flush()
        try:
            print(riga, flush=True)
        except OSError:
            pass


class Tastiera:
    """Un tasto alla volta senza INVIO (terminale in modalita' cbreak), senza bloccare."""
    def __init__(self, attiva):
        self.fd = None
        self.vecchio = None
        if attiva and sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.vecchio = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def leggi(self):
        if self.fd is None:
            return None
        try:
            if select.select([self.fd], [], [], 0)[0]:
                c = os.read(self.fd, 1)
                if not c:                 # terminale chiuso
                    self.chiudi()
                    return None
                return c.decode(errors='ignore')
        except OSError:
            self.chiudi()
        return None

    def chiudi(self):
        if self.fd is not None and self.vecchio is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.vecchio)
            except (OSError, termios.error):
                pass
        self.fd = None


def avvia_drone():
    a = argparse.Namespace(sim=False, px4_ns='', lio=NS + '/lio/odometry', salute=NS + '/lio/salute',
                           sysid=0)
    rclpy.init()
    d = C.Drone(a)
    d.pronto = False
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    # Il ponte dei comandi annuncia qui la fine di decollo e giro iniziale.
    d.create_subscription(Bool, NS + '/volo_pronto',
                          lambda m: setattr(d, 'pronto', d.pronto or bool(m.data)), qos)
    # ATTERRAGGIO SENZA CONTESA (prova generale del 19 ottobre 2026). Il ponte dei comandi
    # pubblica i suoi setpoint su /fmu/in/trajectory_setpoint, che in PX4 e' lo stesso
    # topic uORB in cui scrive il compito di atterraggio: con un NAV_LAND mandato da qui e
    # il ponte ancora attivo, il controllore riceveva a turno "scendi" e "resta a 0.8 m". Il
    # drone e' arrivato a terra ma PX4 non ha riconosciuto l'atterraggio per 60 s (motori
    # accesi), finche' il ponte non ha smesso. Si chiede quindi l'atterraggio al ponte, su
    # land_request come fa lio_to_px4_bridge: il ponte smette di pubblicare e manda lui
    # NAV_LAND. Qui si contano i setpoint che arrivano ancora a PX4.
    d.pub_atterra = d.create_publisher(Empty, NS + '/land_request', 1)
    d.f_traj = C.Frequenza()
    d.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                          lambda m: d.f_traj.tick(), d.qos_sens)
    ex = SingleThreadedExecutor()
    ex.add_node(d)
    fine = threading.Event()

    def gira():
        while not fine.is_set():
            try:
                ex.spin_once(timeout_sec=0.1)
            except Exception:       # contesto chiuso durante l'uscita
                break
    t = threading.Thread(target=gira, daemon=True)
    t.start()

    def chiudi():
        # Prima si ferma il thread dell'esecutore, poi si distrugge il resto: con ex.spin()
        # ancora attivo, rclpy.shutdown() faceva terminare il processo con un core dump
        # ("terminate called without an active exception").
        fine.set()
        t.join(timeout=2.0)
        ex.shutdown()
        d.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return d, chiudi


def vivo(pid):
    if not pid:
        return True
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def posizione_mondo(d, partenza_z):
    """Posizione del drone come la vede il pianificatore: FAST-LIO + punto di partenza."""
    o = d.get('lio')
    if o is None:
        return None
    p = o.pose.pose.position
    return p.x, p.y, partenza_z + p.z


# ====================================================================== controlli a terra
def controlli(d, a, out):
    problemi = []
    out('== Controlli a terra prima della missione ==')
    # Il collaudo sul drone vero viene prima: la missione autonoma presuppone che stima,
    # comando manuale, motori, hover e offboard siano gia' stati provati (fasi 0-7).
    esiti_file = os.path.join(QUI, 'collaudo_registri', 'esiti_reale.json')
    try:
        esiti = json.load(open(esiti_file))
    except (OSError, ValueError):
        esiti = {}
    mancanti = [n for n in range(8) if not esiti.get(str(n), {}).get('ok')]
    if mancanti:
        msg = f'collaudo sul drone vero: fasi non superate {mancanti} ({esiti_file})'
        if a.senza_collaudo:
            out(f'ATTENZIONE: {msg} - ignorato (SENZA_COLLAUDO=1)')
        else:
            problemi.append(msg + ': prima scripts/collaudo/avvia_collaudo.sh reale')
    else:
        out('collaudo sul drone vero: fasi 0-7 superate')

    out('Attendo PX4 (fino a 15 s)...')
    if not C.attendi(lambda: d.get('status') is not None, 15):
        problemi.append('nessun messaggio da PX4 (agente DDS, cavo USB, extras.txt sulla scheda SD)')
        return problemi
    s = d.get('status')
    out(f'PX4: sistema {s.system_id}, modalita\' {d.nav()}, '
        f'{"ARMATO" if d.armato() else "disarmato"}, controlli pre-volo di PX4 '
        f'{"OK" if s.pre_flight_checks_pass else "NO (in POSCTL senza levette e\' normale)"}')
    if d.armato():
        problemi.append('il drone e\' gia\' ARMATO')
    land = d.get('land')
    if land is not None and not land.landed:
        problemi.append('PX4 non vede il drone appoggiato a terra')

    out('Attendo la stima di FAST-LIO (fino a 10 s)...')
    if not C.attendi(lambda: d.get('lio') is not None, 10):
        problemi.append(f'nessuna stima su {NS}/lio/odometry: FAST-LIO non gira o non riceve il lidar')
        return problemi
    out(f'Misuro per {a.durata:.0f} s a drone fermo (nessuno lo tocchi)...')
    pos, ag = [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.durata:
        p = d.get('lio').pose.pose.position
        pos.append((p.x, p.y, p.z))
        sal = d.get('salute')
        if sal is not None and len(sal.data) >= 2:
            ag.append(sal.data[1])
        time.sleep(0.1)
    hz = d.f['lio'].hz()
    escurs = max(math.dist(pos[0], q) for q in pos)
    ag_min = min(ag) if ag else float('nan')
    out(f'FAST-LIO: {hz:.1f} Hz, escursione da fermo {100 * escurs:.1f} cm, punti agganciati '
        f'minimi {ag_min:.0f}')
    if hz < 8.0:
        problemi.append(f'stima a {hz:.1f} Hz (servono almeno 8)')
    if escurs > 0.05:
        problemi.append(f'la stima si muove di {100 * escurs:.1f} cm a drone fermo (massimo 5)')
    if not ag or ag_min < 500:
        problemi.append(f'FAST-LIO aggancia {ag_min:.0f} punti (servono almeno 500: piu\' '
                        'struttura vicina, vedi LEGGIMI_COLLAUDO)')
    o = d.get('lio')
    p = o.pose.pose.position
    if abs(p.z) > 0.10 or math.hypot(p.x, p.y) > 0.10:
        problemi.append(f'FAST-LIO non e\' nell\'origine ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}): il drone e\' '
                        'stato mosso dopo l\'avvio della stima? Riavvia lo script col drone nel punto '
                        'di decollo')

    lp = d.get('lpos')
    ekf = d.get('ekf')
    if lp is None or not lp.xy_valid or not lp.z_valid:
        problemi.append('posizione di PX4 non valida (EKF2 non usa la visione?)')
    else:
        ned, _ = C.lio_in_ned(o)
        scarto = math.dist(ned, (lp.x, lp.y, lp.z))
        out(f'PX4 contro FAST-LIO: scarto {100 * scarto:.0f} cm')
        if scarto > 0.20:
            problemi.append(f'PX4 e FAST-LIO distano {100 * scarto:.0f} cm (EKF2_EV_POS_*, '
                            'assi del lidar: fase 3 del collaudo)')
    if ekf is not None and not (ekf.cs_ev_pos and ekf.cs_ev_yaw):
        problemi.append('EKF2 non fonde posizione e imbardata della visione (EKF2_EV_CTRL = 15, '
                        'EKF2_GPS_CTRL = 0)')
    b = d.get('batt')
    if b is not None and b.remaining >= 0:
        out(f'batteria {b.voltage_v:.2f} V, {100 * b.remaining:.0f}%')
        if b.remaining < BATTERIA_MIN:
            problemi.append(f'batteria al {100 * b.remaining:.0f}% (serve almeno il '
                            f'{100 * BATTERIA_MIN:.0f}%)')
    else:
        out('ATTENZIONE: nessuna lettura della batteria da PX4')
    fs = d.get('fs')
    if fs is not None:
        attivi = [k for k, t in fs.get_fields_and_field_types().items()
                  if t == 'boolean' and getattr(fs, k)]
        out('condizioni di failsafe attive in PX4: ' + (', '.join(attivi) or 'nessuna'))
    return problemi


# ====================================================================== volo
class Sorveglianza:
    def __init__(self, d, a, out):
        self.d, self.a, self.out = d, a, out
        self.atterraggio = None       # motivo, quando comandato
        self.t_atterraggio = 0.0
        self.t_px4_comanda = None     # da quando PX4 e' in LAND/RTL senza che l'abbia chiesto qui
        self.ponte_avvisato = 0.0
        self.emergenza = False
        self.volato = False
        self.t0 = time.monotonic()
        self.t_armato = None
        self.t_pronto = None
        self.t_disarmo = None
        self.missione_fermata = False
        self.ultima_riga = 0.0
        self.max_quota = 0.0
        self.max_fuori = -1e9
        self.partita = False
        self.senza_frontiere = False
        self.log_pos = 0
        self.log_coda = ''
        self.log_letto = 0.0

    def ferma_missione(self, perche):
        """Spegne lo stack della missione (ponte dei comandi compreso). Serve quando il decollo
        non e' concluso: il ponte riarma un drone disarmato finche' non ha volato (vedi tick()
        in position_command_bridge.cpp), e dopo un atterraggio comandato da qui ripartirebbe."""
        if self.missione_fermata or not self.a.pgid_missione:
            return
        self.missione_fermata = True
        self.out(f'spengo lo stack della missione ({perche})')
        try:
            os.killpg(self.a.pgid_missione, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    def atterra(self, motivo):
        if self.atterraggio is not None or self.emergenza:
            return
        self.atterraggio = motivo
        self.t_atterraggio = time.monotonic()
        self.out(f'ATTERRAGGIO: {motivo}')
        self.d.pub_atterra.publish(Empty())   # il ponte smette di pubblicare e chiede LAND
        self.d.atterra()                      # NAV_LAND diretto, se il ponte non risponde
        if not self.d.pronto:
            self.ferma_missione('decollo non concluso')

    def emergenza_motori(self):
        self.emergenza = True
        self.out('EMERGENZA: motori spenti')
        for _ in range(5):
            self.d.disarma(forza=True)
            time.sleep(0.04)
        self.ferma_missione('emergenza')

    def esplorazione_partita(self):
        """L'FSM ha ricevuto il via (lo dice il registro della missione, come in
        run_multi_exploration.sh). Lettura incrementale, al massimo una volta al secondo."""
        ora = time.monotonic()
        if self.partita or not self.a.log_esplorazione or ora - self.log_letto < 1.0:
            return self.partita
        self.log_letto = ora
        try:
            with open(self.a.log_esplorazione, errors='ignore') as f:
                f.seek(self.log_pos)
                nuovo = f.read()
                self.log_pos = f.tell()
        except OSError:
            return False
        testo = self.log_coda + nuovo
        self.log_coda = testo[-64:]
        # FINISH al via = nessuna frontiera dalla prima scansione: in quello stato l'FSM non
        # fa nulla (ne' rientro ne' atterraggio) e il drone resterebbe in hovering.
        self.senza_frontiere = 'WAIT_TRIGGER to FINISH' in testo
        self.partita = 'WAIT_TRIGGER to PLAN_TRAJ' in testo or self.senza_frontiere
        return self.partita

    def sicurezze(self, ora):
        d, a = self.d, self.a
        # Il ponte dei comandi puo' armare un secondo dopo l'avvio di questo programma, prima
        # che le sue iscrizioni ricevano qualcosa (prova generale del 19 ottobre 2026: stima
        # "ferma da inf s", atterraggio subito dopo l'armamento). Si aspettano i primi dati,
        # ma non oltre 3 s: dopo, la mancanza e' un guasto vero.
        if d.get('lio') is None or d.get('lpos') is None:
            if ora - self.t0 > 3.0:
                return 'nessun dato da FAST-LIO o da PX4 al sorvegliante'
            return None
        # 1.0 s come odom_timeout del ponte dei comandi con FAST-LIO: buchi di 0.5-0.6 s sono
        # normale variabilita' sotto carico (0.62 s misurati all'avvio della missione nella
        # prova generale; con 0.5 s il sorvegliante faceva atterrare un drone sano).
        if d.f['lio'].eta() > a.stima_ferma:
            return f'stima di FAST-LIO ferma da {d.f["lio"].eta():.1f} s'
        s = d.get('salute')
        if s is not None and len(s.data) >= 2 and s.data[1] < 300 and d.f['salute'].eta() < 1:
            return f'FAST-LIO aggancia solo {s.data[1]:.0f} punti'
        lp = d.get('lpos')
        if lp is None or not lp.xy_valid or not lp.z_valid:
            return 'posizione di PX4 non valida'
        p = posizione_mondo(d, a.partenza_z)
        if p is not None:
            xmin, ymin, xmax, ymax = a.box
            fuori = max(xmin - p[0], p[0] - xmax, ymin - p[1], p[1] - ymax)
            self.max_fuori = max(self.max_fuori, fuori)
            self.max_quota = max(self.max_quota, p[2])
            if fuori > a.margine:
                return f'fuori dal recinto di {fuori:.2f} m (x {p[0]:.2f}, y {p[1]:.2f})'
            if p[2] > a.quota_max:
                return f'troppo alto: {p[2]:.2f} m > {a.quota_max:.2f} m'
        b = d.get('batt')
        if b is not None and 0 <= b.remaining < BATTERIA_MIN:
            return f'batteria al {100 * b.remaining:.0f}%'
        if not vivo(a.pid_stima):
            return 'stack della stima terminato'
        if not vivo(a.pgid_missione) and not self.missione_fermata:
            return 'stack della missione terminato (nodo critico?)'
        if self.t_pronto is not None and ora - self.t_pronto > a.attesa_via \
                and not self.esplorazione_partita():
            return f'esplorazione non partita {a.attesa_via:.0f} s dopo il decollo'
        if self.esplorazione_partita() and self.senza_frontiere:
            return 'nessuna frontiera al via (FSM in FINISH): niente da esplorare nel recinto?'
        if self.t_armato is not None and ora - self.t_armato > a.tempo_volo_max:
            return f'volo oltre il tempo massimo ({a.tempo_volo_max:.0f} s)'
        return None

    def controlla_failsafe_px4(self, ora, armato):
        """PX4 passato da solo in LAND/DESCEND/RTL (failsafe: batteria critica, stima persa,
        ...) con il ponte dei comandi ancora attivo: la stessa contesa dei setpoint
        dell'atterraggio comandato da qui (vedi avvia_drone). Si chiede al ponte di fermarsi
        (land_request: smette di pubblicare e chiede LAND, cioe' atterraggio sul posto anche
        al posto di un RTL, che al chiuso non evita gli ostacoli); se insiste, si spegne."""
        d = self.d
        if not armato or self.atterraggio is not None or self.emergenza or self.t_pronto is None \
                or d.nav() not in ('LAND', 'DESCEND', 'RTL'):
            self.t_px4_comanda = None
            return
        if self.t_px4_comanda is None:
            self.t_px4_comanda = ora
        # 0.5 s di tolleranza: quando l'atterraggio lo chiede il ponte (fine missione) i suoi
        # ultimi setpoint precedono di poco il cambio di modalita'.
        if ora - self.t_px4_comanda < 0.5 or d.f_traj.eta() > 0.2:
            return
        if not self.ponte_avvisato:
            self.ponte_avvisato = ora
            self.out(f'PX4 in {d.nav()} da solo (failsafe?) con il ponte dei comandi ancora attivo: '
                     'gli chiedo di fermarsi (land_request)')
            d.pub_atterra.publish(Empty())
        elif ora - self.ponte_avvisato > 1.0 and not self.missione_fermata:
            self.ferma_missione(f'PX4 in {d.nav()} e il ponte manda ancora setpoint')

    def riga_stato(self, ora):
        d = self.d
        p = posizione_mondo(d, self.a.partenza_z)
        s = d.get('salute')
        b = d.get('batt')
        pos = f'x {p[0]:5.2f} y {p[1]:5.2f} z {p[2]:4.2f}' if p else 'posizione ?'
        ag = f'{s.data[1]:.0f}' if s is not None and len(s.data) >= 2 else '?'
        bat = f'{100 * b.remaining:.0f}%' if b is not None and b.remaining >= 0 else '?'
        fase = ('emergenza' if self.emergenza else 'atterraggio' if self.atterraggio else
                'esplorazione' if self.esplorazione_partita() else
                'decollo/giro' if d.armato() else 'a terra')
        self.out(f'[{ora - self.t0:5.0f} s] {d.nav():8s} {"ARMATO" if d.armato() else "disarm."} '
                 f'{fase:12s} {pos}  LIO {d.f["lio"].hz():4.1f} Hz ag {ag}  batt {bat}   '
                 f'(SPAZIO emergenza, l atterra)')

    def ciclo(self, tastiera):
        d, a = self.d, self.a
        while True:
            ora = time.monotonic()
            try:
                c = tastiera.leggi()
                if c == ' ':
                    self.emergenza_motori()
                elif c in ('l', 'L'):
                    self.atterra('comando dell\'operatore')
                elif c in ('q', 'Q'):
                    if d.armato():
                        self.out('q: il drone e\' armato, prima atterra (l) o emergenza (SPAZIO)')
                    else:
                        self.out('uscita a drone disarmato')
                        return 'uscita dell\'operatore'
                armato = d.armato()
                if armato and self.t_armato is None:
                    self.t_armato = ora
                    self.out('drone ARMATO dal ponte dei comandi: decollo')
                if d.in_aria():
                    self.volato = True
                if d.pronto and self.t_pronto is None:
                    self.t_pronto = ora
                    self.out('decollo e giro iniziale conclusi (volo_pronto)')
                if armato and self.atterraggio is None and not self.emergenza:
                    motivo = self.sicurezze(ora)
                    if motivo:
                        self.atterra(motivo)
                if self.atterraggio is not None and armato and not self.emergenza \
                        and d.nav() not in ('LAND', 'DESCEND', 'TERMINATION'):
                    d.atterra()       # ripetuto finche' PX4 non e' in LAND
                self.controlla_failsafe_px4(ora, armato)
                if self.atterraggio is not None and armato and not self.missione_fermata \
                        and ora - self.t_atterraggio > 1.0 and d.f_traj.eta() < 0.3:
                    self.ferma_missione('il ponte dei comandi manda ancora setpoint durante '
                                        'l\'atterraggio')
                if not armato:
                    if self.t_disarmo is None:
                        self.t_disarmo = ora
                    if self.emergenza:
                        return 'emergenza'
                    if self.t_armato is not None and ora - self.t_disarmo > 1.0:
                        if d.pronto:
                            # Decollo concluso: il ponte non riarma piu' (ha_volato_).
                            self.out('drone a terra e disarmato')
                            return 'atterrato'
                        self.ferma_missione('disarmato prima della fine del decollo')
                        return 'disarmato prima della fine del decollo'
                    if not vivo(a.pgid_missione) or self.missione_fermata:
                        self.out('stack della missione terminato senza decollo')
                        return 'missione terminata a terra'
                    if self.t_armato is None and ora - self.t0 > a.attesa_armamento:
                        self.out(f'il ponte dei comandi non ha armato il drone in {a.attesa_armamento:.0f} s')
                        self.ferma_missione('nessun armamento')
                        return 'nessun armamento'
                else:
                    self.t_disarmo = None
                if ora - self.ultima_riga > 2.0:
                    self.ultima_riga = ora
                    self.riga_stato(ora)
                time.sleep(0.05)
            except KeyboardInterrupt:
                if d.armato():
                    self.atterra('Ctrl+C')
                    self.out('atterraggio in corso: resto attivo fino al disarmo (SPAZIO = emergenza)')
                else:
                    self.ferma_missione('Ctrl+C a terra')
                    return 'Ctrl+C a terra'


def volo(d, a, out):
    sv = Sorveglianza(d, a, out)

    def perso(signum, _frame):
        # SIGHUP: sessione SSH caduta; SIGTERM: chiusura dello script. Con il drone armato
        # si atterra e si resta attivi fino al disarmo.
        nome = 'SIGHUP (sessione persa)' if signum == signal.SIGHUP else 'SIGTERM'
        if d.armato():
            sv.atterra(nome)
        else:
            sv.ferma_missione(nome)
    signal.signal(signal.SIGHUP, perso)
    signal.signal(signal.SIGTERM, perso)
    tastiera = Tastiera(not a.senza_tastiera)
    if tastiera.fd is None and not a.senza_tastiera:
        out('ERRORE: serve un terminale vero per i tasti di emergenza')
        sv.ferma_missione('nessun terminale')
        return 'nessun terminale', sv
    out('Sorveglianza attiva. SPAZIO = EMERGENZA (motori spenti, il drone cade), l = atterra, '
        'q = esci (a terra)')
    try:
        esito = sv.ciclo(tastiera)
    finally:
        tastiera.chiudi()
    out(f'fine sorveglianza: {esito}; {"in volo" if sv.volato else "mai staccato da terra"}, '
        f'quota massima {sv.max_quota:.2f} m, uscita massima dal recinto {max(sv.max_fuori, 0.0):.2f} m'
        + (f', atterraggio comandato: {sv.atterraggio}' if sv.atterraggio else ''))
    return esito, sv


def atterra(d, a, out):
    if not C.attendi(lambda: d.get('status') is not None, 5):
        out('nessun messaggio da PX4: non so se il drone e\' armato')
        return 1
    if not d.armato():
        return 0
    out('drone ARMATO: comando l\'atterraggio e aspetto il disarmo prima di spegnere la stima')
    d.pub_atterra.publish(Empty())
    t0 = time.monotonic()
    while d.armato() and time.monotonic() - t0 < a.attesa:
        if d.nav() not in ('LAND', 'DESCEND', 'TERMINATION'):
            d.atterra()
        time.sleep(0.5)
    if d.armato():
        out(f'ATTENZIONE: drone ancora armato dopo {a.attesa:.0f} s')
        return 1
    out('drone disarmato')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('modo', choices=['controlli', 'volo', 'atterra'])
    ap.add_argument('--registro', default='', help='file in cui scrivere anche le righe a terminale')
    ap.add_argument('--durata', type=float, default=8.0, help='controlli: misura a drone fermo [s]')
    ap.add_argument('--senza-collaudo', action='store_true',
                    help='controlli: non pretendere le fasi 0-7 del collaudo sul drone vero')
    ap.add_argument('--box', default='', help='volo: recinto xmin,ymin,xmax,ymax [m]')
    ap.add_argument('--margine', type=float, default=0.5, help='volo: tolleranza oltre il recinto [m]')
    ap.add_argument('--quota-max', type=float, default=2.3, help='volo: quota massima [m da terra]')
    ap.add_argument('--partenza-z', type=float, default=0.0,
                    help='quota del punto di riferimento del drone appoggiato (spawn_z) [m]')
    ap.add_argument('--pgid-missione', type=int, default=0, help='gruppo di processi dello stack della missione')
    ap.add_argument('--pid-stima', type=int, default=0, help='processo dello stack della stima')
    ap.add_argument('--log-esplorazione', default='', help='registro dello stack della missione')
    ap.add_argument('--stima-ferma', type=float, default=1.0,
                    help='volo: atterra se la stima di FAST-LIO non arriva da tanti s')
    ap.add_argument('--attesa-armamento', type=float, default=90.0)
    ap.add_argument('--attesa-via', type=float, default=60.0,
                    help='volo: atterra se l\'esplorazione non parte entro tanti s dal decollo')
    ap.add_argument('--tempo-volo-max', type=float, default=600.0)
    ap.add_argument('--attesa', type=float, default=90.0, help='atterra: attesa del disarmo [s]')
    ap.add_argument('--senza-tastiera', action='store_true', help='solo per prove in simulazione')
    a = ap.parse_args()
    if a.modo == 'volo':
        try:
            a.box = [float(v) for v in a.box.split(',')]
            assert len(a.box) == 4
        except (ValueError, AssertionError):
            sys.exit('volo: --box xmin,ymin,xmax,ymax')
    out = Uscita(a.registro)
    d, chiudi = avvia_drone()
    try:
        if a.modo == 'controlli':
            problemi = controlli(d, a, out)
            if problemi:
                out('CONTROLLI NON SUPERATI:')
                for p in problemi:
                    out('  - ' + p)
                return 1
            out('controlli superati')
            return 0
        if a.modo == 'atterra':
            return atterra(d, a, out)
        esito, _ = volo(d, a, out)
        return 0 if esito == 'atterrato' else 2
    finally:
        chiudi()


if __name__ == '__main__':
    sys.exit(main())
