#!/usr/bin/env python3
"""Pixhawk sull'USB del Jetson: MAVLink o DDS? (C2-Explorer, collaudo, ottobre 2026)

  px4_usb_dds.py controlla /dev/ttyACM0          esce con 0 se sulla porta c'e' MAVLink
                                                 (SYS_USB_AUTO non e' 0: il DDS resta fermo)
  px4_usb_dds.py imposta /dev/ttyACM0 COPIA      copia tutti i parametri in COPIA (formato
                                                 QGroundControl), SYS_USB_AUTO = 0, riavvio
  px4_usb_dds.py firmware /dev/ttyACM0           aspetta che sulla porta ci sia PX4 e non il
                                                 bootloader; se il bootloader resta, lo fa
                                                 partire (comando di avvio dell'uploader)

Il bootloader resta fermo se riceve dati nei primi secondi dopo l'accensione: il 19 ottobre
un agente DDS che aveva riaperto subito la porta lo ha bloccato per minuti. Per questo
l'agente va avviato solo con PX4 gia' partito.

Con SYS_USB_AUTO = 2 (di serie) PX4 mette MAVLink sull'USB e il client DDS avviato da
/etc/extras.txt resta "disconnected": e' quello che e' successo alla prima prova del
19 ottobre 2026. Dopo SYS_USB_AUTO = 0 QGroundControl non si collega piu' via USB: per
tornare indietro, /etc/config.txt sulla scheda SD (PROCEDURA_PROVE_REALI.md, punto A.3).
"""
import glob
import os
import struct
import sys
import time

from pymavlink import mavutil


def collega(porta, attesa):
    m = mavutil.mavlink_connection(porta, baud=921600, source_system=254)
    hb = m.wait_heartbeat(timeout=attesa)
    return m, hb


def intero_in_float(v):
    # PX4 manda e riceve i parametri interi come i bit dell'intero nel campo float
    return struct.unpack('<f', struct.pack('<i', v))[0]


def float_in_intero(v):
    return struct.unpack('<i', struct.pack('<f', v))[0]


def imposta(porta, copia):
    m, hb = collega(porta, 10)
    if hb is None:
        sys.exit('Nessun MAVLink dal Pixhawk: niente da fare')
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        sys.exit('Pixhawk ARMATO: non tocco niente')
    m.mav.param_request_list_send(m.target_system, m.target_component)
    par, tot, inizio, ultimo = {}, None, time.time(), time.time()
    while time.time() - ultimo < 3 and time.time() - inizio < 120:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if r is None:
            continue
        ultimo, tot = time.time(), r.param_count
        par[r.param_id] = (r.param_value, r.param_type)
        if len(par) >= tot:
            break
    if not tot or len(par) < tot:
        sys.exit(f'Letti solo {len(par)} parametri su {tot}: niente modifica')
    with open(copia, 'w') as f:
        f.write(f'# Parametri del Pixhawk prima di SYS_USB_AUTO=0 ({time.strftime("%Y-%m-%d %H:%M")})\n')
        f.write('# Vehicle-Id\tComponent-Id\tName\tValue\tType\n')
        for n in sorted(par):
            v, tipo = par[n]
            if tipo == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
                f.write(f'1\t1\t{n}\t{v:.18g}\t9\n')
            else:
                f.write(f'1\t1\t{n}\t{float_in_intero(v)}\t6\n')
    print(f'Copia dei {len(par)} parametri in {copia}')
    m.mav.param_set_send(m.target_system, m.target_component, b'SYS_USB_AUTO',
                         intero_in_float(0), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    t = time.time()
    while time.time() - t < 5:
        r = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if r is not None and r.param_id == 'SYS_USB_AUTO':
            if float_in_intero(r.param_value) != 0:
                break
            print('SYS_USB_AUTO = 0, riavvio il Pixhawk')
            time.sleep(1.5)   # PX4 salva i parametri sulla scheda
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                    0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(1)
            return
    sys.exit('SYS_USB_AUTO non confermato da PX4: niente riavvio')


def nome_usb(porta):
    """Nome USB del dispositivo dietro la porta (PX4_FMU... o PX4_BL_FMU... nel bootloader)."""
    vero = os.path.realpath(porta)
    for n in glob.glob('/dev/serial/by-id/*'):
        if os.path.realpath(n) == vero:
            return os.path.basename(n)
    return ''


def firmware(porta, attesa=25):
    inizio, avviato = time.time(), False
    while time.time() - inizio < attesa:
        nome = nome_usb(porta) if os.path.exists(porta) else ''
        if nome and '_BL_' not in nome:
            return True
        # 8 s di bootloader sono troppi (di solito esce da solo in ~5 s): lo avvio io,
        # come fa px_uploader.py alla fine (GET_SYNC, poi REBOOT; la memoria non cambia)
        if '_BL_' in nome and not avviato and time.time() - inizio > 8:
            import serial
            try:
                with serial.Serial(porta, 115200, timeout=0.5) as s:
                    s.reset_input_buffer()
                    s.write(b'\x21\x20')
                    time.sleep(0.2)
                    if s.read(2) == b'\x12\x10':
                        print('Pixhawk fermo nel bootloader: lo faccio partire')
                        s.write(b'\x30\x20')
                        time.sleep(0.3)
            except Exception:
                pass   # la porta sparisce mentre il bootloader passa a PX4: normale
            avviato = True
        time.sleep(0.5)
    return False


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'firmware':
        sys.exit(0 if firmware(sys.argv[2]) else 1)
    if len(sys.argv) >= 3 and sys.argv[1] == 'controlla':
        _, hb = collega(sys.argv[2], 2.5)
        sys.exit(0 if hb is not None else 1)
    if len(sys.argv) >= 4 and sys.argv[1] == 'imposta':
        imposta(sys.argv[2], sys.argv[3])
        sys.exit(0)
    print(__doc__)
    sys.exit(2)
