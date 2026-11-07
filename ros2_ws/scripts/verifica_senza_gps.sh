#!/bin/bash
# Dice se l'ultima run e' volata DAVVERO senza GPS.
#
# Serve perche' la domanda non si puo' rispondere guardando Gazebo: il GPS
# simulato non sta in Gazebo, lo genera PX4 al proprio interno. E non si puo'
# rispondere nemmeno contando i campioni di vehicle_gps_position, perche' il
# driver simulato li pubblica comunque anche quando EKF2 li ignora - e' l'errore
# che ha fatto scambiare per "senza GPS" una run che il GPS lo usava.
#
# La prova vera e' se EKF2 ha CREATO una sorgente di aiuto GNSS: se non c'e'
# nemmeno il canale, il GPS non e' stato usato in nessun modo.
#
#   uso:  ./scripts/verifica_senza_gps.sh [registro.ulg]
#         senza argomenti prende il registro di volo piu' recente
set -u
ROOTFS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/PX4-Autopilot/build/px4_sitl_default/rootfs"

L="${1:-}"
if [ -z "$L" ]; then
  L=$(ls -t "$ROOTFS"/instance_*/log/*/*.ulg 2>/dev/null | head -1)
fi
if [ -z "$L" ] || [ ! -f "$L" ]; then
  echo "Nessun registro di volo trovato."
  echo "Se la run e' in corso va bene comunque: il file viene scritto mentre vola."
  exit 1
fi
echo "registro: $L"
echo "          ($(date -r "$L" '+%d/%m %H:%M'), $(du -h "$L" | cut -f1))"

python3 - "$L" <<'PY'
import sys
import numpy as np
from pyulog import ULog

u = ULog(sys.argv[1])
p = u.initial_parameters
esito = []

print("\n1) PARAMETRI DELL'AUTOPILOTA")
atteso = {'EKF2_GPS_CTRL': 0, 'EKF2_EV_CTRL': 15, 'EKF2_HGT_REF': 3}
ok_par = True
for k, a in atteso.items():
    v = p.get(k)
    buono = (v is not None and int(v) == a)
    ok_par &= buono
    print(f"   {k:15s} = {v}" + ("" if buono else f"   <-- atteso {a}"))
esito.append(("parametri di volo cieco", ok_par))

print("\n2) SORGENTI DI AIUTO CREATE DA EKF2  (la prova decisiva)")
src = sorted(d.name.replace('estimator_aid_src_', '') for d in u.data_list if 'aid_src' in d.name)
gnss = [s for s in src if 'gnss' in s or 'gps' in s]
print(f"   {src}")
print(f"   sorgenti GNSS: {gnss if gnss else 'NESSUNA'}")
esito.append(("nessuna fusione GNSS", not gnss))
ev = [s for s in src if s.startswith('ev_')]
esito.append(("visione usata come riferimento", len(ev) >= 2))

print("\n3) COSA IL FILTRO STAVA USANDO")
d = [x for x in u.data_list if x.name == 'estimator_status_flags']
if d:
    f = d[0].data
    for k in ('cs_gps', 'cs_ev_pos', 'cs_ev_vel', 'cs_ev_yaw', 'cs_ev_hgt', 'cs_fake_pos'):
        if k in f:
            print(f"   {k:13s} attivo {100*(np.array(f[k]) > 0).mean():5.1f}% del tempo")
    if 'cs_fake_pos' in f:
        # fake_pos e' la misura fittizia che EKF2 usa quando resta SENZA
        # riferimenti: se e' accesa, il volo cieco non stava funzionando.
        esito.append(("mai rimasto senza riferimenti",
                      (np.array(f['cs_fake_pos']) > 0).mean() < 0.05))

print("\n4) IL GPS SIMULATO PUBBLICAVA? (non e' un problema, ma va saputo)")
for n in ('vehicle_gps_position', 'sensor_gps'):
    dd = [x for x in u.data_list if x.name == n]
    if dd:
        print(f"   {n}: {len(dd[0].data['timestamp'])} campioni, IGNORATI da EKF2")

# Il volo c'e' stato davvero? Senza questo, "zero GPS" non significa nulla:
# anche un drone che non decolla non usa il GPS.
print("\n5) IL DRONE HA VOLATO DAVVERO?")
def g(n):
    r = [x for x in u.data_list if x.name == n]
    return r[0].data if r else None
st, gp = g('vehicle_status'), g('vehicle_local_position_groundtruth')
volato = False
if st and gp:
    arm = np.array(st['arming_state'])
    if (arm == 2).any():
        z = -np.array(gp['z'])
        t = (np.array(gp['timestamp'], dtype=float) - gp['timestamp'][0]) / 1e6
        volato = z.max() > 0.3
        print(f"   armato: si    quota max {z.max():.2f} m    durata {t[-1]:.0f} s")
    else:
        print("   MAI ARMATO - la run non e' decollata")
esito.append(("ha volato", volato))

print("\n" + "=" * 52)
tutto = all(v for _, v in esito)
for nome, v in esito:
    print(f"   [{'si' if v else 'NO'}] {nome}")
print("=" * 52)
print("VERDETTO: volo senza GPS CONFERMATO" if tutto
      else "VERDETTO: NON confermato - vedi le voci con NO")
PY
