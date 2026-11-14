---
name: jetson-gpu-non-si-accende
description: "8 nov 2026: dopo il riavvio delle 11:13 la GPU dell'Orin non si accende (nvgpu ACR bootstrap failed); Vulkan/Open3D/Gazebo in crash; NON e' la sandbox di Bash; come riconoscerlo"
metadata:
  node_type: memory
  type: project
  originSessionId: d45e099c-5f29-4d43-b964-05063554539f
  modified: 2026-11-08T10:25:22.416Z
---

L'8 nov 2026, avvio delle 11:13:59, la GPU della Jetson non si e' accesa: il kernel ripete
`nvgpu: 17000000.gpu ... invalid mem acr_falcon2_sysmem_desc` / `ACR bootstrap failed` /
`Failed initialization for: g->ops.acr.acr_construct_execute` (114 volte in 9 minuti),
`/dev/nvgpu/` e' vuota. Effetti: Open3D OffscreenRenderer (Filament/Vulkan) stampa
`libnvrm_gpu.so: NvRmGpuLibOpen failed, error=4` e va in panic in `createVkInstance` (core dump,
non un'eccezione Python: un try/except non lo ferma); con DISPLAY impostato invece RuntimeError
`enumeratePhysicalDevices: ErrorInitializationFailed`. Nei due avvii precedenti della stessa
mattina (10:16 e 10:43, voli reali) zero errori ACR e Vulkan funzionante; gli aggiornamenti apt
del 7 nov non toccano pacchetti nvidia-l4t. Causa non trovata; rimedio presunto: riavvio pulito
(da confermare).

Falsa pista da non ripetere: il primo sospetto e' stato la sandbox del Bash di Claude (nodi GPU
mancanti, /dev/nvmap in sola lettura), ma fuori sandbox era identico: /dev/nvmap e' `cr--r-----
root video` anche normalmente e /dev/nvgpu era vuota perche' il driver non aveva finito di
accendere la GPU. Il journal di questa Jetson mescola le date (l'orologio salta all'avvio): per
collocare gli eventi usare `journalctl -o short-monotonic` e `journalctl --list-boots`.

**Why:** con la GPU spenta falliscono le anteprime di ricostruisci_mappa.py, Gazebo (sensori
gpu_lidar) e CUDA; il volo vero no (FAST-LIO, PX4, esplorazione e Poisson girano sulla CPU).
**How to apply:** se Open3D/Gazebo/Vulkan vanno in crash, prima di toccare il codice controllare
`journalctl -k -b 0 | grep -c "ACR bootstrap failed"`; se e' > 0 proporre all'utente un riavvio.
Vedi [[jetson-port-status]] (stranezze di Open3D sulla Jetson) e [[collaudo-fase8-missione-webcam]].
