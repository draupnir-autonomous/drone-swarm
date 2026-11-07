# Librerie di terze parti modificate

Dal 7 novembre 2026 queste librerie sono copiate DENTRO il repo (con le modifiche già applicate),
non più come repository git annidati. Le patch qui accanto sono solo documentazione: dicono cosa
è stato cambiato rispetto all'originale, nel caso si voglia aggiornare la libreria.

| cartella | origine | versione | patch |
|---|---|---|---|
| `ros2_ws/src/terze_parti/Sophus` | https://github.com/strasdat/Sophus | 1.22.10 | `Sophus-1.22.10.patch` |
| `ros2_ws/src/terze_parti/rpg_vikit` | https://github.com/Robotic-Developer-Road/rpg_vikit | 4b7abc8 (+ `vikit_py/COLCON_IGNORE`) | `rpg_vikit-4b7abc8.patch` |
| `ros2_ws/src/fast_lio/include/ikd-Tree` | https://github.com/hku-mars/ikd-Tree (ramo fast_lio) | sottomodulo di FAST-LIO | nessuna |
| `terze_parti/Livox-SDK2` | https://github.com/Livox-SDK/Livox-SDK2 | c0796f0 (v1.3.1-3) | nessuna |

I loro `.git` originali sono in `_git_annidati_backup/` sulla Jetson (non versionati).
