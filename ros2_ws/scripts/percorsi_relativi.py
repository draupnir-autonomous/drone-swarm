#!/usr/bin/env python3
"""Rende RELATIVI i collegamenti simbolici di ros2_ws/install che puntano dentro il
progetto (ros2_ws/build, ros2_ws/src, ros2_ws/install).

colcon build --symlink-install li crea con percorsi ASSOLUTI: spostato o copiato il
progetto, puntano ancora alla cartella vecchia (o a niente) e lo stack usa file di
un'altra copia senza dirlo. Quelli verso il sistema (/opt/ros/...) restano assoluti.
Una build successiva li ricrea assoluti: per questo lo richiama preflight.sh prima
di ogni run (costa meno di un secondo).

Uso: percorsi_relativi.py [ros2_ws]     (default: la cartella sopra questo script)
"""
import os
import sys


def main():
    ws = os.path.realpath(sys.argv[1] if len(sys.argv) > 1 else
                          os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    install = os.path.join(ws, 'install')
    rifatti = 0
    for radice, cartelle, file in os.walk(install):
        for n in cartelle + file:
            link = os.path.join(radice, n)
            if not os.path.islink(link):
                continue
            dest = os.readlink(link)
            if not os.path.isabs(dest) or '/ros2_ws/' not in dest:
                continue
            resto = dest.split('/ros2_ws/', 1)[1]
            if resto.split('/', 1)[0] not in ('build', 'src', 'install'):
                continue
            nuovo = os.path.relpath(os.path.join(ws, resto), os.path.dirname(link))
            tmp = link + '.c2x_tmp'
            os.symlink(nuovo, tmp)
            os.replace(tmp, link)
            rifatti += 1
    print(f'collegamenti resi relativi: {rifatti}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
