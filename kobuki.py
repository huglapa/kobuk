import numpy as np
from math import pi, sin, cos
from matplotlib import pyplot as plt
from Kobuki_project.classes_meca import Vecteur3d


class Kobuki(object):
    """Robot Mobile"""

    def __init__(self, rayon=0.075, distance=0.35, pos=Vecteur3d(), ori=0, const=1, nom='tortue', c='green'):

        self.pos = [pos]
        self.dist = distance    # dist entre les roues
        self.r = rayon  # rayon des roues
        self.vit_t = 0  # vitesse tout droit
        self.vit_r = 0
        self.ori = [ori]     # orientation

        a = self.r/2
        b = self.r/self.dist
        self.jacobian = np.array([[a, a], [b, -b]])

        self.const = const  # constante pour utiliser lors des randoms
        self.nom = nom
        self.color = c

    def __str__(self):
        msg = 'Kobuki(' + str(self.pos[-1]) + ',' + ')'
        return msg

    def __repr__(self):
        msg = 'Kobuki(' + str(self.pos[-1]) + ',' + ')'
        return msg

    def mcd(self, vg=0, vd=0):
        """Modèle cinématique direct : prend comme entrée les vitesses de rotation des roues droite et gauche
        et calcule les vitesse de translation et rotation de la plateforme"""
        vit_roues = np.array((vd, vg))
        vit_robot = np.dot(self.jacobian, vit_roues)

        return vit_robot

    def mci(self, vit_trans=0, vit_rot=0):
        """Modèle cinématique inverse : prend comme entrée les vitesse de translation et rotation de la plateforme
        et calcule les vitesses de rotation des roues droite et gauche"""
        vit_robot = np.array((vit_trans, vit_rot))
        vit_roues = np.dot(np.linalg.inv(self.jacobian), vit_robot)

        return vit_roues

    def simulMCD(self, dt, vg, vd):
        """Calcul de la position suivante après un pas de temps en fonction des entrées du mcd"""
        vit_rob = self.mcd(vg, vd)
        self.ori.append(self.ori[-1]+dt*vit_rob[1])

        dx = self.pos[-1].x+dt*vit_rob[0]*cos(self.ori[-1])
        dy = self.pos[-1].y+dt*vit_rob[0]*sin(self.ori[-1])
        self.pos.append(Vecteur3d(dx, dy))

    def simulMCI(self, dt, vt, vr):
        """Calcul de la position suivante après un pas de temps en fonction des entrées du mci"""
        vit_roues = self.mci(vt, vr)
        vit_rob = np.array((vt, vr))
        self.ori.append(self.ori[-1]+dt*vit_rob[1])

        dx = self.pos[-1].x+dt*vit_rob[0]*cos(self.ori[-1])
        dy = self.pos[-1].y+dt*vit_rob[0]*sin(self.ori[-1])
        self.pos.append(Vecteur3d(dx, dy))

    def trajectoire(self):
        """plot de la trajectoire du robot"""
        trajx = []
        trajy = []
        for i in range(0, len(self.pos)):
            trajx.append(self.pos[i].x)
            trajy.append(self.pos[i].y)
        plt.plot(trajx, trajy)  # color=self.color)
        plt.show()
