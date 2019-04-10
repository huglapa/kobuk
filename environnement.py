from Kobuki_project.kobuki import Kobuki
from math import pi, sin, cos, sqrt
from matplotlib import pyplot as plt
import pygame
import pygame.draw as pygDraw
from pygame.time import Clock as pygClock
import numpy as np
from random import random, randint, seed


class Simulateur(object):
    robots = []

    def __init__(self, nom):
        self.nom = nom

    def addKobuki(self, K):
        """ajout d'un kobuki dans l'environnement"""
        self.robots.append(K)

    def rmKobuki(self, name):
        """retrait d'un kobuki de l'environnement"""
        for r in self.robots:
            if r.nom == name:
                self.robots.remove(r)
            else:
                print('No Kobuki named ' + name + ' in simulateur ' + self.nom + '.')

    def trace(self):
        """trace l'ensemble des positions de chaque robot de l'environnement"""
        plt.figure('Plan de ' + self.nom)
        for t in self.robots:
            X = []
            Y = []
            for i in t.pos:
                X.append(i.x)
                Y.append(i.y)
            plt.plot(X, Y, color=t.color, label=t.nom)
            liste_nom = ''
            for r in self.robots:
                liste_nom = liste_nom + ' + ' + r.nom
            print(liste_nom)
            plt.title('trajectoire de '+liste_nom)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.legend()
            plt.axis('equal')
            plt.grid(b=True)
        plt.show()

    def controlRoues(self, name, step, vg, vd):
        for r in self.robots:
            if r.nom == name:
                r.simulMCD(step, vg, vd)
            else:
                print('No Kobuki named ' + name + ' in simulateur ' + self.nom + '.')

    def controleur(self, name, x_des, y_des, kvit=0.02, kangle=0.09, step=0.02, duree=50):
        for r in self.robots:
            if r.nom == name:
                t=[0]
                while t[-1]<duree:
                    pos_err = np.array((r.pos[-1].x, r.pos[-1].y))-np.array((x_des, y_des))
                    theta_err = np.arctan2(y_des-r.pos[-1].y, x_des-r.pos[-1].x)
                    vtrans = kvit * np.linalg.norm(pos_err)
                    vrot = -kangle * theta_err
                    r.simulMCI(step, vtrans, vrot)
                    t.append(t[-1]+step)

    def trajSinMCD(self, name, step=0.01, duree=1, a=1, omega=1):
        """trajectoire de la forme A *sin(omega*t)"""
        t = [0]

        while t[-1] < duree:
            for r in self.robots:
                if r.nom == name:
                    vg = a * abs(sin(omega * t[-1]))
                    vd = a * abs(sin((pi/2*omega)+omega * t[-1]))
                    r.simulMCD(step, vg, vd)
            t.append(t[-1]+step)

    def trajSinMCI(self, name, step=0.01, duree=1, a=pi/2, omega=1):
        t = [0]
        while t[-1] < duree:
            for r in self.robots:
                if r.nom == name:
                    vr = a*sin(omega*t[-1])
                    vt = 1
                    r.simulMCI(step, vt, vr)
            t.append(t[-1]+step)

    def trajCirc(self, name, rayon=20, vrot=1, step=0.01, duree=1):
        """trajectoire circulaire de rayon donné à un vitesse donnée"""
        for r in self.robots:
            if r.nom == name:
                v_g = vrot*(rayon+r.dist/2)
                v_d = vrot*(rayon-r.dist/2)
                t = [0]
                while t[-1] < duree:
                    t.append(t[-1]+step)
                    r.simulMCD(step, v_g, v_d)

    def controlRouesRand(self, step, duree):
        t = [0]
        while t[-1] < duree:
            t.append(t[-1] + step)
            for r in self.robots:
                seed(a=None, version=2)
                g = randint(-2, 5)
                d = randint(-2, 5)
                self.controlRoues(r.nom, step, g, d)

    def goToPos(self, name, x, y, duree=1, step=0.01):
        """Déplace le robot à la position (x,y) du repère global."""
        for r in self.robots:
            if r.nom == name:
                t = [0]
                dist_x = x - r.pos[-1].x
                dist_y = y - r.pos[-1].y
                angle = r.ori[-1] - np.arctan2(dist_y, dist_x)
                while t[-1] < duree:
                    t.append(t[-1]+step)
                    r.simulMCI(step, 0, -angle/duree)
                t = [0]
                dist_x = x - r.pos[-1].x
                dist_y = y - r.pos[-1].y
                while t[-1] < duree:
                    t.append(t[-1]+step)
                    vt = sqrt(dist_x**2 + dist_y**2)
                    r.simulMCI(step, vt/duree, 0)

    def goToPosRandom(self, nb_ite=5, step=0.01, duree=1):
        i = 0
        while i < nb_ite:
            i += 1
            for r in self.robots:
                seed(a=None, version=2)
                x = randint(-5+int(r.pos[-1].x), 5+int(r.pos[-1].x))
                y = randint(-5+int(r.pos[-1].y), 5+int(r.pos[-1].y))
                self.goToPos(r.nom, x, y, duree, step)

    def followLeader(self, name):
        for r in self.robots:
            if r.nom == name:
                leader = r
                print(leader.nom)
                print(name)
            self.trajCirc(leader.nom, duree=20)
            self.goToPos(r.nom, leader.pos[-1].x, leader.pos[-1].y)


if __name__ == "__main__":  # false lors d'un import

    def simulationControl(env):

        from time import time
        screen_size = 800

        # Init pygame
        pygame.init()
        thisClock = pygClock()
        screen = pygame.display.set_mode((screen_size, screen_size))

        scale = 100
        offset = np.array([300, 300]).astype(np.int_)

        image = pygame.image.load("tortoise.jpg").convert()
        image = pygame.transform.scale(image, (60, 45))
        background = pygame.image.load("beach.png").convert()
        # background = pygame.transform.scale(background, (800, 800))

        tStart = time()
        thisClock.tick()

        seed(a=None, version=2)
        for r in env.robots:
            seed(a=None, version=2)
            radius = randint(15, 50)
            rot_speed = randint(1, 3)
            const = randint(1,3)
            x = randint(10, screen_size - 10)
            y = randint(10, screen_size - 10)
            env.goToPos(r.nom, x, y)
            r.const = const
            screen.blit(image, (int(r.pos[-1].x), int(r.pos[-1].y)))
        t = 0.  # current time
        env.goToPos('rob1', screen_size/2, screen_size/2)
        while time() < tStart + 20.:  # Simulate twenty seconds
            pygame.event.get()
            screen.fill((255, 255, 255))  # Make screen white again -> otherwise all are displayed
            screen.blit(background, (0, 0))
            # Plot current

            #aPend.drawPyGame(screen, (255, 255, 0))
            #screen.blit(image, (int(rob2.pos[-1].x) , int(rob2.pos[-1].y) ))
            vr = 1 * sin(1 * t)
            vt = 60
            dt = float(thisClock.tick(30)) / 1000.  # Let time pass, at least as much to have 30fps max
            for r in env.robots:
                #env.trajCirc(r.nom, radius*r.const, rot_speed, step=0.01, duree=1)
                #env.goToPos(r.nom,radius*r.const,const*r.const*radius)
                #env.controleur(r.nom, x_des=1+screen_size/2, y_des=1+screen_size/2,kvit=0.1,kangle=.1)
                screen.blit(image, (int(r.pos[-1].x), int(r.pos[-1].y)))
                r.simulMCI(dt, vt, vr)

            #screen.blit(image, (int(rob1.pos[-1].x)+screen_size/2, int(rob1.pos[-1].y)+screen_size/2))
            # for r in env.robots:
            #     env.trajCirc(r.nom, radius, rot_speed, step=0.01, duree=1)



            # Refresh
            pygame.display.flip()

            # vr = 1 * sin(1 * t)
            # vt = 60
            # env.simulMCI(dt, vt, vr)
            # Update time
            t += dt

        pygame.display.quit()

        return None

    rob1 = Kobuki(nom='rob1', c='red')
    rob2 = Kobuki(nom='rob2', c='blue')
    rob3 = Kobuki(nom='rob3', c='green')

    rob4 = Kobuki(nom='rob4', c='black')
    rob5 = Kobuki(nom='rob5', c='yellow')
    rob6 = Kobuki(nom='rob6', c='orange')

    # rob7 = Kobuki(nom='rob7', c='magenta')
    # rob8 = Kobuki(nom='rob8', c='purple')
    # rob9 = Kobuki(nom='rob9', c='pink')

    simu = Simulateur('simu')

    simu.addKobuki(rob1)
    simu.addKobuki(rob2)
    simu.addKobuki(rob3)
    simu.addKobuki(rob4)
    simu.addKobuki(rob5)
    simu.addKobuki(rob6)

    # simu.addKobuki(rob7)
    # simu.addKobuki(rob8)
    # simu.addKobuki(rob9)

    dt = 0.01
    time = 85
    ite = 20
    x = 30
    y = -5

    # simu.goToPos('rob1', 2, 1)

    # simu.trajCirc('rob1',rayon=1, duree=time)

    # simu.trajCirc('rob1',rayon=1,duree=time)
    #simu.trajSinMCI('rob2', dt,time/2)
    #simu.goToPos('rob3', 10, 8)
    simu.trajSinMCD('rob4', dt, time)

    #simu.goToPos('rob1', 10, 10)
    #simu.trajCirc('rob1', rayon=1, duree=85)

    #simu.controleur('rob2', x_des=rob1.pos[-1].x, y_des=rob1.pos[-1].y, duree=120)
    # simu.controleur('rob2', x_des=10, y_des=10, duree=80)
    # simu.controleur('rob3', x_des=0, y_des=10, duree=80)
    # simu.controleur('rob4', x_des=0, y_des=0, duree=80)
    #simu.controleur('rob1', x_des=10, y_des=10)
    simu.trace()

    #simu.goToPos('rob1', 0, 10)
    # simu.followLeader('rob1')
    #simulationControl(simu)
