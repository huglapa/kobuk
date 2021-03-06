import numpy as np
from math import pi
import matplotlib.pyplot as plt


class MoteurCC(object):
    """Modèle numérique du moteur à courant continu, avec Um comme entrée,
        et comme vitesse omega et couple gamma en sortie"""
    def __init__(self, Um, name, gamma=0, omega=0., R=1, L=0.001, kc=0.01, ke=0.01, J=0.01, f=0.1):
        self.tension = Um
        self.resistance = R
        self.inductance = L
        self.const_couple = kc  # const de couple
        self.const_fcem = ke    # const de la fcem
        self.inertie = J    # inertie du rotor
        self.frot_visq = f   # const frot visqueux
        self.couple = np.array((0,gamma))
        self.vitesse = np.array((0,omega))
        self.courant = np.array((0,0))
        self.vitesseAna = np.array((0,0))
        self.nom = name

    def EqElec(self, tension):
        """Equation electrique : Um(t) = E(t) + R*i(t)
         Hypothese : inductance L = 0
         Simulation avec tension Um en entrée et courant i en sortie."""
        courant_i = (tension-self.const_fcem*self.vitesse[-1])/self.resistance
        #self.courant.np.append(courant_i)
        self.courant = np.append(self.courant, ((courant_i)))

    def EqElec_induct(self, dt, tension):
        """ Equation electrique : Um(t) = E(t) + R*i(t) + L*(di(t)/dt)
         Simulation avec tension Um en entrée et courant i en sortie."""

        courant_i = (dt/self.inductance)*tension-self.const_fcem*self.vitesse[-1]+self.courant[-1]*\
                    (self.inductance/dt-self.resistance)

        self.courant = np.append(self.courant, ((courant_i)))

    def EqMoteur(self):
        """ Equation du moteur : gamma(t) = k_c * i(t)"""
        couple_gamma = self.const_couple*self.courant[-1]
        #self.couple.np.append(couple_gamma)
        self.couple = np.append(self.couple, ((couple_gamma)))

    def EqMeca(self, dt):
        """Equation mecanique : J*(dV(t)/dt)+f*V(t) = gamma(t)
        avec V(t) ==> vitesse de rotation omega du moteur
        simulation avec couple gamma en entrée et vitesse omega en sortie"""
        vitesse_suivante = (dt/self.inertie)*self.couple[-1]+self.vitesse[-1]*(1-self.frot_visq*(step/self.inertie))
        #self.vitesse.append(vitesse_suivante)
        self.vitesse = np.append(self.vitesse, ((vitesse_suivante)))

    def calcVit(self, dt, tension):
        self.EqElec(tension)
        self.EqMoteur()
        self.EqMeca(dt)

    def calcVit_induct(self, dt, tension):
        self.EqElec_induct(dt, tension)
        self.EqMoteur()
        self.EqMeca(dt)

    def analytical(self, t, tension):
        """solution analytique du problème sous l'hypothèse L=0"""
        K = self.const_couple/(self.const_fcem * self.const_couple + self.resistance * self.frot_visq)
        tau = self.resistance * self.inertie / (self.const_fcem * self.const_couple + self.resistance * self.frot_visq)
        speed = K*(1-np.exp(-t/tau))*tension
        #self.vitesseAna.append(speed)
        self.vitesseAna = np.append(self.vitesseAna,((speed)))
        return None

    #def couple(self):


class SimuMotCC(object):
    motors = []

    def __init__(self, nom):
        self.nom = nom

    def addMot(self,mot):
        """ajout d'un moteur dans le simulateur"""
        self.motors.append(mot)

    def rmKMot(self, mot):
        """retrait d'un moteur du simulateur"""
        for m in self.motors:
            if m.nom == mot:
                self.motors.remove(m)
            else:
                print('No motor named ' + mot + ' in simulateur ' + self.nom + '.')

    def ctrlP(self, vit_act, vit_des, P):
        volt = P * (vit_des - vit_act)

        return volt

    def ctrlPI(self, mot, vit_act, vit_des, kp, ki, dt=0.01):
        for m in self.motors:
            if m.nom == mot:
                vit_des = [vit_des]
                voltP = kp * (vit_des[-1] - m.vitesse[-1])
                voltI = (dt/ki) * sum(vit_des-m.vitesse)

    def simul(self, name, dt, duree, tens):
        for m in self.motors:
            if m == name:
                #t = [0]
                tt = np.array([0,0])
                while tt[-1] < duree:
                    print('aaa')
                    #t.append(t[-1]+dt)
                    tt = np.append(tt, (tt[-1] + dt))
                    m.analytical(tt[-1], tens)
                    m.calcVit(dt, tens)

        return None


    def simul_ctrlP(self, name, dt, duree, vitesse, Kp):
        for m in self.motors:
            if m == name:
                t = [0]
                tt = np.array((0,0))
                while tt[-1] < duree:
                    #t.append(t[-1]+dt)
                    tt = np.append(tt, ((tt[-1]+dt)))
                    tens = self.ctrlP(m.vitesse[-1], vitesse, Kp)

                    m.analytical(tt[-1], tens)
                    m.calcVit(dt, tens)
        return tt
    #
    # def simul_ctrlPI(self, name, dt, duree, vitesse, Kp):
    #     for m in self.motors:
    #         if m == name:
    #             t = [0]
    #             while t[-1] < duree:
    #                 t.append(t[-1]+dt)
    #                 tens = self.ctrlPI(m.vitesse[-1], vitesse, Kp)+dt*
    #
    #                 m.analytical(t[-1], tens)
    #                 m.calcVit(dt,tens)
    #     return t

    def simul_induct(self, name, dt, duree, tens):
        for m in self.motors:
            if m == name:
                t = [0]
                while t[-1] < duree:
                    t.append(t[-1]+dt)
                    m.analytical(t[-1], tens)
                    m.calcVit_induct(dt, tens)
        return t


if __name__ == "__main__":  # false lors d'un import

    vit = 150
    prop = 150
    step = 0.01
    duration = 1

    mot1 = MoteurCC(1, name='mot1')
    mot2 = MoteurCC(1, name='mot2')
    mot3 = MoteurCC(1, name='mot3')
    mot4 = MoteurCC(1, name='mot4')

    env = SimuMotCC('env')
    env.addMot(mot1)
    env.addMot(mot2)
    env.addMot(mot3)
    env.addMot(mot4)

    env.simul(mot1.nom, step, duration, 1)
    time = np.array((0,0))
    while time[-1]<duration:
        time = np.append(time,(time[-1]+step))

    # time = env.simul(mot1.nom, step, duration, 1)
    # time2 = env.simul_induct(mot2, step, duration,1)
    # time3 = env.simul_ctrlP(mot3, step, duration, vit, prop)
    # time4 = env.simul_induct(mot4, step, duration,1)

    plt.subplot(2, 2, 1)
    plt.plot(time, mot1.vitesse, time, mot1.vitesseAna)
    plt.title('Simulation 1')
    plt.ylabel('vitesse')

    # plt.subplot(2, 2, 2)
    # plt.plot(time2, mot2.vitesse, time2, mot2.vitesseAna)
    # plt.title('Simulation 2')
    # plt.ylabel('vitesse')
    #
    # plt.subplot(2, 2, 3)
    # plt.plot(time3, mot3.vitesse, time3, mot3.vitesseAna)
    # plt.title('Solution 3')
    # plt.xlabel('time (s)')
    # plt.ylabel('vitesse')
    #
    # plt.subplot(2, 2, 4)
    # plt.plot(time4, mot4.vitesse, time4, mot4.vitesseAna)
    # plt.title('Solution 4')
    # plt.xlabel('time (s)')
    # plt.ylabel('vitesse')

    plt.show()
