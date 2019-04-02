import numpy as np
from math import pi
import matplotlib.pyplot as plt


class MoteurCC(object):
    """Modèle numérique du moteur à courant continu, avec Um comme entrée,
        et comme vitesse omega et couple gamma en sortie"""
    def __init__(self, Um, gamma=0, omega=0., R=1, L=0.001, kc=0.01, ke=0.01, J=0.01, f=0.1):
        self.tension = Um
        self.resistance = R
        self.inductance = L
        self.const_couple = kc  # const de couple
        self.const_fcem = ke    # const de la fcem
        self.inertie = J    # inertie du rotor
        self.frot_visq = f   # const frot visqueux
        self.couple = [gamma]
        self.vitesse = [omega]
        self.courant = [0]
        self.vitesseAna = [0]

    def EqElec(self, tension):
        """Equation electrique : Um(t) = E(t) + R*i(t)
         Hypothese : inductance L = 0
         Simulation avec tension Um en entrée et courant i en sortie."""

        courant_i = (tension-self.const_fcem*self.vitesse[-1])/self.resistance
        self.courant.append(courant_i)

    def EqElecAvecL(self, step, tension):
        """ Equation electrique : Um(t) = E(t) + R*i(t) + L*(di(t)/dt)
         Simulation avec tension Um en entrée et courant i en sortie."""

        courant_i = (step/self.inductance)*tension-self.const_fcem*self.vitesse[-1]+self.courant[-1]*\
                    (self.inductance/step-self.resistance)
        self.courant.append(courant_i)

    def EqMoteur(self):
        """ Equation du moteur : gamma(t) = k_c * i(t)"""
        couple_gamma = self.const_couple*self.courant[-1]
        self.couple.append(couple_gamma)

    def EqMeca(self, step):
        """Equation mecanique : J*(dV(t)/dt)+f*V(t) = gamma(t)
        avec V(t) ==> vitesse de rotation omega du moteur
        simulation avec couple gamma en entrée et vitesse omega en sortie"""
        vitesse_suivante = (step/self.inertie)*self.couple[-1]+self.vitesse[-1]*(1-self.frot_visq*(step/self.inertie))
        self.vitesse.append(vitesse_suivante)

    def analytical(self, t, tension):
        """solution analytique du problème sous l'hypothèse L=0"""
        K = self.const_couple/(self.const_fcem * self.const_couple + self.resistance * self.frot_visq)
        tau = self.resistance * self.inertie / (self.const_fcem * self.const_couple + self.resistance * self.frot_visq)
        speed = K*(1-np.exp(-t/tau))*tension
        self.vitesseAna.append(speed)
        return None

    #def couple(self):


def controlP(vit_act, vit_des, P):
    volt = P * (vit_des - vit_act)

    return volt


def simulation(moteur, dt, duree, tens):
    t = [0]
    while t[-1] < duree:
        t.append(t[-1]+dt)

        mot1.analytical(t[-1], tens)
        moteur.EqElec(tens)
        moteur.EqMoteur()
        moteur.EqMeca(dt)
    return t


def simulation_PROP(moteur, dt, duree, vitesse, Kp):
    t = [0]
    while t[-1] < duree:
        t.append(t[-1]+dt)
        tens = controlP(moteur.vitesse[-1], vitesse, Kp)

        mot1.analytical(t[-1], tens)
        moteur.EqElec(tens)
        moteur.EqMoteur()
        moteur.EqMeca(dt)
    return t


def simulation_induct(moteur, dt, duree, tens):
    t = [0]
    while t[-1] < duree:
        t.append(t[-1]+dt)
        mot1.analytical(t[-1], tens)

        moteur.EqElecAvecL(dt, tens)
        moteur.EqMoteur()
        moteur.EqMeca(dt)
    return t

vit = 150
prop = 150
step = 0.01
duration = 1

mot1 = MoteurCC(1)
mot2 = MoteurCC(1)
mot3 = MoteurCC(1)
mot4 = MoteurCC(1)

time = simulation(mot1, step, duration, 1)
time2 = simulation_induct(mot2, step, duration,1)
time3 = simulation_PROP(mot3, step, duration, vit, prop)
time4 = simulation_induct(mot4, step, duration,1)

plt.subplot(2, 1, 1)
plt.plot(time, mot1.vitesse)
plt.title('Simulation 1')
plt.ylabel('vitesse')

plt.subplot(2, 1, 2)
plt.plot(time2, mot2.vitesse)
plt.title('Simulation 2')
plt.ylabel('vitesse')

plt.subplot(2, 2, 3)
plt.plot(time3, mot3.vitesse)
plt.title('Solution 3')
plt.xlabel('time (s)')
plt.ylabel('vitesse')

plt.subplot(2, 2, 4)
plt.plot(time4, mot4.vitesse)
plt.title('Solution 4')
plt.xlabel('time (s)')
plt.ylabel('vitesse')

plt.show()
