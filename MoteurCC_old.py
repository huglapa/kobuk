import numpy as np
from math import pi
import matplotlib.pyplot as plt


class MoteurCC(object):
    """Modèle numérique du moteur à courant continu, avec Um comme entrée,
        et comme vitesse omega et couple gamma en sortie"""
    def __init__(self, name, Um=0, gamma=0., omega=0., R=1, L=0.001, kc=0.01, ke=0.01, J=0.01, f=0.1):
        self.tension = [Um]
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
        self.nom = name

    def EqElec(self, tension):
        """Equation electrique : Um(t) = E(t) + R*i(t)
         Hypothese : inductance L = 0
         Simulation avec tension Um en entrée et courant i en sortie."""
        self.tension.append(tension)

        courant_i = (self.tension[-1]-self.const_fcem*self.vitesse[-1])/self.resistance
        self.courant.append(courant_i)

    def EqElec_induct(self, dt, tension):
        """ Equation electrique : Um(t) = E(t) + R*i(t) + L*(di(t)/dt)
         Simulation avec tension Um en entrée et courant i en sortie."""
        self.tension.append(tension)

        courant_i = (dt/self.inductance)*self.tension[-1]-self.const_fcem*self.vitesse[-1]+self.courant[-1] * \
                    (self.inductance/dt-self.resistance)
        self.courant.append(courant_i)

    def EqMoteur(self):
        """ Equation du moteur : gamma(t) = k_c * i(t)"""
        couple_gamma = self.const_couple*self.courant[-1]
        self.couple.append(couple_gamma)

    def EqMeca(self, dt):
        """Equation mecanique : J*(dV(t)/dt)+f*V(t) = gamma(t)
        avec V(t) ==> vitesse de rotation omega du moteur
        simulation avec couple gamma en entrée et vitesse omega en sortie"""
        vitesse_suivante = (dt/self.inertie)*self.couple[-1]+self.vitesse[-1]*(1-self.frot_visq*(step/self.inertie))
        self.vitesse.append(vitesse_suivante)

    def simul1step(self, dt, tension):
        self.EqElec(tension)
        self.EqMoteur()
        self.EqMeca(dt)

    def simul1step_induct(self, dt, tension):
        self.EqElec_induct(dt, tension)
        self.EqMoteur()
        self.EqMeca(dt)

    def analytical(self, t, tension):
        """solution analytique du problème sous l'hypothèse L=0"""
        K = self.const_couple/(self.const_fcem * self.const_couple + self.resistance * self.frot_visq)
        tau = self.resistance * self.inertie / (self.const_fcem * self.const_couple + self.resistance * self.frot_visq)
        speed = K*(1-np.exp(-t/tau))*tension
        self.vitesseAna.append(speed)
        return None


class SimuMotCC(object):
    motors = []

    def __init__(self, nom):
        self.nom = nom

    def addMot(self,mot):
        """ajout d'un moteur dans le simulateur"""
        self.motors.append(mot)

    def rmMot(self, mot):
        """retrait d'un moteur du simulateur"""
        for m in self.motors:
            if m.nom == mot:
                self.motors.remove(m)
            else:
                print('No motor named ' + mot + ' in simulateur ' + self.nom + '.')

    def simul(self, name, dt, duree, tens):
        for m in self.motors:
            if m == name:
                t = [0]
                while t[-1] < duree:
                    t.append(t[-1]+dt)

                    m.analytical(t[-1], tens)
                    m.simul1step(dt, tens)
        return t

    def simul_ctrlP(self, name, dt, duree, vit_des, Kp):
        for m in self.motors:
            if m == name:
                t = [0]
                while t[-1] < duree:
                    t.append(t[-1]+dt)
                    tens = Kp * (vit_des-m.vitesse[-1])

                    m.analytical(t[-1], tens)
                    m.simul1step(dt, tens)
        return t

    def simul_ctrlPI(self, name, dt=0.01, duree=1, vit_des=1, kp=20, ki=60):
        for m in self.motors:
            if m == name:
                t = [0]
                volt_i = 0
                while t[-1] < duree:
                    t.append(t[-1]+dt)

                    volt_i += (1/(dt*ki)) * (vit_des - m.vitesse[-1])
                    volt = kp * (vit_des - m.vitesse[-1]) + volt_i
                    #tens = self.ctrlPI(m.vitesse[-1], vit_des, kp)+dt*

                    m.analytical(t[-1], volt)
                    m.simul1step(dt, volt)
        return t

    def simul_induct(self, name, dt, duree, tens):
        for m in self.motors:
            if m == name:
                t = [0]
                while t[-1] < duree:
                    t.append(t[-1]+dt)
                    m.analytical(t[-1], tens)
                    m.simul1step_induct(dt, tens)
        return t


if __name__ == "__main__":  # false lors d'un import

    vit = 1
    prop = 20
    integral = 45
    step = 0.01
    duration = 1
    consigne = np.ones((101))

    mot1 = MoteurCC(name='mot1',Um=1)
    mot2 = MoteurCC(name='mot2',Um=1)
    mot3 = MoteurCC(name='mot3',Um=1)
    mot4 = MoteurCC(name='mot4',Um=1)

    env = SimuMotCC('env')
    env.addMot(mot1)
    env.addMot(mot2)
    env.addMot(mot3)
    env.addMot(mot4)

    time = env.simul(mot1, step, duration, 1)
    time2 = env.simul_induct(mot2, step, duration, 1)
    #time3 = env.simul_ctrlP(mot3, step, duration, vit, prop)
    time3 = env.simul_ctrlPI(mot3, step, duration, vit, prop, integral)

    time4 = env.simul_induct(mot4, step, duration, 1)

    plt.plot(time3, mot3.vitesse, label='simulation')
    plt.plot(time3, mot3.vitesseAna, label='solution analytique')
    plt.plot(time3, consigne, label='consigne')

    plt.title('Correcteur PI ( avec Kp='+str(prop)+', Ki='+str(integral)+')')
    plt.xlabel('time (s)')
    plt.ylabel('rot speed (rad/s)')
    plt.legend()



    # plt.subplot(2, 2, 1)
    # plt.plot(time, mot1.vitesse, time, mot1.vitesseAna)
    # plt.title('Simulation 1')
    # plt.ylabel('vitesse')
    #
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
