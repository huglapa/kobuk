import numpy
class Vecteur3d(object):
    """Définit un vecteur 3D, produit vect avec * et produit scalaire avec **"""
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Vecteur3d(%g, %g, %g)" % (self.x, self.y, self.z)

    def __repr__(self):
        return "Vecteur3d(%g, %g, %g)" % (self.x, self.y, self.z)

    def __add__(self, other):
        return Vecteur3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __neg__(self):
        return Vecteur3d(-self.x , -self.y, -self.z)

    def __sub__(self, other):
        return self + (-other)

    def __mul__(self, other):               # produit vectoriel
        if type(other) == Vecteur3d:
            X = self.y * other.z - self.z * other.y
            Y = self.x * other.z - self.z * other.x
            Z = self.x * other.y - self.y * other.x
        else:
            X = other * self.x
            Y = other * self.y
            Z = other * self.z

        return Vecteur3d(X, Y, Z)

    def __rmul__(self, other):
        return self * other

    def __pow__(self, other):           # Produit scalaire
        if type(other) == Vecteur3d:
            X = self.x * other.x
            Y = self.y * other.y
            Z = self.z * other.z
            return X+Y+Z
        else:
            X = other * self.x
            Y = other * self.y
            Z = other * self.z
            return Vecteur3d(X,Y,Z)

    def __rpow__(self, other):
        return self * other

    def __truediv__(self, other):
        return self ** (1/other)

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        else:
            return False

    def __gt__(self, other):
        return self.mod()>other.mod()

    def __lt__(self, other):
        return self.mod()<other.mod()

    def __ge__(self, other):
        return not self.mod()<other.mod()

    def __le__(self, other):
        return not self.mod()>other.mod()

    def mod (self):
        return (self ** self) ** 0.5

    def norm(self):
        return self * (1/self.mod())

    def normed(self):
        M= self.norm()
        self.x = M.x
        self.y = M.y
        self.z = M.z


if __name__ == "__main__": # false lors d'un import

    V1 = Vecteur3d(1, 0, 0)
    V2 = Vecteur3d(0, 1, 0)
    V3 = Vecteur3d(1, 2, 3)

    print("V1+V2 =", V1+V2)
    print("-V3 =", -V3)
    print("V2-V3+V1 =", V2-V3+V1)
    print("V1*V2 =", V1*V2)
    print("V1*42 =", V1*42)
    print("V1**V2=", V1**V2)

    V12 = V1+V2
    print("module de V12 =", V12.mod())
    print("Vecteur V12 normalisé =", V12.norm())
    print()
    print(V12, V12.mod())
    print(V12.normed())
    print(V12, V12.mod())


class Torseur(object):
    """P: point d'application; R: résultante; M: moment"""
    def __init__(self, p=Vecteur3d(), r=Vecteur3d(), m=Vecteur3d()):
        self.p = p
        self.r = r
        self.m = m

    def __str__(self):
        msg = 'Torseur('+str(self.p)+',\n\t'+str(self.r)+',\n\t'+str(self.m)+')\n'
        return msg

    def __repr__(self):
        msg = 'Torseur('+str(self.p)+','+str(self.r)+','+str(self.m)+')'
        return msg

    def chgPt(self, pd=Vecteur3d()):
        md = self.m + (pd - self.p) * self.r
        self.p = pd
        self.m = md

    def __add__(self, other):
        if self.p != other.p:
            Ptemp = other.p
            other.chgPt(self.p)

        S = Torseur(self.p, self.r + other.r, self.m + other.m)

        try:
            other.chgPt(Ptemp)
        except:
            pass

        return S

    def __mul__(self, other):
            chgPt(other, self.p)
            return float(self.r * other.m + self.m * other.r)

    def __neg__(self):
            return Torseur(self.p, -self.r, -self.m)

    def __sub__(self, other):
            return self + (-other)

    def __eq__(self, other):
        if not (self.p == other.p):
            Ptemp = other.p
            other.chgPt(self.p)

        S = (self.r == other.r and self.m == other.m)

        try:
            other.chgPt(Ptemp)
        except:
            pass
        return S


if __name__ == "__main__":  # false lors d'un import

    P1 = Vecteur3d(y=3)
    R1 = Vecteur3d(1, 2, 3)
    M1 = Vecteur3d(3, 2, 3)

    P2 = Vecteur3d()
    R2 = Vecteur3d(4, 1, 3)
    M2 = Vecteur3d(3, 0, 3)

    T1 = Torseur(P1, R1, M1)
    T0 = Torseur()
    T2 = Torseur(P2, R2, M2)

    print(T0)
    # T1.chgPt(P2)
    print(T1)
    print(T2)
    print(T1 + T2 == T2 + T1)
    print(T2)

    #else:
    #   return (self +)

    # type(self) == type(other) moins bien que isinstance(other, Torseur)
    # class Rectangle
    # class Carre(Rectangle)
    #    def __init__(self,cote)

    # rectangle __init__(self,cote,cote)
    # super(carre, self).__init__(self,cote,cote)
