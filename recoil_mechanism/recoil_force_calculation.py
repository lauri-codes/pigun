import numpy as np
from scipy import constants
from scipy.integrate import tplquad
import matplotlib.pyplot as mpl


def get_Bz(z):
    """Calculates magnetic field at distance z from solenoid center. Formula
    from: https://en.wikipedia.org/wiki/Solenoid#Finite_continuous_solenoid.
    The minus sign in front indicates that the field points towards -z axis.
    """
    return -(mu0*N*I)/(2)*((l_sol/2-z)/(l_sol*np.sqrt(r_sol**2 + (l_sol/2-z)**2)) + (l_sol/2+z)/(l_sol*np.sqrt(r_sol**2 + (l_sol/2+z)**2)))


def get_mz(Br, V):
    """Calculates magnetic moment for a magnet with given remanence and volume.
    Formula from:
    https://en.wikipedia.org/wiki/Magnetic_moment#Relation_to_magnetization
    """
    return (1/mu0)*Br*V


def get_dBdz(z):
    """Calculates derivative of magnetic field with respect to z, at distance
    z. Uses centered finite difference.
    """
    h = 0.0001
    z_m = z-0.5*h
    z_p = z+0.5*h
    B_m = get_Bz(z_m)
    B_p = get_Bz(z_p)
    dBdz = (B_p-B_m)/h
    return dBdz

def get_Fz(z, mz):
    """Calculates force on magnetic moment mz at distance z from solenoid
    center. Formula from:
    https://en.wikipedia.org/wiki/Magnetic_moment#Force_on_a_moment
    """
    return mz*get_dBdz(z)

def get_integrated_Fz(z_mag):
    """Calculates the force on the magnet caused by the magnetic field of the
    solenoid. The magnet and solenoid axes are aligned, and z_mag measures the
    distance of the magnet center from the solenoid center.

    The integral calculates the total force by integrating B*M over the magnet
    volume in cylindrical coordinates.
    """
    return tplquad(
        lambda z, theta, r: M*get_dBdz(z)*r,
        0, r_mag,
        lambda theta: 0, lambda theta: 2*np.pi,
        lambda theta, z: z_mag-l_mag/2, lambda theta, phi: z_mag+l_mag/2,
        epsabs=1e-04,
        epsrel=1e-04,
    )

# Parameters
pi = constants.pi
mu0 = constants.mu_0
I = 6                       # Current in solenoid
r_sol = 0.003               # Solenoid radius
l_sol = 0.02                # Solenoid length
N = 1000                    # Number of coils
r_mag = 0.003               # Magnet radius
l_mag = 0.02                # Magnet length
Br = 1.280                  # Magnet remanecence. Value for N42 grade neodymium magnet.
V_mag = l_mag*pi*r_mag**2   # Magnet volume
mz = get_mz(Br, V_mag)      # Magnetization
M = mz/V_mag                # Magnetization density

# Force calculated with finite magnet and magnetization
Fz_int = []
for iz in z:
    Fz_int.append(get_integrated_Fz(iz)[0])
mpl.plot(100*z, Fz_int)
mpl.xlabel("z (cm)")
mpl.ylabel("F (N)")
mpl.show()
