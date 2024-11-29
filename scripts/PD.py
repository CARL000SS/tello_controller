
class PD:
    def __init__(self, dt_):

        # Intervalo de muestreo
        self.dt = dt_

        # Variables de error y control
        self.e = 0.0    # Error actual
        self.e_1 = 0.0  # Error anterior
        self.de = 0.0   # Derivada del error
        self.u = 0.0    # Control

        # limite de velocidad / entrada de control
        self.lim_vel = 10

    def derivate(self, e, e_1):
        # Calcula la derivada del error
        return (e - e_1) / self.dt
    
    def set_gains(self, kkp, kkd):
        # Actualiza las ganancias
        self.kp = kkp
        self.kd = kkd
    
    def saturate(self, u):
        if u > self.lim_vel:
            u = self.lim_vel
        elif u < -self.lim_vel:
            u = -self.lim_vel
        return u

    def control(self, w_d, w):
        self.e = w_d - w                                # Error actual
        self.de = self.derivate(self.e, self.e_1)       # Derivada del error

        # self.u = self.saturate(self.kp * self.e + self.kd * self.de)
        self.u = self.kp * self.e + self.kd * self.de   # Señal de control

        self.e_1 = self.e                               # Guarda el error actual como error previo
        return self.u