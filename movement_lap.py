import math
from typing import NamedTuple


class MovementProfile(NamedTuple):
    """Class container of movement profile"""
    velo: float
    acc: float
    dec: float
    jerk: float


class LapMove:
    """One lap of movement (acc, velo, dec) and container for parameters (X0, X, V0, V, t, S)"""
    def __init__(self, name, x0, x, v0, v, a, t, s, t_start, t_end):
        self.name = name
        self.X0 = x0
        self.X = x
        self.V0 = v0
        self.V = v
        self.a = a
        self.t = t
        self.S = s
        self.t_start = t_start
        self.t_end = t_end

    def set_class_name(self, name):
        """Set internal lap move name by the value of acceleration"""
        if name:
            self.name = name
            return
        if self.a < 0:
            self.name = 'Deceleration'
        elif self.a > 0:
            self.name = 'Acceleration'
        else:
            if self.V == 0 and self.V0 == 0:
                self.name = 'Dwell Time'
            else:
                self.name = 'Velocity'

    def calculate_lap_move(self):
        """Main method to calculate all type of movements"""
        if self.a > 0:
            self.calculate_acceleration_lap()
        elif self.a < 0:
            self.calculate_deceleration_lap()
        else:
            self.calculate_velocity_lap()

    def calculate_acceleration_lap(self):
        """Calculate lap in acceleration move. You need set V0, X0, t, a and then may calculate V, X, S"""
        self.calculate_end_coordinate()
        self.calculate_path_from_coordinate()
        self.calculate_end_velo()

    def calculate_deceleration_lap(self):
        """Calculate lap in deceleration move. You need set V, X, t, a and then may calculate V0, X0, S"""
        self.calculate_start_velo()
        self.calculate_start_coordinate()
        self.calculate_path_from_coordinate()

    def calculate_velocity_lap(self):
        """Calculate lap in velocity move. You need set V0, V, X0, X, a=0 and then may calculate t, S"""
        self.calculate_path_from_coordinate()
        self.calculate_in_velo_time()

    def calculate_end_coordinate(self):
        """Calculate end coordinate X if you know X0, V0, a, t"""
        self.X = self.X0 + self.V0 * self.t + self.a / 2 * self.t ** 2

    def calculate_start_coordinate(self):
        """Calculate start coordinate X0 if you know X, V0, a, t"""
        self.X0 = self.X - self.V0 * self.t - self.a / 2 * self.t ** 2

    def calculate_path_from_coordinate(self):
        """Calculate S if you know X and X0. Only Positive"""
        self.S = abs(self.X - self.X0)

    def calculate_end_time(self):
        """Calculate End time t_end if you know t_start and t"""
        self.t_end = self.t_start + self.t

    def calculate_end_velo(self):
        """Calculate end velo V if you know V0, a, t"""
        self.V = self.V0 + self.a * self.t

    def calculate_start_velo(self):
        """Calculate start velo V0 if you know V, a, t"""
        self.V0 = self.V - self.a * self.t

    def calculate_in_velo_time(self):
        """Calculate t in VELO MOVE if you know S, V"""
        self.t = self.S / self.V

    def get_velo_in_time(self, time):
        """Return Velo V1 in moment with input time t"""
        return self.V0 + self.a * time

    def get_time_from_start_in_coord(self, silence, coordinate):
        """Calculate t in EXPR: at2/2 + V0t + (x0 - x) = 0  in cases when a=0 and a<>0.
        Return only one root and only when it positive"""
        if coordinate < self.X0 or coordinate > self.X:
            raise Exception('OutOfRange')
        if self.a != 0:
            a = float(self.a / 2)
            b = float(self.V0)
            c = float(self.X0 - coordinate)
            D = float((b ** 2) - (4 * a * c))
            if not silence:
                print(f"b in sqr {b ** 2}, 4ac = {4 * a * c}, Disсriminant = {D}")
                print(f"a = {a}, b = {b}, c = {c}, coordinate = {coordinate}, x0 = {self.X0}")
            t1 = float((-b + math.sqrt(D)) / 2 / a)
            t2 = float((-b - math.sqrt(D)) / 2 / a)
            if not silence:
                print(f"t1 = {t1}, t2 = {t2}")
            if t1 > 0 and t2 < 0:
                return t1
            elif t1 < 0 and t2 > 0:
                return t2
            else:
                return min(t1, t2)
        else:
            return (coordinate - self.X0) / self.V0

    def __str__(self):
        return f'{self.name} - X0={self.X0}, X={self.X}, S={self.S}, V0={self.V0}, V={self.V}, a={self.a}, ' \
               f't={self.t}, t_start={self.t_start}, t_end={self.t_end}'


class LapMoveWithJerk(LapMove):
    """One lap of movement (acc, velo, dec, jerk_up, jerk_down)
    and container for parameters (X0, X, V0, V, t, S, t_start, t_end)"""
    def __init__(self, x0, x, v0, v, a, t, s, t_start, t_end, j):
        super().__init__(x0, x, v0, v, a, t, s, t_start, t_end)
        self.j = j

    def calculate_end_coordinate(self):
        self.X = self.X0 + self.V0 * self.t + self.a / 2 * self.t ** 2 + self.j / 6 * self.t ** 3

    def calculate_start_coordinate(self):
        self.X0 = self.X - self.V0 * self.t - self.a / 2 * self.t ** 2 - self.j / 6 * self.t ** 3

    def calculate_end_velo(self):
        self.V = self.V0 + self.a * self.t + self.j / 2 * self.t ** 2

    def calculate_start_velo(self):
        self.V0 = self.V - self.a * self.t - self.j / 2 * self.t ** 2
