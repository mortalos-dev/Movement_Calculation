from math import sqrt
from movement_lap import LapMove, LapMoveWithJerk


class Movement:
    """Need to aggregate and calculate data of one movement"""
    def __init__(self, profile, x0, x, v0, v):
        self.profile = profile
        self.X0 = x0
        self.X = x
        self.V0 = v0
        self.V = v
        self.vector = []

    def sort_movements(self):
        self.vector.sort(key=lambda x: getattr(x, 'X0'))

    def add_movement_to_end(self, profile, new_x, new_v):
        """Add new lap of distance that need to be started with current end parameters,
        but may with new movement profile"""
        pass

    def calculate_common(self, silence):
        """You enter values and start the calculation
        Now every single movement stored in vector - list of all movements
        In stage of calculation program choose what movement shall be"""
        S_acc = self.profile.acc / 2 * ((self.profile.velo - self.V0) / self.profile.acc) ** 2
        S_dec = self.profile.dec / 2 * ((self.profile.velo - self.V) / self.profile.dec) ** 2
        if not silence:
            print(f'Acceleration path = {S_acc}, '
                  f'Deceleration path = {S_dec}, '
                  f'Setpoint path = {abs(self.X - self.X0)}')

        if (S_dec + S_acc) < abs(self.X - self.X0):
            self.calculate_trapezial_common(silence, self.X0, self.X, self.V0, self.V)
        else:
            self.calculate_triangle_common(silence, self.X0, self.X, self.V0, self.V)

    def _calculate_common(self, silence, x0, x, v0, v):
        """You enter values and start the calculation
        Now every single movement stored in vector - list of all movements
        In stage of calculation program choose what movement shall be"""
        t_acc = abs(self.profile.velo - v0) / self.profile.acc
        t_dec = (self.profile.velo - v) / self.profile.dec
        S_acc = v0 * t_acc + self.profile.acc / 2 * t_acc ** 2
        S_dec = v0 * t_dec + self.profile.dec / 2 * t_dec ** 2
        if not silence:
            print(f'Acceleration path = {S_acc}, '
                  f'Deceleration path = {S_dec}, '
                  f'Setpoint path = {abs(x - x0)}')

        if (S_dec + S_acc) < abs(x - x0):
            self.calculate_trapezial_common(silence, x0, x, v0, v)
        else:
            self.calculate_triangle_common(silence, x0, x, v0, v)

    def calculate_trapezial_common(self, silence, x0, x, v0, v):
        t_acc = (self.profile.velo - v0) / self.profile.acc
        t_dec = (self.profile.velo - v) / self.profile.dec
        if not silence:
            print(f'Trapezial movement case: t_acc = {t_acc}, t_dec = {t_dec}, x0={x0}, x={x}, v0={v0}, v={v}')

        self.calculate_process_common(silence, t_acc, t_dec, x0, x, v0, v)

    def calculate_triangle_common(self, silence, x0, x, v0, v):
        t_acc, t_dec, S_acc, S_dec, V0_dec = self.triangle_movement_calculation(silence, x0, x, v0, v)
        if not silence:
            print(f'Triangle movement case: t_acc = {t_acc}, t_dec = {t_dec}, x0={x0}, x={x}, v0={v0}, v={v}')

        self.calculate_process_common(silence, t_acc, t_dec, x0, x, v0, v)

    def calculate_process_common(self, silence, t_acc, t_dec, x0, x, v0, v):
        acc_move = self.calculate_acceleration_move_common(silence, t_acc, x0, v0)
        self.vector.append(acc_move)
        dec_move = self.calculate_deceleration_move_common(t_dec, x, v)
        velo_move = self.calculate_velocity_move_common(acc_move.X, dec_move.X0, acc_move.V, dec_move.V0)
        if acc_move.X > dec_move.X0:
            print(f'ALARM: ACC > DEC, diff = {acc_move.X - dec_move.X0}')
            if (acc_move.X - dec_move.X0) < 0.01:   # constant really small value
                acc_move.X, dec_move.X0 = dec_move.X0, acc_move.X
        else:
            self.vector.append(velo_move)

        self.vector.append(dec_move)
        self.sort_movements()
        self.calculate_through_timing(0)

    def calculate_acceleration_move_common(self, silence, t, x0, v0):
        # initiations for acc_move calculate
        acc_move = LapMove('Acceleration', x0, 0, v0, 0, self.profile.acc, t, 0, 0, 0)
        acc_move.calculate_acceleration_lap()
        if not silence:
            print(acc_move.X0, acc_move.X, acc_move.V0, acc_move.V, acc_move.t)
        return acc_move

    def calculate_deceleration_move_common(self, t, x, v):
        # initiations for dec_move calculate
        dec_move = LapMove('Deceleration', 0, x, 0, v, -self.profile.dec, t, 0, 0, 0)
        dec_move.calculate_deceleration_lap()
        return dec_move

    def calculate_velocity_move_common(self, x0, x, v0, v):
        # initiations for velo_move
        velo_move = LapMove('Velocity', x0, x, v0, v, 0, 0, 0, 0, 0)
        velo_move.calculate_velocity_lap()
        return velo_move

    def triangle_movement_calculation(self, silence, x0, x, v0, v):
        """Calculate Triangle move, when exist only Acceleration and Deceleration without Velocity Movement
        RETURN TUPLE (t_acc, t_dec, S_acc, S_dec, V0_dec (V_acc))"""
        dec = float(self.profile.dec)
        acc = float(self.profile.acc)
        a = float(dec / 2 * (dec / acc + 1))
        V = float(v)
        V0 = float(v0)
        b = float(V * (dec / acc + 1))
        X0 = float(x0)
        X = float(x)
        c = float(V0 / acc * (V - V0) + (V - V0) ** 2 / (2 * acc) - (X - X0))
        D = float((b ** 2) - (4 * a * c))
        if not silence:
            print(f"b in sqr {b ** 2}, 4ac = {4 * a * c}, Disсriminant = {D}")
            print(f"a = {a}, b = {b}, c = {c}, x = {X}, x0 = {X0}")
        t1 = float((-b + sqrt(D)) / 2 / a)
        t2 = float((-b - sqrt(D)) / 2 / a)
        if not silence:
            print(f"t1 = {t1}, t2 = {t2}")
        t_dec = float(t1 if t1 > 0 else t2 if t2 > 0 else None)
        t_acc = float((V - V0 + dec * t_dec) / acc)
        if not silence:
            print(f'Common case: t_acc = {t_acc}, t_dec = {t_dec}')
        V0_dec = float(V0 + acc * t_acc)
        S_acc = float(V0 * t_acc + acc / 2 * t_acc ** 2)
        S_dec = float(V0_dec * t_dec - dec / 2 * t_dec ** 2)
        if not silence:
            print(f'Acceleration S = {S_acc}, '
                  f'Deceleration S = {S_dec}, '
                  f'V_acc = V0_dec = {V0_dec}')
        return t_acc, t_dec, S_acc, S_dec, V0_dec

    def calculate_through_timing(self, start_time):
        # start-to-end timing in all sorted movements
        for idx, item in enumerate(self.vector):
            if idx == 0:
                self.vector[idx].t_start = start_time
            else:
                self.vector[idx].t_start = self.vector[idx - 1].t_end

            self.vector[idx].calculate_end_time()

    def find_lap_by_coord(self, coordinate):
        """Find lap by coordinate"""
        for idx, item in enumerate(self.vector):
            if item.X0 <= coordinate < item.X:
                return item
        else:
            raise Exception('OutOfRange')

    def find_time_till_the_end(self, coordinate):
        """Find time till the end of movement"""
        lap = self.find_lap_by_coord(coordinate)
        time = self.vector[-1].t_end - (lap.get_time_from_start_in_coord(True, coordinate) + lap.t_start)
        return time

    def get_lap_end_time(self):
        return self.vector[-1].t_end

    def print_parameters(self):
        for item in self.vector:
            print(item)
        print()


class MovementWithSlowing(Movement):
    """Movement with slowing while move"""
    def __init__(self, profile, slow_profile, x0, x, v0, v, x_start_slow, x_stop_slow):
        super().__init__(profile, x0, x, v0, v)
        self.slow_profile = slow_profile
        self.x_start_slow = x_start_slow
        self.x_stop_slow = x_stop_slow

    def calculate_slowing_parameters(self, silence):
        # find lap in which slowing starts and delete all next
        for idx, item in enumerate(self.vector):
            if item.X0 <= self.x_start_slow < item.X:
                item.X = self.x_start_slow
                item.calculate_path_from_coordinate()
                item.t = item.get_time_from_start_in_coord(silence, item.X)
                item.V = item.get_velo_in_time(item.t)

                self.vector = self.vector[:idx + 1]
                break
        else:
            raise Exception('OutOfRange')

        # calculate slowing path
        V = self.vector[-1].V
        if V > self.slow_profile.velo:
            # need to decelerate to slow velocity and last of path move in slow velocity
            self.calculate_slowing_common(silence, V, -self.slow_profile.dec)
        elif V < self.slow_profile.velo:
            # need to accelerate to slow (really faster) velocity and last of path move in slow (really faster) velocity
            self.calculate_slowing_common(silence, V, self.slow_profile.acc)
        else:
            # need to all path move in slow velocity
            self.calculate_slowing_common(silence, V, 0)

        # recalculate other parameters of movement till the end of path (self.X)
        self._calculate_common(silence, self.x_stop_slow, self.X, self.vector[-1].V, self.V)

    def calculate_slowing_common(self, silence, v, a):
        slowing_time = abs(v - self.slow_profile.velo) / abs(a) if a != 0 else 0
        slowing_path = v * slowing_time + a / 2 * slowing_time ** 2
        max_slowing_path = abs(self.x_start_slow - self.x_stop_slow)
        if not silence:
            print(v, self.slow_profile.velo, a, slowing_path, max_slowing_path)
        if slowing_path >= max_slowing_path:
            # need to calculate dec lap of slowwing movement: time, velo in end point
            slow_dec_move = LapMove('Deceleration Slowwing', self.x_start_slow, self.x_stop_slow,
                                    v, self.slow_profile.velo,
                                    a, 0, max_slowing_path, 0, 0)
            slow_dec_move.t = slow_dec_move.get_time_from_start_in_coord(silence, self.x_stop_slow)
            slow_dec_move.V = slow_dec_move.get_velo_in_time(slow_dec_move.t)
            slow_dec_move.calculate_end_coordinate()
            if not silence:
                print(slow_dec_move.t, slow_dec_move.V, slow_dec_move.S)
            self.vector.append(slow_dec_move)
        else:
            # need to calculate dec and velo lap of slowing movement
            slowing_end_coord = self.x_start_slow + slowing_path
            slow_dec_move = LapMove('Deceleration Slowwing', self.x_start_slow, slowing_end_coord,
                                    v, self.slow_profile.velo,
                                    a, 0, slowing_path, 0, 0)
            slow_dec_move.t = slow_dec_move.get_time_from_start_in_coord(silence, slowing_end_coord)
            slow_dec_move.V = slow_dec_move.get_velo_in_time(slow_dec_move.t)
            self.vector.append(slow_dec_move)

            slow_move_path = self.x_stop_slow - slow_dec_move.X
            slow_move = LapMove('Slow Moving', slow_dec_move.X, self.x_stop_slow,
                                slow_dec_move.V, self.slow_profile.velo,
                                0, 0, slow_move_path, 0, 0)
            slow_move.calculate_in_velo_time()
            self.vector.append(slow_move)


class MovementWithJerk(Movement):
    """Movement with Jerk """
    pass
