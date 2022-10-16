from movement_lap import MovementProfile
from movement import Movement, MovementWithSlowing


class App:
    """Main program
    One movement overlay another to reduce whole time of movement and main timing from start to end"""

    fast_Z = MovementProfile(1000, 5000, 5000, 10000)
    slow_Z = MovementProfile(120, 5000, 5000, 10000)
    normal_Z = MovementProfile(600, 3000, 3000, 100000)

    fast_Y = MovementProfile(1500, 5000, 5000, 100000)
    slow_Y = MovementProfile(120, 5000, 5000, 10000)
    normal_Y = MovementProfile(1000, 3000, 3000, 100000)

    Z_Axis_to_pick = MovementWithSlowing(fast_Z, slow_Z, 0, 350, 0, 0, 200, 300)
    Z_Axis_to_pick.calculate_common(True)
    Z_Axis_to_pick.calculate_slowing_parameters(True)
    Z_Axis_to_pick.print_parameters()
    time_start_next = Z_Axis_to_pick.get_lap_end_time() - Z_Axis_to_pick.find_time_till_the_end(349.99)
    print(time_start_next)

    Z_Axis_from_pick = Movement(normal_Z, 0, 350, 0, 0)
    Z_Axis_from_pick.calculate_common(True)
    Z_Axis_from_pick.calculate_through_timing(time_start_next)
    Z_Axis_from_pick.print_parameters()
    time_start_next = Z_Axis_from_pick.get_lap_end_time() - Z_Axis_from_pick.find_time_till_the_end(150)
    print(time_start_next)

    Y_Axis_to_place = Movement(normal_Y, 0, 850, 0, 0)
    Y_Axis_to_place.calculate_common(True)
    Y_Axis_to_place.calculate_through_timing(time_start_next)
    Y_Axis_to_place.print_parameters()
    time_start_next = Y_Axis_to_place.get_lap_end_time() - Y_Axis_to_place.find_time_till_the_end(600)
    print(time_start_next)

    Z_Axis_to_place = MovementWithSlowing(normal_Z, slow_Z, 0, 850, 0, 0, 400, 500)
    Z_Axis_to_place.calculate_common(True)
    Z_Axis_to_place.calculate_slowing_parameters(True)
    Z_Axis_to_place.calculate_through_timing(time_start_next)
    Z_Axis_to_place.print_parameters()
    time_start_next = Z_Axis_to_place.get_lap_end_time() - Z_Axis_to_place.find_time_till_the_end(849.99)
    print(time_start_next)

    Z_Axis_from_place = Movement(fast_Z, 0, 850, 0, 0)
    Z_Axis_from_place.calculate_common(True)
    Z_Axis_from_place.calculate_through_timing(time_start_next)
    Z_Axis_from_place.print_parameters()
    time_start_next = Z_Axis_from_place.get_lap_end_time() - Z_Axis_from_place.find_time_till_the_end(600)
    print(time_start_next)

    Y_Axis_to_pick = Movement(fast_Y, 0, 850, 0, 0)
    Y_Axis_to_pick.calculate_common(True)
    Y_Axis_to_pick.calculate_through_timing(time_start_next)
    Y_Axis_to_pick.print_parameters()
    time_start_next = Y_Axis_to_pick.get_lap_end_time() - Y_Axis_to_pick.find_time_till_the_end(600)
    print(time_start_next)


if __name__ == '__main__':
    App()
