from mini_rover.minirover_twist_publisher import tank_drive


def test_tank_drive():
    left, right = tank_drive(1.0, 0.0)
    assert left == 1.0 and right == 1.0, "Error on full forward power"

    left, right = tank_drive(-1.0, 0.0)
    assert left == -1.0 and right == -1.0, "Error on full back power"

    left, right = tank_drive(0.0, 1.0)
    assert left == -1.0 and right == 1.0, "Error on full left power"

    left, right = tank_drive(0.0, -1.0)
    assert left == 1.0 and right == -1.0, "Error on full right power"

    left, right = tank_drive(1.0, 1.0)
    assert left == 0.0 and right == 1.0, "Error on full forward / left power"

    left, right = tank_drive(1.0, -1.0)
    assert left == 1.0 and right == 0.0, "Error on full forward / right power"
