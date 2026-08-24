"""Verify protocol encoding against manual examples."""

from bunker_mini.protocol import MotionCommand


def test_forward_example() -> None:
    cmd = MotionCommand.from_si(0.15, 0.0)
    assert cmd.to_bytes() == bytes.fromhex("0096000000000000")


def test_rotate_example() -> None:
    cmd = MotionCommand.from_si(0.0, 0.2)
    assert cmd.to_bytes() == bytes.fromhex("000000c800000000")


if __name__ == "__main__":
    test_forward_example()
    test_rotate_example()
    print("protocol examples OK")
