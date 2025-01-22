import sys


def calculate_output_voltage(r_top: float, r_bot: float) -> float:
    v_ref: float = 1.25
    v_out = v_ref * ((r_top / r_bot) + 1)
    return v_out


if __name__ == "__main__":
    if len(sys.argv) == 3:
        r_top = float(sys.argv[1])
        r_bot = float(sys.argv[2])
        v_out: float = calculate_output_voltage(r_top, r_bot)
        print(f"Vout = {v_out:.2f}")
    else:
        print("Expected arguments r_top and r_bot as command inputs!")
