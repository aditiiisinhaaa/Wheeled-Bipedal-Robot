import time
import math
import lgpio
from smbus2 import SMBus

GPIOCHIP = 0
LOOP_HZ = 200
DT = 1.0 / LOOP_HZ

ENC_L = (17, 27)
ENC_R = (22, 23)
MOT_L = (12, 5)
MOT_R = (13, 6)
INV_ENC_L, INV_ENC_R = False, True
INV_MOT_L, INV_MOT_R = False, True
CPR = 44 * 30
PWM_FREQ = 20000
MIN_PWM = 8.0

KP, KI, KD = 12.0, 0.5, 1.8
I_LIM = 30.0
KP_V, KI_V = 1.0, 0.5
SP_LIM = 4.0
KP_YAW = 0.05

ALPHA = 0.98
FALL_DEG = 35.0
PITCH_SIGN = 1.0


class MPU9250:
    ADDR = 0x68

    def __init__(self, bus=1):
        self.bus = SMBus(bus)
        w = self.bus.write_byte_data
        w(self.ADDR, 0x6B, 0x01)
        time.sleep(0.1)
        w(self.ADDR, 0x1A, 0x02)
        w(self.ADDR, 0x1B, 0x08)
        w(self.ADDR, 0x1C, 0x08)
        w(self.ADDR, 0x1D, 0x02)
        self.gb = (0.0, 0.0, 0.0)

    @staticmethod
    def _s16(h, l):
        v = (h << 8) | l
        return v - 65536 if v & 0x8000 else v

    def read(self):
        d = self.bus.read_i2c_block_data(self.ADDR, 0x3B, 14)
        s = self._s16
        ax = s(d[0], d[1]) / 8192.0
        ay = s(d[2], d[3]) / 8192.0
        az = s(d[4], d[5]) / 8192.0
        gx = s(d[8], d[9]) / 65.5 - self.gb[0]
        gy = s(d[10], d[11]) / 65.5 - self.gb[1]
        gz = s(d[12], d[13]) / 65.5 - self.gb[2]
        return ax, ay, az, gx, gy, gz

    def calibrate(self, n=400):
        print("Calibrating gyro - keep robot still...")
        sx = sy = sz = 0.0
        self.gb = (0.0, 0.0, 0.0)
        for _ in range(n):
            _, _, _, gx, gy, gz = self.read()
            sx += gx
            sy += gy
            sz += gz
            time.sleep(0.003)
        self.gb = (sx / n, sy / n, sz / n)


class Encoder:
    TABLE = {
        (0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
        (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1,
    }

    def __init__(self, h, pins, invert=False):
        self.h, self.a, self.b = h, pins[0], pins[1]
        self.sign = -1 if invert else 1
        self.count = 0
        for p in (self.a, self.b):
            lgpio.gpio_claim_alert(h, p, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        self.state = self._read()
        self.cbs = [
            lgpio.callback(h, p, lgpio.BOTH_EDGES, self._cb)
            for p in (self.a, self.b)
        ]

    def _read(self):
        return (lgpio.gpio_read(self.h, self.a) << 1) | lgpio.gpio_read(self.h, self.b)

    def _cb(self, chip, gpio, level, tick):
        new = self._read()
        self.count += self.sign * self.TABLE.get((self.state, new), 0)
        self.state = new


class Motor:
    def __init__(self, h, pins, invert=False):
        self.h, self.pwm, self.dir = h, pins[0], pins[1]
        self.sign = -1 if invert else 1
        lgpio.gpio_claim_output(h, self.pwm, 0)
        lgpio.gpio_claim_output(h, self.dir, 0)

    def set(self, u):
        u *= self.sign
        if abs(u) > 0.5:
            u = math.copysign(MIN_PWM + abs(u) * (100 - MIN_PWM) / 100.0, u)
        else:
            u = 0.0
        u = max(-100.0, min(100.0, u))
        lgpio.gpio_write(self.h, self.dir, 1 if u >= 0 else 0)
        lgpio.tx_pwm(self.h, self.pwm, PWM_FREQ, abs(u))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    h = lgpio.gpiochip_open(GPIOCHIP)
    imu = MPU9250()
    encL = Encoder(h, ENC_L, INV_ENC_L)
    encR = Encoder(h, ENC_R, INV_ENC_R)
    mL = Motor(h, MOT_L, INV_MOT_L)
    mR = Motor(h, MOT_R, INV_MOT_R)

    imu.calibrate()
    ax, ay, az, *_ = imu.read()
    angle = PITCH_SIGN * math.degrees(math.atan2(ax, az))

    bal_i = vel_i = 0.0
    vel_f = 0.0
    target_vel = 0.0
    yaw_target = 0
    armed = False
    prev_cL = prev_cR = 0
    t_prev = time.perf_counter()
    next_t = t_prev
    last_print = 0

    try:
        while True:
            next_t += DT
            now = time.perf_counter()
            dt = max(now - t_prev, 1e-4)
            t_prev = now

            ax, ay, az, gx, gy, gz = imu.read()
            acc_angle = PITCH_SIGN * math.degrees(math.atan2(ax, az))
            rate = PITCH_SIGN * gy
            angle = ALPHA * (angle + rate * dt) + (1 - ALPHA) * acc_angle

            cL, cR = encL.count, encR.count
            v = ((cL - prev_cL) + (cR - prev_cR)) / 2.0 / dt / CPR
            prev_cL, prev_cR = cL, cR
            vel_f += 0.2 * (v - vel_f)

            if abs(angle) > FALL_DEG:
                armed = False
            if not armed:
                mL.set(0)
                mR.set(0)
                bal_i = vel_i = 0.0
                if abs(angle) < 2.0:
                    armed = True
                    encL.count = encR.count = prev_cL = prev_cR = 0
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                continue

            v_err = target_vel - vel_f
            vel_i = clamp(vel_i + KI_V * v_err * dt, -SP_LIM, SP_LIM)
            sp = clamp(KP_V * v_err + vel_i, -SP_LIM, SP_LIM)

            err = angle - sp
            bal_i = clamp(bal_i + KI * err * dt, -I_LIM, I_LIM)
            u = KP * err + bal_i + KD * rate

            turn = KP_YAW * ((cL - cR) - yaw_target)

            mL.set(clamp(u - turn, -100, 100))
            mR.set(clamp(u + turn, -100, 100))

            if now - last_print > 0.1:
                last_print = now
                print(f"ang={angle:6.2f} sp={sp:5.2f} v={vel_f:5.2f} u={u:6.1f}")

            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

    except KeyboardInterrupt:
        pass
    finally:
        mL.set(0)
        mR.set(0)
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
