#!/usr/bin/env python3
# ============================================================
# INTERACTIVE UAV CONTROLLER TOOLBOX
# ============================================================
#
# - Offline tuning toolbox
# - CasADi MLP dynamics
# - Interactive sliders
# - Re-simulates automatically when gains change
# - ONLY plots position tracking
#
# ============================================================

import json
import math
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider
from collections import deque
from scipy.signal import savgol_filter


# ============================================================
# CONFIG
# ============================================================

DT = 1.0 / 30.0
SIM_TIME = 60.0
N = int(SIM_TIME / DT)

DELAY_STEPS = 12

WINDOW = 11
POLY = 3

CONTROL_ALPHA = 0.15

MAX_DVDT = 5.0
MAX_VEL = 5.0
MAX_CMD = 1.0


# ============================================================
# LOAD MODEL
# ============================================================

MODEL_SO = "drone_model.so"
MODEL_JSON = "drone_metadata.json"

with open(MODEL_JSON, "r") as f:
    meta = json.load(f)

WINDOW_SIZE = int(meta["window_size"])

f_mlp = ca.external("f_mlp", MODEL_SO)

print(f"MLP loaded | window_size={WINDOW_SIZE}")


# ============================================================
# FILTER
# ============================================================

class OnlineSavgol:

    def __init__(self):

        self.buf = deque(
            [np.zeros(4)] * WINDOW,
            maxlen=WINDOW
        )

    def update(self, x):

        self.buf.append(np.copy(x))

        arr = np.array(self.buf)

        filt = savgol_filter(
            arr,
            WINDOW,
            POLY,
            axis=0
        )

        return filt[-1]


# ============================================================
# REFERENCES
# ============================================================

L = 1.5

REF_POINTS = np.array([

    [ L/2,  L/2, 1.8,  np.deg2rad(45)],
    [-L/2,  L/2, 1.5,  np.deg2rad(135)],
    [-L/2, -L/2, 1.3,  np.deg2rad(-135)],
    [ 0.0,  0.0, 1.6,  np.deg2rad(0)]

])

HOLD_TIME = 10.0


def get_reference(t):

    idx = int(t / HOLD_TIME) % len(REF_POINTS)

    xd = np.copy(REF_POINTS[idx])

    dxd = np.zeros(4)

    return xd, dxd


# ============================================================
# MODEL
# ============================================================

MODEL_SIMP = np.array([

    0.36050576,
    0.09429137,

    0.32957424,
    0.04444115,

    0.78166194,
    0.56543884,

    2.92184766,
    3.19338679
])

Ku = np.diag([

    MODEL_SIMP[0],
    MODEL_SIMP[2],
    MODEL_SIMP[4],
    MODEL_SIMP[6]
])

Kv = np.diag([

    MODEL_SIMP[1],
    MODEL_SIMP[3],
    MODEL_SIMP[5],
    MODEL_SIMP[7]
])

A = np.eye(4) - DT * Kv
B = DT * Ku


# ============================================================
# SIMULATION
# ============================================================


def run_sim(params):

    Ksp = np.diag([
        params['ksp_x'],
        params['ksp_y'],
        params['ksp_z'],
        params['ksp_psi']
    ])

    Ksd = np.diag([
        params['ksd_x'],
        params['ksd_y'],
        params['ksd_z'],
        params['ksd_psi']
    ])

    Kp = np.diag([
        params['kp_x'],
        params['kp_y'],
        params['kp_z'],
        params['kp_psi']
    ])

    Kd = np.diag([
        params['kd_x'],
        params['kd_y'],
        params['kd_z'],
        params['kd_psi']
    ])

    x = np.copy(REF_POINTS[0])

    v_body = np.zeros(4)

    u = np.zeros(4)

    u_prev = np.zeros(4)

    w_Ur_prev = np.zeros(4)

    u_buffer = [
        np.zeros(4)
        for _ in range(DELAY_STEPS)
    ]

    sg = OnlineSavgol()

    v_history = deque(
        [np.zeros(4)] * WINDOW_SIZE,
        maxlen=WINDOW_SIZE
    )

    u_history = deque(
        [np.zeros(4)] * WINDOW_SIZE,
        maxlen=WINDOW_SIZE
    )

    log_t = []
    log_x = []
    log_xd = []

    for k in range(N):

        t = k * DT

        xd, dxd = get_reference(t)

        # ====================================================
        # PREDICTOR
        # ====================================================

        x_pred = np.copy(x)
        v_pred = np.copy(v_body)

        yaw = x_pred[3]

        for i in range(DELAY_STEPS):

            u_delayed = u_buffer[i]

            v_pred = A @ v_pred + B @ u_delayed

            R = np.array([
                [math.cos(yaw), -math.sin(yaw), 0, 0],
                [math.sin(yaw),  math.cos(yaw), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            v_world = R @ v_pred

            x_pred += DT * v_world

            x_pred[3] = math.atan2(
                math.sin(x_pred[3]),
                math.cos(x_pred[3])
            )

            yaw = x_pred[3]

        # ====================================================
        # ERROR
        # ====================================================

        x_til = xd - x_pred

        x_til[3] = math.atan2(
            math.sin(x_til[3]),
            math.cos(x_til[3])
        )

        # ====================================================
        # WORLD VELOCITY
        # ====================================================

        yaw = x[3]

        R = np.array([
            [math.cos(yaw), -math.sin(yaw), 0, 0],
            [math.sin(yaw),  math.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        v_world = R @ v_body

        # ====================================================
        # CONTROLLER
        # ====================================================

        w_Ur = (
            Kd @ dxd
            +
            Ksp @ np.tanh(Kp @ x_til)
        )

        dUr = (w_Ur - w_Ur_prev) / DT

        w_Ur_prev = np.copy(w_Ur)

        M = R @ Ku

        rhs = (
            dUr
            +
            Ksd @ (w_Ur - v_world)
            +
            Kv @ v_world
        )

        u_raw = np.linalg.solve(M, rhs)

        u = (
            CONTROL_ALPHA * u_raw
            +
            (1.0 - CONTROL_ALPHA) * u_prev
        )

        u_prev = np.copy(u)

        u = np.clip(u, -MAX_CMD, MAX_CMD)

        # ====================================================
        # DELAY BUFFER
        # ====================================================

        u_buffer.pop(0)
        u_buffer.append(np.copy(u))

        # ====================================================
        # FILTER
        # ====================================================

        v_smooth = sg.update(v_body)

        v_history.append(np.copy(v_smooth))
        u_history.append(np.copy(u))

        # ====================================================
        # MLP INPUT
        # ====================================================

        v_win = np.array(v_history)
        u_win = np.array(u_history)

        x_input = np.hstack([
            v_win.flatten(),
            u_win.flatten(),
            u.flatten()
        ]).reshape(1, -1)

        # ====================================================
        # MLP
        # ====================================================

        dvdt = f_mlp(x_input).full().flatten()

        dvdt = np.clip(dvdt, -MAX_DVDT, MAX_DVDT)

        v_body = v_smooth + DT * dvdt

        v_body = np.clip(v_body, -MAX_VEL, MAX_VEL)

        # ====================================================
        # INTEGRATE
        # ====================================================

        yaw = x[3]

        R = np.array([
            [math.cos(yaw), -math.sin(yaw), 0, 0],
            [math.sin(yaw),  math.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        v_world = R @ v_body

        x += DT * v_world

        x[3] = math.atan2(
            math.sin(x[3]),
            math.cos(x[3])
        )

        if not np.all(np.isfinite(x)):
            break

        log_t.append(t)
        log_x.append(np.copy(x))
        log_xd.append(np.copy(xd))

    return (
        np.array(log_t),
        np.array(log_x),
        np.array(log_xd)
    )


# ============================================================
# INITIAL PARAMETERS
# ============================================================

params = {

    'ksp_x': 0.6,
    'ksp_y': 0.6,
    'ksp_z': 1.0,
    'ksp_psi': 1.5,

    'ksd_x': 0.3,
    'ksd_y': 0.3,
    'ksd_z': 1.0,
    'ksd_psi': 0.5,

    'kp_x': 1.5,
    'kp_y': 1.5,
    'kp_z': 1.2,
    'kp_psi': 1.5,

    'kd_x': 0.0,
    'kd_y': 0.0,
    'kd_z': 0.0,
    'kd_psi': 0.0,
}


# ============================================================
# INITIAL SIMULATION
# ============================================================

log_t, log_x, log_xd = run_sim(params)


# ============================================================
# FIGURE
# ============================================================

fig, axs = plt.subplots(4, 1, figsize=(14, 12))

plt.subplots_adjust(
    left=0.32,
    bottom=0.03,
    right=0.98,
    top=0.98,
    hspace=0.35
)

names = ['x', 'y', 'z', 'yaw']

real_lines = []
ref_lines = []

for i in range(4):

    l1, = axs[i].plot(
        log_t,
        log_x[:, i],
        linewidth=2
    )

    l2, = axs[i].plot(
        log_t,
        log_xd[:, i],
        '--',
        linewidth=2
    )

    real_lines.append(l1)
    ref_lines.append(l2)

    axs[i].grid(True)

    axs[i].set_ylabel(names[i])

axs[-1].set_xlabel('time [s]')


# ============================================================
# SLIDERS
# ============================================================

slider_axes = {}
sliders = {}

slider_names = list(params.keys())

for i, name in enumerate(slider_names):

    ax = plt.axes([
        0.03,
        0.95 - i * 0.055,
        0.22,
        0.025
    ])

    slider = Slider(
        ax=ax,
        label=name,
        valmin=0.0,
        valmax=5.0,
        valinit=params[name],
        valstep=0.01
    )

    slider_axes[name] = ax
    sliders[name] = slider


# ============================================================
# UPDATE FUNCTION
# ============================================================

updating = False


def update(val):

    global updating

    if updating:
        return

    updating = True

    try:

        for name in params:
            params[name] = sliders[name].val

        log_t, log_x, log_xd = run_sim(params)

        for i in range(4):

            real_lines[i].set_data(
                log_t,
                log_x[:, i]
            )

            ref_lines[i].set_data(
                log_t,
                log_xd[:, i]
            )

            axs[i].relim()
            axs[i].autoscale_view()

        fig.canvas.draw_idle()

    except Exception as e:

        print(f"Simulation error: {e}")

    updating = False


# ============================================================
# CONNECT CALLBACKS
# ============================================================

for s in sliders.values():
    s.on_changed(update)


# ============================================================
# SHOW
# ============================================================

plt.show()
