#!/usr/bin/env python3
"""
Bebop Drone Simulation Toolbox
================================
• El dron parte VOLANDO en x0 = [0, 0, 1.5, 0]  (ya en aire, v_body = 0)
• Pipeline identico al simulador Webots (BebopWebotsMLPSim.timer_cb):
    - Filtro Savitzky-Golay online sobre v_body
    - u_history.append(u)  ANTES de construir la entrada MLP
    - Euler:  v_new = v_smooth + dt * f_mlp(v_win, u_win, u_actual)
    - Integracion inercial identica a _integrate_inertial
• Controlador SC identico a neroControl.py (opt=1, alpha=0.6)
• Referencia: 4 poses x 10 s, ciclo infinito
• Mover cualquier slider -> re-simula y actualiza graficas al soltar

Requiere drone_model.so y drone_metadata.json en el mismo directorio.
Requisitos:  pip install numpy scipy matplotlib casadi
"""

import math
import json
import os
import numpy as np
import matplotlib
_backends = ["TkAgg", "Qt5Agg", "Qt6Agg", "WxAgg", "GTK4Agg", "MacOSX"]
for _b in _backends:
    try:
        matplotlib.use(_b, force=True)
        import matplotlib.pyplot as _t; _t.figure(); _t.close("all")
        break
    except Exception:
        continue

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button
from collections import deque
from scipy.signal import savgol_filter

# =============================================================================
# 0.  MODELO MLP  —  CasADi + drone_model.so + drone_metadata.json
# =============================================================================
import casadi as ca

_BASE = os.path.dirname(os.path.abspath(__file__))
_SO   = os.path.join(_BASE, "drone_model.so")
_JSON = os.path.join(_BASE, "drone_metadata.json")

with open(_JSON) as _f:
    _meta = json.load(_f)

WINDOW_SIZE = int(_meta["window_size"])
DT          = float(_meta["dt"])
_F_MLP      = ca.external("f_mlp", _SO)

def f_mlp_call(v_win: np.ndarray, u_win: np.ndarray, u_actual: np.ndarray) -> np.ndarray:
    """
    Identico a BebopWebotsMLPSim._build_mlp_input + _mlp_derivative:
        x_input = hstack(v_win.flatten(), u_win.flatten(), u_actual.flatten()).reshape(1,-1)
        dvdt    = f_mlp(x_input).full().flatten()
    """
    x_in = np.hstack((v_win.flatten(),
                      u_win.flatten(),
                      u_actual.flatten())).reshape(1, -1)
    return _F_MLP(x_in).full().flatten()

# =============================================================================
# 1.  PARAMETROS
# =============================================================================
SIM_DURATION = 60.0
N_STEPS      = int(SIM_DURATION / DT)
T_AXIS       = np.arange(N_STEPS) * DT

SG_WIN  = 11   # SG_WINDOW del simulador
SG_POLY = 3    # SG_POLYORD del simulador
Z_FLOOR = 0.05

# Estado inicial: dron ya volando, quieto en [0, 0, 1.5, 0]
X0_POS  = np.array([0.0, 0.0, 1.5, 0.0])  # [x, y, z, yaw]
V0_BODY = np.zeros(4)                       # [vx_b, vy_b, vz_b, vpsi]

# Referencia — 4 poses x 10 s, ciclo infinito  (= RefPublisher)
REF_POINTS = np.array([
    [ 0.75,  0.75, 1.8],
    [-0.75,  0.75, 1.5],
    [-0.75, -0.75, 1.3],
    [ 0.00,  0.00, 1.6],
])
REF_YAWS = np.deg2rad([45.0, 135.0, -135.0, 0.0])
REF_HOLD = 10.0   # segundos por pose

# Model_simp del controlador (Parameters en neroControl.py)
# Identified from real-time filtered body-frame velocities:
MODEL_SIMP = np.array([1.02625, 2.35400, 0.72230, 1.40995,
                       2.34511, 2.06254, 2.56992, 2.49211])

# =============================================================================
# 2.  FILTRO SAVITZKY-GOLAY ONLINE  (copia de OnlineSavgolFilter)
# =============================================================================
class OnlineSavgolFilter:
    def __init__(self, wl=SG_WIN, po=SG_POLY, n=4):
        assert wl % 2 == 1
        self.wl, self.po = wl, po
        self._buf = deque([np.zeros(n)] * wl, maxlen=wl)

    def update(self, v_raw: np.ndarray) -> np.ndarray:
        self._buf.append(v_raw.copy())
        return savgol_filter(np.array(self._buf), self.wl, self.po, axis=0)[-1]

    def reset(self, v0=None):
        v0 = v0 if v0 is not None else np.zeros(4)
        self._buf = deque([v0.copy()] * self.wl, maxlen=self.wl)

# =============================================================================
# 3.  SIMULADOR  (replica timer_cb FLYING con warmup_done=True)
# =============================================================================
class DroneSimulator:
    def __init__(self):
        self._reset()

    def _reset(self):
        self.x      = X0_POS.copy()
        self.v_body = V0_BODY.copy()
        self.sg     = OnlineSavgolFilter()
        self.sg.reset(self.v_body)
        self.v_history = deque([np.zeros(4)] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self.u_history = deque([np.zeros(4)] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

    def reset(self):
        self._reset()

    def step(self, u_cmd: np.ndarray) -> np.ndarray:
        """
        Replica exacta de timer_cb() en modo FLYING / warmup_done=True:

          v_smooth = _update_velocity_buffer(v_body_raw)
                     └─ sg.update(v_raw) + v_history.append(v_smooth)

          u_history.append(u_cmd)          <- PRIMERO (igual que Webots)

          v_new = _euler_step(v_smooth, u_cmd, dt)
                  └─ dvdt = f_mlp( hstack(v_win, u_win, u_actual) )
                     return v_smooth + dt*dvdt

          v_body_raw = v_new
          _integrate_inertial(v_body_raw, dt)
        """
        u = np.clip(u_cmd, -1.0, 1.0)

        # _update_velocity_buffer
        v_smooth = self.sg.update(self.v_body)
        self.v_history.append(v_smooth.copy())

        # u_history ANTES de leer (igual que el nodo)
        self.u_history.append(u.copy())

        # _build_mlp_input
        v_win = np.array(self.v_history)   # (WINDOW_SIZE, 4)
        u_win = np.array(self.u_history)   # (WINDOW_SIZE, 4)

        # Euler step
        dvdt        = f_mlp_call(v_win, u_win, u)
        self.v_body = v_smooth + DT * dvdt

        # _integrate_inertial
        vx_b, vy_b, vz_b, vpsi = self.v_body
        psi   = self.x[3]
        x_dot = vx_b * math.cos(psi) - vy_b * math.sin(psi)
        y_dot = vx_b * math.sin(psi) + vy_b * math.cos(psi)
        self.x[0] += x_dot * DT
        self.x[1] += y_dot * DT
        self.x[2] += vz_b  * DT
        self.x[3] += vpsi  * DT
        self.x[3]  = math.remainder(self.x[3], 2.0 * math.pi)
        self.x[2]  = max(self.x[2], Z_FLOOR)

        return self.x.copy()

# =============================================================================
# 4.  CONTROLADOR SC  (cController opt=1 de neroControl.py)
# =============================================================================
class SCController:
    def __init__(self, gains: dict):
        self.gains     = gains
        self.w_Ur_prev = np.zeros(4)
        self.b_Ud      = np.zeros(4)
        self.b_Ud_ant  = np.zeros(4)

    def reset(self):
        self.w_Ur_prev[:] = 0.0
        self.b_Ud[:]      = 0.0
        self.b_Ud_ant[:]  = 0.0

    def compute(self, w_X, w_dX, w_Xd, w_dXd=None) -> np.ndarray:
        if w_dXd is None:
            w_dXd = np.zeros(4)
        g = self.gains
        Ksp = np.diag([g['ksp_x'], g['ksp_y'], g['ksp_z'], g['ksp_psi']])
        Ksd = np.diag([g['ksd_x'], g['ksd_y'], g['ksd_z'], g['ksd_psi']])
        Kp  = np.diag([g['kp_x'],  g['kp_y'],  g['kp_z'],  g['kp_psi']])
        Kd  = np.diag([g['kd_x'],  g['kd_y'],  g['kd_z'],  g['kd_psi']])
        Ku  = np.diag(MODEL_SIMP[[0, 2, 4, 6]])
        Kv  = np.diag(MODEL_SIMP[[1, 3, 5, 7]])

        w_Xtil    = w_Xd - w_X
        w_Xtil[3] = math.atan2(math.sin(w_Xtil[3]), math.cos(w_Xtil[3]))

        w_Ur  = Kd @ w_dXd + Ksp @ np.tanh(Kp @ w_Xtil)
        w_dUr = (w_Ur - self.w_Ur_prev) / CTRL_DT   # dt del controlador = 1/10 s
        self.w_Ur_prev = w_Ur.copy()

        psi   = w_X[3]
        c, s  = math.cos(psi), math.sin(psi)
        w_F_b = np.array([[ c, -s, 0, 0],
                           [ s,  c, 0, 0],
                           [ 0,  0, 1, 0],
                           [ 0,  0, 0, 1]])

        b_Ud_raw  = np.linalg.inv(w_F_b @ Ku) @ (
                        w_dUr + Ksd @ (w_Ur - w_dX) + Kv @ w_dX)

        # opt=1 → alpha = 0.6
        alpha         = 0.6
        self.b_Ud     = alpha * b_Ud_raw + (1.0 - alpha) * self.b_Ud_ant
        self.b_Ud_ant = self.b_Ud.copy()

        return np.clip(self.b_Ud, -1.0, 1.0)

# =============================================================================
# 5.  REFERENCIA
# =============================================================================
def get_reference(t: float) -> np.ndarray:
    idx = int(t / REF_HOLD) % len(REF_POINTS)
    p   = REF_POINTS[idx]
    return np.array([p[0], p[1], p[2], REF_YAWS[idx]])

# =============================================================================
# 6.  LOOP DE SIMULACION COMPLETA
# =============================================================================
CTRL_TICKS = 3   # controlador a 10 Hz: 1 vez cada 3 ticks del sim (30 Hz)
CTRL_DT    = DT * CTRL_TICKS   # dt interno del controlador = 1/10 s

def run_simulation(gains: dict):
    drone = DroneSimulator()
    ctrl  = SCController(gains)

    pos_log = np.zeros((N_STEPS, 4))
    ref_log = np.zeros((N_STEPS, 4))

    u = np.zeros(4)   # senal mantenida entre ticks del controlador (ZOH)

    for k in range(N_STEPS):
        t    = k * DT
        w_Xd = get_reference(t)

        w_X  = drone.x.copy()

        # Velocidad inercial: w_dX = w_F_b @ v_body
        # (igual que rGetSensorData en neroControl: w_dX = w_F_b @ b_twist)
        psi  = w_X[3]
        c, s = math.cos(psi), math.sin(psi)
        w_F_b = np.array([[ c, -s, 0, 0],
                           [ s,  c, 0, 0],
                           [ 0,  0, 1, 0],
                           [ 0,  0, 0, 1]])
        w_dX = w_F_b @ drone.v_body

        # Warmup: primeras WINDOW_SIZE muestras con u=0
        # Controlador a 10 Hz: solo recalcula cada CTRL_TICKS pasos
        if k < WINDOW_SIZE:
            u = np.zeros(4)
        elif k % CTRL_TICKS == 0:
            u = ctrl.compute(w_X, w_dX, w_Xd)
        # else: mantiene u del tick anterior (zero-order hold)

        drone.step(u)

        pos_log[k] = drone.x.copy()
        ref_log[k] = w_Xd

    return pos_log, ref_log

# =============================================================================
# 7.  GANANCIAS POR DEFECTO  (opt=1 de neroControl.py)
# =============================================================================
DEFAULT_GAINS = {
    'ksp_x': 1.2,   'ksp_y': 1.2,   'ksp_z': 1.2527,  'ksp_psi': 3.0,
    'ksd_x': 0.7,   'ksd_y': 0.7,   'ksd_z': 2.5484,  'ksd_psi': 1.2455,
    'kp_x':  1.95,  'kp_y':  1.95,  'kp_z':  1.5747,  'kp_psi':  2.0,
    'kd_x':  0.0,   'kd_y':  0.0,   'kd_z':  0.0,     'kd_psi':  0.0,
}

SLIDER_CFG = [
    # (key,       label,    min,   max)
    ('ksp_x',   'Ksp x',  0.0,   5.0),
    ('ksp_y',   'Ksp y',  0.0,   5.0),
    ('ksp_z',   'Ksp z',  0.0,   8.0),
    ('ksp_psi', 'Ksp ψ',  0.0,   8.0),
    ('ksd_x',   'Ksd x',  0.0,   8.0),
    ('ksd_y',   'Ksd y',  0.0,   8.0),
    ('ksd_z',   'Ksd z',  0.0,  12.0),
    ('ksd_psi', 'Ksd ψ',  0.0,   6.0),
    ('kp_x',    'Kp x',   0.0,   6.0),
    ('kp_y',    'Kp y',   0.0,   6.0),
    ('kp_z',    'Kp z',   0.0,   6.0),
    ('kp_psi',  'Kp ψ',   0.0,   6.0),
    ('kd_x',    'Kd x',   0.0,   3.0),
    ('kd_y',    'Kd y',   0.0,   3.0),
    ('kd_z',    'Kd z',   0.0,   3.0),
    ('kd_psi',  'Kd ψ',   0.0,   3.0),
]

# =============================================================================
# 8.  GUI
# =============================================================================
COLOR_EST   = ['#2196F3', '#4CAF50', '#FF5722', '#AB47BC']
COLOR_REF   = '#BDBDBD'
COLOR_CHG   = '#90A4AE'
PLOT_LABELS = ['X  (m)', 'Y  (m)', 'Z  (m)', 'Yaw  (°)']
PLOT_TITLES = ['Posicion X', 'Posicion Y', 'Altitud Z', 'Guinada ψ']

fig = plt.figure(figsize=(17, 10))
fig.canvas.manager.set_window_title('Bebop Drone Toolbox')

# Graficas arriba, sliders abajo
gs_top = gridspec.GridSpec(2, 2, figure=fig,
                           left=0.06, right=0.98,
                           top=0.93,  bottom=0.38,
                           hspace=0.45, wspace=0.30)
axes = [fig.add_subplot(gs_top[r, c]) for r in range(2) for c in range(2)]

gs_sl = gridspec.GridSpec(4, 4, figure=fig,
                          left=0.06, right=0.98,
                          top=0.34,  bottom=0.04,
                          hspace=1.1, wspace=0.55)

# Simulacion inicial
print("Simulacion inicial...")
pos0, ref0 = run_simulation(DEFAULT_GAINS.copy())
print("Listo.")

lines_ref = []
lines_est = []

for i, ax in enumerate(axes):
    yref = np.degrees(ref0[:, i]) if i == 3 else ref0[:, i]
    yest = np.degrees(pos0[:, i]) if i == 3 else pos0[:, i]

    lr, = ax.plot(T_AXIS, yref, color=COLOR_REF, lw=1.1, ls='--', label='Ref')
    le, = ax.plot(T_AXIS, yest, color=COLOR_EST[i], lw=1.5, label='Est')
    lines_ref.append(lr)
    lines_est.append(le)

    for k in range(1, int(SIM_DURATION / REF_HOLD) + 1):
        tc = k * REF_HOLD
        if tc < SIM_DURATION:
            ax.axvline(x=tc, color=COLOR_CHG, lw=0.7, ls=':', alpha=0.6)

    ax.set_title(PLOT_TITLES[i], fontsize=9, pad=3)
    ax.set_xlabel('t (s)', fontsize=8)
    ax.set_ylabel(PLOT_LABELS[i], fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, linewidth=0.5, alpha=0.5)
    if i == 0:
        ax.legend(fontsize=7, loc='upper right', framealpha=0.6)

fig.text(0.5, 0.965,
         'Bebop Drone Toolbox — parte en [0, 0, 1.5, 0], controlador activo desde t=0.67 s',
         ha='center', fontsize=10, fontweight='bold')
fig.text(0.5, 0.948,
         'Suelta el slider para re-simular | lineas grises = cambio de pose de referencia',
         ha='center', fontsize=8, color='gray')

# Metricas
txt_metrics = fig.text(0.5, 0.375, '', ha='center', fontsize=8, color='#444')

def _metrics(pos, ref):
    err  = np.linalg.norm(ref[:, :3] - pos[:, :3], axis=1)[WINDOW_SIZE:]
    rmse = np.sqrt(np.mean(err ** 2))
    maxe = np.max(err)
    return f'RMSE pos = {rmse:.4f} m   |   Error max = {maxe:.4f} m   (sin warmup)'

txt_metrics.set_text(_metrics(pos0, ref0))

# Sliders
sliders = {}
for idx, (key, lbl, vmin, vmax) in enumerate(SLIDER_CFG):
    row = idx // 4
    col = idx %  4
    ax_s = fig.add_subplot(gs_sl[row, col])
    sl   = Slider(ax=ax_s, label=lbl,
                  valmin=vmin, valmax=vmax,
                  valinit=DEFAULT_GAINS[key],
                  valstep=round((vmax - vmin) / 500, 5))
    sl.label.set_fontsize(8)
    sl.valtext.set_fontsize(7)
    sliders[key] = sl

# Boton reset
ax_btn    = fig.add_axes([0.44, 0.005, 0.12, 0.025])
btn_reset = Button(ax_btn, 'Reset ganancias')
btn_reset.label.set_fontsize(8)

# Callbacks
def _redraw(pos_new, ref_new):
    for i in range(4):
        yref = np.degrees(ref_new[:, i]) if i == 3 else ref_new[:, i]
        yest = np.degrees(pos_new[:, i]) if i == 3 else pos_new[:, i]
        lines_ref[i].set_ydata(yref)
        lines_est[i].set_ydata(yest)
        axes[i].relim()
        axes[i].autoscale_view()
    txt_metrics.set_text(_metrics(pos_new, ref_new))
    fig.canvas.draw_idle()

def _on_release(event):
    for sl in sliders.values():
        if event.inaxes == sl.ax:
            g = {k: s.val for k, s in sliders.items()}
            pos_new, ref_new = run_simulation(g)
            _redraw(pos_new, ref_new)
            return

def _on_reset(event):
    for key, sl in sliders.items():
        sl.set_val(DEFAULT_GAINS[key])
    pos_new, ref_new = run_simulation(DEFAULT_GAINS.copy())
    _redraw(pos_new, ref_new)

fig.canvas.mpl_connect('button_release_event', _on_release)
btn_reset.on_clicked(_on_reset)

plt.show()