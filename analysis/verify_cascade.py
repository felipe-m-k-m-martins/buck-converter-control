"""Check of the analytic cascaded PI design on the averaged buck model.

Linear averaged model (no PWM, no saturation, no current limit), with the
example gains from the design: L = 220 uH, C = 470 uF, R = 1 ohm, Vin = 24 V.
Requires: pip install control numpy
"""
import numpy as np
import control as ct

VIN, L, C, R = 24.0, 220e-6, 470e-6, 1.0
s = ct.tf('s')

# Inner current loop: duty -> iL approximated by Vin/(L s)
Ci = 0.115 + 1445 / s
_, pm_i, _, wc_i = ct.margin(Ci * VIN / (L * s))
print(f"Inner loop: phase margin {pm_i:.1f} deg, crossover {wc_i / (2 * np.pi):.0f} Hz")

# Outer voltage loop: plant seen with the inner loop closed, 1/(C s + 1/R)
Cv = 1.18 + 2510 / s
_, pm_v, _, wc_v = ct.margin(Cv / (C * s + 1 / R))
print(f"Outer loop: phase margin {pm_v:.1f} deg, crossover {wc_v / (2 * np.pi):.0f} Hz")

# Full cascade on the second-order averaged plant
A = np.array([[0, -1 / L], [1 / C, -1 / (R * C)]])
B = np.array([[VIN / L], [0]])
plant = ct.ss(A, B, np.eye(2), 0, inputs='d', outputs=['iL', 'vo'], name='plant')
ci = ct.tf2ss(Ci, inputs='ei', outputs='d', name='Ci')
cv = ct.tf2ss(Cv, inputs='ev', outputs='iref', name='Cv')
sum_i = ct.summing_junction(inputs=['iref', '-iL'], output='ei', name='sum_i')
sum_v = ct.summing_junction(inputs=['vref', '-vo'], output='ev', name='sum_v')
loop = ct.interconnect([plant, ci, cv, sum_i, sum_v], inputs='vref', outputs=['vo', 'iL'])

t = np.linspace(0, 0.01, 40001)
resp = ct.step_response(loop, T=t)
vref = 12.0
vo = vref * np.squeeze(resp.outputs[0])
outside = np.where(np.abs(vo - vref) > 0.02 * vref)[0]
print(f"0 to {vref:.0f} V step: overshoot {(vo.max() - vref) / vref * 100:.2f} %, "
      f"2 % settling time {t[outside[-1]] * 1e3:.2f} ms")
