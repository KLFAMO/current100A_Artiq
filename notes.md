11-09-2024

Current fluctuates around +-10 mA (need to be explainded). On both, Power supply and LEM
V_set |     V_gate  |   I_power_sup |   I_lem_mA
3.130       3.11        0               4
3.180       3.16        0               5
3.270       3.15        0.005           12
3.4         3.38        0.027           33
3.6         3.57        0.180           182
3.7         3.67        0.415           410
3.8         3.77        0.912           880
3.9                     1.88            1820
4.0                     3.96            3376

27-09-2024

controller cycle = 2 ms
meas and sets = 1.17 ms  (TIM7: prescaler=80, period=2000)
for prescaler 80 -> period is measured in us.

single cnv measurement = 1us

changed cycle to 1ms
adc average 200
meas and sets time = 220us

23.10.2024
40A  -  I=-0.015 (ringing)
30A  -  I=-0.019 (ringing)
20A  -  I=-0.025 (ringing)
10A  -  I=-0.042 (ringing)
5A   -  I=-0.077 (ringing)
2A   -  I=-0.18  (ringing)

we set I = -0.01 for 40 A
