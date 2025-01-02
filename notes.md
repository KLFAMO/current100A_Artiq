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

15-12-2024
new high power circuit with diodes for tock

Vg - I characteristics
cur     vg
0       3.3997
0.5     4.063
1       4.168
1.5     4.234
2       4.283
2.5     4.323
3       4.358
3.5     4.390
4       4.4169
4.5     4.44
5       4.46
6       4.54
7       4.578
8       4.608
9       4.634

10      4.66
11      4.683
12      4.705
13      4.726
14      4.745
15      4.764
16      4.783
17      4.8
18      4.817
19      4.834

20      4.851
21      4.866
22      4.881
23      4.895
24      4.91
25      4.924


02-01-2025
working on shortening cycle time

mode 1 - 140us
mode 2 - 125us
mode 0 - 12us

full cycle set to 200us (before 300us)