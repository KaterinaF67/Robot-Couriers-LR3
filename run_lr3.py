import argparse
import csv
import json
from pathlib import Path
from mas import Settings, Simulation, make_scenario
from experiments_lr3 import lower_initial_batteries
def main():
    p=argparse.ArgumentParser(description='MAS lab 3: charging resource conflicts')
    p.add_argument('--policy',choices=('fifo','priority','both'),default='both')
    p.add_argument('--assignment',choices=('central','contract'),default='central')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--robots',type=int,default=8)
    p.add_argument('--stations',type=int,default=1)
    p.add_argument('--orders',type=int,default=20)
    p.add_argument('--width',type=int,default=18)
    p.add_argument('--height',type=int,default=18)
    p.add_argument('--ticks',type=int,default=900)
    p.add_argument('--battery',type=int,default=100)
    p.add_argument('--rate',type=int,default=5)
    p.add_argument('--obstacles',type=float,default=.08)
    p.add_argument('--trace',action='store_true')
    p.add_argument('--output',default='results_lr3/manual')
    a=p.parse_args()
    cfg=Settings(width=a.width,height=a.height,n_robots=a.robots,
                 n_stations=a.stations,n_orders=a.orders,max_ticks=a.ticks,
                 battery_capacity=a.battery,charging_rate=a.rate,
                 seed=a.seed,obstacles=a.obstacles,scenario='manual')
    try:
        scenario=lower_initial_batteries(make_scenario(cfg),cfg.battery_capacity)
    except ValueError as exc:
        p.error(str(exc))
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for policy in (('fifo','priority') if a.policy=='both' else (a.policy,)):
        sim=Simulation(cfg,a.assignment,scenario,charge_policy=policy,trace=a.trace)
        row=sim.run();rows.append(row)
        print(json.dumps(row,ensure_ascii=False,indent=2))
        if a.trace:
            (out/(policy+'_trace.json')).write_text(json.dumps(sim.events,ensure_ascii=False,indent=2),encoding='utf-8')
    with (out/'summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,rows[0]);w.writeheader();w.writerows(rows)
if __name__=='__main__':main()
