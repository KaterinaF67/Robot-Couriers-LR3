from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mas import Settings, Simulation, make_scenario
from experiments_lr3 import lower_initial_batteries
def main():
    checked=0
    for seed in range(25):
        for assignment in ('central','contract'):
            for policy in ('fifo','priority'):
                cfg=Settings(width=12+(seed%5)*2,height=12+(seed%5)*2,
                             n_robots=5+(seed%7),n_stations=1+(seed%3),
                             n_orders=8+(seed%16),seed=seed,
                             obstacles=.04+.01*(seed%8),
                             battery_capacity=90+(seed%4)*15,
                             max_ticks=550,charging_rate=4+(seed%4))
                scenario=lower_initial_batteries(make_scenario(cfg),cfg.battery_capacity)
                sim=Simulation(cfg,assignment,scenario,charge_policy=policy)
                result=sim.run()
                assert result['charger_violations']==0
                sim.assert_invariants()
                checked+=1
    print(f'Passed: {checked} stress runs')
if __name__=='__main__': main()
