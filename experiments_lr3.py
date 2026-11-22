import csv
import json
import math
import statistics
from pathlib import Path
from mas import Settings, Simulation, make_scenario
from mas.core import Scenario
SCENARIOS = {
    'low': dict(width=16, height=16, n_robots=6, n_stations=2, n_orders=14,
                arrivals=80, max_ticks=750, battery_capacity=100, charging_rate=5,
                obstacles=.06),
    'medium': dict(width=20, height=20, n_robots=10, n_stations=1, n_orders=24,
                   arrivals=100, max_ticks=1100, battery_capacity=105, charging_rate=5,
                   obstacles=.08),
    'high': dict(width=24, height=24, n_robots=14, n_stations=2, n_orders=36,
                 arrivals=130, max_ticks=1400, battery_capacity=110, charging_rate=5,
                 obstacles=.10),
}
METRICS = ('completion_rate', 'mean_delivery_ticks', 'mean_assignment_ticks',
           'charging_grants', 'mean_queue_wait', 'max_queue_wait',
           'urgent_mean_wait', 'urgent_grants', 'queue_wait_jain',
           'charging_wait_ticks', 'max_charger_queue', 'charging_busy_ticks',
           'emergency_recoveries', 'messages', 'unfinished', 'charger_violations',
           'charger_constraint_checks', 'pending_charge_requests', 'pending_max_wait')
def lower_initial_batteries(scenario, capacity):
    batteries = tuple(round(capacity*(0.48 + 0.04*(i % 5)))
                      for i in range(len(scenario.courier_positions)))
    return Scenario(scenario.width, scenario.height, scenario.walls,
                    scenario.courier_positions, scenario.charger_positions,
                    scenario.orders, batteries, scenario.seed)
def conduct(directory='results_lr3', seeds=range(10), assignment_mode='central'):
    if assignment_mode not in ('central', 'contract'):
        raise ValueError('unknown assignment mode')
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    records = []
    for name, settings in SCENARIOS.items():
        for seed in seeds:
            cfg = Settings(**settings, scenario=name, seed=seed)
            baseline = make_scenario(cfg)
            scenario = lower_initial_batteries(baseline, cfg.battery_capacity)
            for policy in ('fifo', 'priority'):
                sim = Simulation(cfg, assignment_mode, scenario,
                                 charge_policy=policy)
                row = sim.run()
                records.append(row)
                print(f'{name:6s} seed={seed:02d} {policy:8s} '
                      f'orders={row["completed"]}/{row["orders"]} '
                      f'grants={row["charging_grants"]} '
                      f'wait={row["mean_queue_wait"]} '
                      f'urgent={row["urgent_mean_wait"]}')
    with (out/'runs.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=records[0])
        writer.writeheader()
        writer.writerows(records)
    summary = []
    for scenario_name in SCENARIOS:
        for policy in ('fifo', 'priority'):
            group = [r for r in records if r['scenario'] == scenario_name
                     and r['charge_policy'] == policy]
            result = {'scenario':scenario_name, 'charge_policy':policy,
                      'n':len(group), 'assignment_mode':assignment_mode}
            for metric in METRICS:
                values=[float(r[metric]) for r in group if r[metric] is not None]
                result[metric+'_mean'] = round(statistics.mean(values), 4) if values else None
                result[metric+'_sd'] = round(statistics.stdev(values), 4) if len(values)>1 else 0
                result[metric+'_observations'] = len(values)
            summary.append(result)
    with (out/'summary.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f, fieldnames=summary[0])
        writer.writeheader()
        writer.writerows(summary)
    (out/'config.json').write_text(json.dumps({
        'scenarios':SCENARIOS, 'seeds':list(seeds),
        'assignment_mode': assignment_mode,
        'charge_policies':['fifo','priority'],
        'initial_battery_rule':'capacity * (0.48 + 0.04*(robot_index % 5))',
        'priority_rule':'100*(1-battery/capacity)+0.8*queue_wait_ticks',
        'urgent_cutoff':'battery_at_request <= 35% of capacity',
        'metric_scope':'mean_queue_wait uses granted requests; pending requests and their maximum wait are reported separately'
    }, ensure_ascii=False, indent=2), encoding='utf-8')
    return records,summary
if __name__=='__main__':
    conduct()
