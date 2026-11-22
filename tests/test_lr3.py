import unittest
from mas import Settings, Simulation, make_scenario
from experiments_lr3 import lower_initial_batteries
class ChargingConflictTests(unittest.TestCase):
    def make(self, policy):
        cfg=Settings(width=9,height=9,n_robots=3,n_stations=1,n_orders=2,
                     obstacles=0,coordination_delay=1)
        sim=Simulation(cfg,'central',charge_policy=policy)
        station=sim.chargers['C1']
        for rid in ('R1','R2','R3'):
            robot=sim.couriers[rid]
            robot.state='WAIT_CHARGE'
            robot.charger_id='C1'
        return sim,station
    def set_requests(self, sim, station):
        station.queue=['R1','R2','R3']
        station.queued_at={'R1':0,'R2':0,'R3':0}
        station.battery_at_request={'R1':.9,'R2':.1,'R3':.5}
        sim.clock=10
        for rid,charge in [('R1',90),('R2',10),('R3',50)]:
            sim.couriers[rid].battery=charge
            sim.couriers[rid].capacity=100
    def test_fifo_serves_first_arrival(self):
        sim,station=self.make('fifo')
        self.set_requests(sim,station)
        sim.station_step(station)
        self.assertEqual(station.occupant,'R1')
    def test_priority_serves_low_charge(self):
        sim,station=self.make('priority')
        self.set_requests(sim,station)
        sim.station_step(station)
        self.assertEqual(station.occupant,'R2')
        self.assertEqual(sim.urgent_waits,[10])
    def test_aging_prevents_permanent_overtaking(self):
        sim,station=self.make('priority')
        self.set_requests(sim,station)
        station.queue=['R1','R2']
        station.queued_at={'R1':0,'R2':150}
        sim.clock=150
        sim.station_step(station)
        self.assertEqual(station.occupant,'R1')
    def test_no_double_grant(self):
        sim,station=self.make('priority')
        self.set_requests(sim,station)
        sim.station_step(station)
        first=station.occupant
        sim.station_step(station)
        self.assertEqual(station.occupant,first)
        self.assertEqual(sim.charging_grants,1)
        sim.assert_invariants()
    def test_station_violation_counter_detects_bad_state(self):
        sim, station = self.make('fifo')
        sim.couriers['R1'].state = 'CHARGING'
        sim.couriers['R1'].charger_id = 'C1'
        with self.assertRaisesRegex(AssertionError, 'without matching station occupant'):
            sim.assert_invariants()
        self.assertGreater(sim.charger_violations, 0)
        self.assertEqual(sim.charger_constraint_checks, 1)
    def test_tiny_battery_with_default_reserve_rejected(self):
        cfg = Settings(battery_capacity=1)
        with self.assertRaisesRegex(ValueError, 'Battery capacity'):
            Simulation(cfg, 'central')
    def test_pending_charge_requests_reported(self):
        sim, station = self.make('fifo')
        sim.couriers['R1'].state = 'WAIT_CHARGE'
        station.queue = ['R1']
        station.queued_at = {'R1': 0}
        station.battery_at_request = {'R1': .5}
        sim.clock = 4
        station.occupant = 'R2'
        for order in sim.orders.values():
            order.status = 'COMPLETED'
            order.completed_at = 4
            order.assigned_at = 0
        sim.completed = len(sim.orders)
        result = sim.run()
        self.assertEqual(result['pending_charge_requests'], 1)
        self.assertGreaterEqual(result['pending_max_wait'], 4)
    def test_stale_request_removed(self):
        sim,station=self.make('priority')
        self.set_requests(sim,station)
        sim.couriers['R1'].state='IDLE'
        sim.station_step(station)
        self.assertNotIn('R1',station.queue)
        self.assertNotIn('R1',station.queued_at)
        self.assertNotIn('R1',station.battery_at_request)
    def test_invalid_policy_rejected(self):
        with self.assertRaises(ValueError):
            Simulation(Settings(),'central',charge_policy='random')
    def test_paired_scenario_and_input_immutability(self):
        cfg=Settings(n_robots=5,seed=71)
        scenario=lower_initial_batteries(make_scenario(cfg),cfg.battery_capacity)
        a=Simulation(cfg,'central',scenario,charge_policy='fifo')
        b=Simulation(cfg,'central',scenario,charge_policy='priority')
        self.assertEqual(a.scenario,b.scenario)
        self.assertEqual(tuple(r.battery for r in a.couriers.values()),
                         tuple(r.battery for r in b.couriers.values()))
        self.assertEqual(a.scenario.orders,b.scenario.orders)
    def test_full_pairs_safety(self):
        for seed in (0,3,10,17,25):
            cfg=Settings(width=14,height=14,n_robots=9,n_stations=2,
                         n_orders=15,max_ticks=350,seed=seed,
                         battery_capacity=100,obstacles=.08)
            scenario=lower_initial_batteries(make_scenario(cfg),cfg.battery_capacity)
            for policy in ('fifo','priority'):
                sim=Simulation(cfg,'central',scenario,charge_policy=policy)
                result=sim.run()
                self.assertEqual(result['charger_violations'],0)
                self.assertEqual(result['unresolved_in_transit'],0)
                sim.assert_invariants()
    def test_trace_contains_grant_policy(self):
        sim,station=self.make('priority')
        sim.trace_enabled=True
        self.set_requests(sim,station)
        sim.station_step(station)
        self.assertTrue(any(e['type']=='charge_granted' and
                            e['policy']=='priority' for e in sim.events))
    def test_results_reproducible(self):
        cfg=Settings(width=12,height=12,n_robots=6,n_stations=2,
                     n_orders=8,max_ticks=250,seed=33,obstacles=.04)
        scenario=lower_initial_batteries(make_scenario(cfg),cfg.battery_capacity)
        a=Simulation(cfg,'central',scenario,charge_policy='priority').run()
        b=Simulation(cfg,'central',scenario,charge_policy='priority').run()
        self.assertEqual(a,b)
if __name__=='__main__': unittest.main()
