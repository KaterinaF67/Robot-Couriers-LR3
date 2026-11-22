import unittest
from mas import Settings, Simulation, make_scenario
from mas.core import Scenario, OrderSpec
def fixed_scenario(cfg, robots, stations, tasks, walls=()):
    return Scenario(cfg.width, cfg.height, frozenset(walls), tuple(robots),
                    tuple(stations), tuple(OrderSpec(*task) for task in tasks),
                    tuple([cfg.battery_capacity] * len(robots)), cfg.seed)
class RegressionTests(unittest.TestCase):
    def test_central_skips_impossible_first_order(self):
        cfg = Settings(width=8, height=8, n_robots=1, n_stations=1,
                       n_orders=2, battery_capacity=17, reserve=0,
                       obstacles=0, max_ticks=120)
        sc = fixed_scenario(cfg, [(0, 1)], [(0, 0)],
                            [('T001', (7, 7), (7, 6), 0),
                             ('T002', (1, 1), (1, 2), 0)])
        s = Simulation(cfg, 'central', sc)
        result = s.run()
        self.assertEqual(s.orders['T002'].status, 'COMPLETED')
        self.assertGreaterEqual(result['completed'], 1)
    def test_contract_does_not_reannounce_every_tick(self):
        cfg = Settings(width=8, height=8, n_robots=1, n_stations=1,
                       n_orders=1, battery_capacity=2, reserve=0,
                       obstacles=0, max_ticks=110)
        sc = fixed_scenario(cfg, [(0, 1)], [(0, 0)],
                            [('T001', (7, 7), (7, 6), 0)])
        s = Simulation(cfg, 'contract', sc)
        s.run()
        self.assertLess(s.message_counts['TASK_ANNOUNCE'], 15)
    def test_inputs_are_immutable(self):
        cfg = Settings(n_orders=2)
        sc = make_scenario(cfg)
        with self.assertRaises(AttributeError):
            sc.width = 4
        with self.assertRaises(AttributeError):
            sc.orders[0].pickup = (0, 0)
    def test_scenario_repeated_seed(self):
        for seed in (0, 1, 2, 3, 42, 99):
            cfg = Settings(seed=seed)
            self.assertEqual(make_scenario(cfg), make_scenario(cfg))
    def test_separate_agents_have_separate_mutable_containers(self):
        cfg = Settings(n_robots=2)
        s = Simulation(cfg, 'central')
        agents = list(s.couriers.values())
        agents[0].route.append((1, 2))
        self.assertEqual(agents[1].route, [])
    def test_ack_and_completion_cannot_duplicate_orders(self):
        cfg = Settings(width=9, height=9, n_robots=2, n_stations=1,
                       n_orders=5, obstacles=0, max_ticks=400)
        for mode in ('central', 'contract'):
            s = Simulation(cfg, mode)
            s.run()
            self.assertEqual(s.completed,
                             sum(o.status == 'COMPLETED' for o in s.orders.values()))
            self.assertEqual(s.completed,
                             len(set(o.spec.id for o in s.orders.values()
                                     if o.status == 'COMPLETED')))
    def test_each_sent_message_is_counted(self):
        cfg = Settings(n_orders=5, n_robots=3, max_ticks=400)
        for mode in ('central', 'contract'):
            s = Simulation(cfg, mode)
            s.run()
            self.assertEqual(sum(s.message_counts.values()), s.next_message_id - 1)
            self.assertGreater(s.message_bytes, 0)
    def test_recovery_is_visible_and_not_silent(self):
        cfg = Settings(width=8, height=8, n_robots=1, n_stations=1,
                       n_orders=1, max_ticks=100, obstacles=0)
        sc = fixed_scenario(cfg, [(0, 1)], [(0, 0)],
                            [('T001', (5, 5), (6, 6), 0)])
        s = Simulation(cfg, 'central', sc, trace=True)
        r = s.couriers['R1']
        r.battery = 0
        s.emergency_recovery(r, 'test')
        self.assertEqual(r.state, 'RECOVERING')
        self.assertEqual(s.recoveries, 1)
        self.assertTrue(any(e['type'] == 'emergency_recovery' for e in s.events))
        for _ in range(16):
            s.step()
        self.assertGreater(r.battery, 0)
        self.assertNotEqual(r.state, 'RECOVERING')
    def test_no_collisions_and_bounded_battery_every_tick(self):
        cfg = Settings(width=14, height=14, n_robots=12, n_stations=3,
                       n_orders=25, obstacles=.10, max_ticks=300, seed=25)
        sc = make_scenario(cfg)
        for mode in ('central', 'contract'):
            s = Simulation(cfg, mode, sc)
            for _ in range(300):
                s.step()
                s.assert_invariants()
    def test_message_delay_positive(self):
        cfg = Settings(n_orders=1, coordination_delay=3)
        s = Simulation(cfg, 'central')
        s.send('TEST', 'D', 'R1')
        msg = s.in_flight[-1]
        self.assertEqual(msg.delivered_at - msg.sent_at, 4)
        s.deliver()
        self.assertFalse(s.mailbox['R1'])
        s.clock = 4
        s.deliver()
        self.assertEqual(len(s.mailbox['R1']), 1)
    def test_station_queue_unique(self):
        cfg = Settings(n_orders=2, n_robots=3)
        s = Simulation(cfg, 'central')
        c = s.chargers['C1']
        r = s.couriers['R1']
        r.state = 'WAIT_CHARGE'
        r.charger_id = 'C1'
        s.send('CHARGE_REQUEST', 'R1', 'C1')
        s.send('CHARGE_REQUEST', 'R1', 'C1')
        s.clock = 2
        s.deliver()
        s.station_step(c)
        self.assertEqual(c.occupant, 'R1')
        self.assertEqual(c.queue, [])
    def test_no_auction_in_central(self):
        cfg = Settings(n_orders=5, max_ticks=500)
        s = Simulation(cfg, 'central')
        s.run()
        self.assertEqual(s.message_counts['TASK_ANNOUNCE'], 0)
        self.assertEqual(s.message_counts['TASK_OFFER'], 0)
    def test_full_auction_in_contract(self):
        cfg = Settings(n_orders=5, max_ticks=500)
        s = Simulation(cfg, 'contract')
        s.run()
        self.assertGreater(s.message_counts['TASK_ANNOUNCE'], 0)
        self.assertGreater(s.message_counts['TASK_OFFER'], 0)
        self.assertGreater(s.message_counts['TASK_ACK'], 0)
    def test_order_requeued_on_emergency(self):
        cfg = Settings(width=10, height=10, n_robots=1, n_stations=1,
                       n_orders=1, obstacles=0)
        sc = fixed_scenario(cfg, [(1, 1)], [(0, 0)],
                            [('T001', (3, 3), (4, 4), 0)])
        s = Simulation(cfg, 'central', sc)
        r = s.couriers['R1']
        o = s.orders['T001']
        r.order_id = o.spec.id
        r.state = 'TO_DROPOFF'
        o.robot = r.id
        o.status = 'IN_PROGRESS'
        s.emergency_recovery(r, 'test')
        self.assertEqual(o.status, 'REQUEUED')
        self.assertIsNone(o.robot)
        self.assertIsNone(r.order_id)
    def test_bad_parameters(self):
        for kwargs in [dict(width=3), dict(obstacles=.9), dict(n_robots=0),
                       dict(battery_capacity=0), dict(coordination_delay=-1)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Settings(**kwargs).validate()
if __name__ == '__main__':
    unittest.main()
