import unittest
from mas import Settings,Simulation,make_scenario
class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.cfg=Settings(width=12,height=12,n_robots=4,n_stations=2,n_orders=8,arrivals=15,max_ticks=500,seed=3,obstacles=.04)
        self.scenario=make_scenario(self.cfg)
    def test_identical_scenario(self):
        self.assertEqual(make_scenario(self.cfg),make_scenario(self.cfg))
    def test_modes(self):
        for mode in ('central','contract'):
            sim=Simulation(self.cfg,mode,self.scenario)
            res=sim.run()
            self.assertGreater(res['completed'],0)
            self.assertLessEqual(res['completed'],res['orders'])
            self.assertGreater(res['messages'],0)
    def test_contract_messages(self):
        sim=Simulation(self.cfg,'contract',self.scenario)
        sim.run()
        self.assertGreater(sim.message_counts['TASK_ANNOUNCE'],0)
        self.assertGreater(sim.message_counts['TASK_OFFER'],0)
    def test_central_no_auction(self):
        sim=Simulation(self.cfg,'central',self.scenario)
        sim.run()
        self.assertEqual(sim.message_counts['TASK_ANNOUNCE'],0)
        self.assertEqual(sim.message_counts['TASK_OFFER'],0)
    def test_duplicate_positions_never(self):
        sim=Simulation(self.cfg,'contract',self.scenario)
        for i in range(80):sim.step()
        self.assertEqual(len(set(r.position for r in sim.couriers.values())),len(sim.couriers))
    def test_invalid_configuration(self):
        with self.assertRaises(ValueError):Settings(width=2).validate()
if __name__=='__main__':unittest.main()
