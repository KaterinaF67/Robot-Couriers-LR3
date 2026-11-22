import unittest
from mas import Settings,Simulation,make_scenario
class TestStress(unittest.TestCase):
    def test_several_maps_and_modes(self):
        for seed in (0,1,2):
            cfg=Settings(width=14,height=13,n_robots=5,n_stations=2,n_orders=10,arrivals=25,max_ticks=400,seed=seed)
            sc=make_scenario(cfg)
            for mode in ('central','contract'):
                sim=Simulation(cfg,mode,sc)
                result=sim.run()
                self.assertGreaterEqual(result['completed'],0)
                self.assertLessEqual(result['completed'],10)
                self.assertEqual(len(set(r.position for r in sim.couriers.values())),5)
    def test_trace_created(self):
        cfg=Settings(n_orders=4,n_robots=3,seed=7,max_ticks=100)
        sim=Simulation(cfg,'contract',trace=True)
        sim.run()
        self.assertTrue(any(x['type']=='send' for x in sim.events))
if __name__=='__main__':unittest.main()
