# Copyright (c) 2020, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest
import subprocess
import time
from mlp import LocoPlanner, Config
from utils import check_motion
from hpp.corbaserver.rbprm.utils import ServerManager

class TestTalosWalkSl1mQuasistatic(unittest.TestCase):
    def test_talos_walk_sl1m_quasistatic(self):
        cfg = Config()
        cfg.load_scenario_config("talos_flatGround_quasiStatic")
        cfg.contact_generation_method = "sl1m"
        # ~ cfg.centroidal_method = "quasistatic"
        cfg.IK_store_centroidal = True
        cfg.IK_store_zmp = True
        cfg.IK_store_effector = True
        cfg.IK_store_contact_forces = True
        cfg.IK_store_joints_derivatives = True
        cfg.IK_store_joints_torque = True
        cfg.ITER_DYNAMIC_FILTER = 0

        with ServerManager('hpp-rbprm-server'):
            loco_planner = LocoPlanner(cfg)
            loco_planner.run()

            check_motion(self, loco_planner)


if __name__ == '__main__':
    unittest.main()
