"""
Module for running MuJoCo simulations.
"""

import mujoco
from mujoco.viewer import launch

def run_simulation(xml_path):
    """Run a simulation using the specified XML file."""
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    launch(model, data)
