"""
Module for interacting with MuJoCo using its Python API.
"""

# Import MuJoCo modules
import mujoco

def load_model(xml_path):
    """Load a MuJoCo model from an XML file."""
    return mujoco.MjModel.from_xml_path(xml_path)
