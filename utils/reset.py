"""This file should be used to reset the config file to its default values"""
import yaml
from pathlib import Path

def reset_perspective() :
    """Resets the perspective transformation to its default values"""
    
    config = yaml.safe_load(open(str(Path(__file__).parent.parent / "config" / "board.yaml")))
    config["img"]["perspective"]["pts1"] = [[0, 0], [0, 0], [0, 0], [0, 0]]
    config["img"]["perspective"]["pts2"] = [[0, 0], [0, 0], [0, 0], [0, 0]]
    with open(str(Path(__file__).parent.parent / "config" / "board.yaml"), "w") as f :
        yaml.dump(config, f)
    return True