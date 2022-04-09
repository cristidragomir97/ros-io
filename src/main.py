import rospy, argparse, sys, time, os
from pathlib import Path
from core.library import Library
from core.config import Config
from core.factory import Factory 
from core.utils import *

def arg_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args =  parser.parse_args()
    return os.getcwd() + "/" + args.config

    
if __name__ == "__main__":
    rospy.init_node("ros-io", disable_signals=True)

    conf = arg_parse()
    config = Config(conf)
    lib = Library(config)
    Factory(library=lib, config=config)
