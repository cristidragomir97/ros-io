import rospy, argparse, sys, time, os
from pathlib import Path
from core.library import Library
from core.config import Config
from core.factory import Factory 
from core.utils import *
from git import Repo
    
def git_clone(repo, folder):
    if not os.path.exists(folder):
        print("cloning {} to {}".format(repo, folder))
        Repo.clone_from(repo, folder)
    else:
        print("{} already exists, skipping".format(folder))

    
if __name__ == "__main__":
    print('I2C BUS:', scan_bus())

    config_repo = os.getenv('CONFIG_REPO')
    config_file = os.getenv('CONFIG_FILE')
    print(config_repo, config_file)

    if config_file is not None:
        print("* loading config from volume: {}".format(config_file))
        conf = config_file
    elif config_repo  is not None:
        print("* config file not found. downloading from github: {}".format(config_file))
        git_clone(config_repo, os.getcwd() + '/config')
        conf = os.getcwd() + '/config/config.json'
        print(conf)

    rospy.init_node("robotblock", disable_signals=True)

    
    config = Config(conf)
    lib = Library(config)
    Factory(library=lib, config=config)
