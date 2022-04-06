import json, sys, os, shutil
from core.utils import *
import subprocess
from git import Repo


class Channel():
    def __init__(self):
        self.name = name
        self.role = role
        self.pin = pin
        self.topic = topic
        self.args = args


class Device():

    def handle_channel(self, obj, item):
        _name = obj["channels"][item]["name"]
        _role = obj["channels"][item]["role"]
        _pin = obj["channels"][item]["channel"]
        _topic = obj["channels"][item]["topic"]
        _args = obj["channels"][item]["args"]
        channel_obj = Channel(_name, _role, _pin, _topic, _args)
        self.channels.append(channel_obj)

    def clone_repo(self):
        folder = "./library/{}".format(self.name)
        if os.path.isdir(folder):
            shutil.rmtree(folder)
            print(os.path.realpath(folder))
            Repo.clone_from(self.repo, folder)
        
    def __init__(self, obj):
        
        self.name = obj["name"]
        self.library = obj["library"]
        self.args = obj["args"]
        self.repo = obj["repo"]
        self.channels = []

        try:
            # check if we are dealing with a 
            if ("channel_no" in obj):
                self.channel_no = int(obj["channel_no"])

                for ch in range(0, self.channel_no):
                    self.handle_channel(obj, ch)

            else:
                self.channel_no = 0
                self.role = obj["role"]
                self.topic = obj["topic"]  
    
            self.clone_repo()

        except KeyError as k:
            formatted = json.dumps(obj, indent=4)   
            logg(__name__, "ERROR", "Can't load device. Field {} is missing.\n{}\n  ".format(k, formatted))

    def get_channel(self, n):
        return self.channels[n]


class Config():

    def __init__(self,  _file):
        with open(_file) as f:
            self.contents = json.load(f)[0]
            logg(__name__, "INFO", "loading configuration file: {}".format(_file))
            _, self.dev  = self.parse(self.contents)
            print(_, self.dev)
           
    
    def parse(self, contents):
        err = []
        dev = []

        try:
    
                components = contents["components"]
                for element in components:
                    dev.append(Device(element))
                
                print(dev)


        except Exception as e :
            logg(__name__, "WARNING","Can't load configuration item - field {} is missing ".format(e))

            err.append(
                json.dumps(e,
                    default = lambda o: o.__dict__,
                    sort_keys = True,
                    indent = 4)
            )

        return err, dev

    def pretty_print(self):

        for dev in self.dev:
            
            if dev.channel_no > 0:
                logg(__name__, "INFO", "[DEVICE]: {}, {}, {}".format(dev.name, dev.library, dev.args))

                for channel in dev.channels:
                    logg(__name__, "INFO", "[CHANNEL{}][{}, {}, {}".format(channel.pin, channel.name, channel.role, channel.topic))
            else:
                logg(__name__, "INFO", "[DEVICE]: {}, {}, {}, {}, {}".format(dev.name, dev.library, dev.role, dev.topic, dev.args))

    def get_devices(self):
        return self.dev
