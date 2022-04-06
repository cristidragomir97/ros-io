import json, sys, os, shutil 
from jsonschema import Draft7Validator
from colorama import Fore

from git import Repo

schema = {
    "file" :{
        "type" : "object",
        "properties" : {
            "name" : {"type" : "string"},
            "desc" : {"type" : "string"},
            "components": {"type" : "object"}
        },
    },

    "simple_guts" : {
        "type" : "object",
        "properties": {
                "role" : {"type": "string"},
                "topic" : {"type": "string"},
                "repo" : {"type": "string"},
                "library" : {"type": "string"},
                "address" : {"type": "string"},
                "args": {"type" : "object"}
        }, 
        "required": ["role", "topic", "repo", "library", "address"]
    }, 

    "complex_guts" : {
        "type" : "object",
        "properties": {
                "repo" : {"type": "string"},
                "library" : {"type": "string"},
                "address" : {"type": "string"},
                "channel_no": {"type": "number"},
                "channels": {"type" : "object"},
                "args": {"type" : "object"},
            }, 
            "required": ["repo", "library", "address",  "channel_no", "channels"]
    },

    "channel":{
        "type" : "object",
        "properties": {
            "role" : {"type" : "string"},
            "topic": {"type" : "string"},
            "channel": {"type" : "string"},
            "args": {"type" : "object"}
        },
        "required": ["role", "topic", "channel"]
    }
}

def _validate(data, schema, name, tabs):
        validator = Draft7Validator(schema)
        raw_errors = validator.iter_errors(data)
        err = list(raw_errors)

        prefix = ""
        for i in range(tabs):
            prefix += "\t"


        if len(err) == 0:
            prefix += "* "
            print(prefix + "{}:{} check {}".format(name, Fore.GREEN,  Fore.RESET))
            return True

        else: 
           
            print(prefix + "* {}:".format(name))
            for error in err:
                print(prefix + "\t {}{}{}".format(Fore.RED, error.message, Fore.RESET,))

            return False

class Channel():
    def __init__(self, obj, name):
        self.name = name
        self.role = obj["role"]
        self.channel = obj["channel"]
        self.topic = obj["topic"]
        self.args = obj["args"]

    def __str__(self):
        return "pin: {} \n\t\t * name: {} \n\t\t * role:{} \n\t\t * topic:{} \n\t\t * args: {} \n\t\t ".format(self.channel, self.name, self.role, self.topic, self.args)

class Component():

    def __init__(self, obj, name, multichannel=False):
            self.multichannel = multichannel
            self.name = name
            self.args = obj["args"]
            self.library = obj["library"]
            self.repo = obj["repo"]

            self.clone_repo()

            if multichannel: 
                self.channel_no = obj["channel_no"]
                self.channels = []
            else:  
                self.role = obj["role"]
                self.topic = obj["topic"]
                self.repo = obj["repo"]
                self.address = obj["address"]

    def clone_repo(self):
        folder = "./library/{}".format(self.name)
        if os.path.isdir(folder):
            shutil.rmtree(folder)
            print(os.path.realpath(folder))
            Repo.clone_from(self.repo, folder)

    def __str__(self):
        if self.multichannel:
            return "[{}] \n\t * library: {} \n\t * repository: {} \n\t * arguments: {} \n\t * channels: ".format(self.name, self.library, self.repo, self.args)
        else: 
            return "[{}] \n\t * library: {} \n\t * repository: {} \n\t * role: {} \n\t * topic: {} \n\t * arguments: {} ".format(self.name, self.library,  self.repo, self.role, self.topic, self.args)

class Config():

    def __init__(self, path):
        self.components = []
        with open(path, 'r') as f:
            self.instance = json.load(f)

            # VALIDATE THE FILE STRUCTURE
            if _validate(self.instance, schema["file"], "file", tabs=0) is True:
                self.validate_components()
    
    def pretty_print(self):
        for item in config.components:
            print(item)

            if item.multichannel:
                for channel in item.channels:
                    print("\t\t *", channel)



    def validate_components(self):
        componentz = self.instance["components"]

        # ITERATE TROUGH ALL COMPONENTS IN THE CONFIG FILE
        for item in componentz:
            thing = componentz[item]

            
            ## HANDLE MULTI CHANNEL COMPONENT 
            if "channel_no" in thing:
                
                # CREATE COMPONENT OBJECT
                component_obj = Component(obj = thing, name = item, multichannel = True)
    
                if _validate(thing, schema["complex_guts"], item, tabs = 1) is True:
                    ch = thing["channels"]

                    for channel in ch:
                        if _validate(ch[channel], schema["channel"], channel, tabs = 2) is True:

                            # CREATE CHANNEL OBJECT AND ADD TO THE COMPONENT OBJECT
                            channel_obj = Channel(obj = ch[channel], name = channel)
                            component_obj.channels.append(channel_obj)
                
                self.components.append(component_obj)

            ## HANDLE SINGLE CHANNEL COMPONENT 
            else:
                if _validate(thing, schema["simple_guts"], item, tabs = 1) is True:
                    ## create compoent here
                    component_obj = Component(obj = thing, name = item, multichannel = False)
                    self.components.append(component_obj)
                    
    def get_components(self):
        return self.components
