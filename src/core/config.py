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
            "downloads": {"type" : "object"},
            "parts": {"type" : "object"}
        },
    },

<<<<<<< Updated upstream
    "simple_guts" : {
=======
    "downloads": {
        "type" : "object",
        "properties" : {
            "repos" : {"type" : "array"},
            "install_deps" : {"type" : "string"},
        }
    },

    "single-channel" : {
>>>>>>> Stashed changes
        "type" : "object",
        "properties": {
                "role" : {"type": "string"},
                "topic" : {"type": "string"},
                "folder" : {"type": "string"},
                "library" : {"type": "string"},
                "address" : {"type": "string"},
                "args": {"type" : "object"}
        }, 
        "required": ["role", "topic", "library",  "folder", "address"]
    }, 

<<<<<<< Updated upstream
    "complex_guts" : {
=======
    "multi-channel" : {
>>>>>>> Stashed changes
        "type" : "object",
        "properties": {
                "folder" : {"type": "string"},
                "library" : {"type": "string"},
                "address" : {"type": "string"},
                "channel_no": {"type": "number"},
                "channels": {"type" : "object"},
                "args": {"type" : "object"},
            }, 
            "required": ["library", "folder", "address",  "channel_no", "channels"]
    },

    "channel":{
        "type" : "object",
        "properties": {
            "role" : {"type" : "string"},
            "topic": {"type" : "string"},
            "pin": {"type" : "string"},
            "args": {"type" : "object"}
        },
        "required": ["role", "topic", "pin"]
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

class Downloads():
    def __init__(self, obj):
        self.repos = obj["repos"]
        self.install_deps = obj["install_deps"]

class Channel():
    def __init__(self, obj, name):
        self.name = name
        self.role = obj["role"]
        self.pin = obj["pin"]
        self.topic = obj["topic"]
        self.args = obj["args"]

    def __str__(self):
        return "pin: {} \n\t\t * name: {} \n\t\t * role:{} \n\t\t * topic:{} \n\t\t * args: {} \n\t\t ".format(self.pin, self.name, self.role, self.topic, self.args)

class Part():

    def __init__(self, obj, name, multichannel=False):
            self.multichannel = multichannel
            self.name = name
            self.args = obj["args"]
            self.library = obj["library"]
            self.folder = obj["folder"]
            

            if multichannel: 
                self.channel_no = obj["channel_no"]
                self.channels = []
            else:  
                self.role = obj["role"]
                self.topic = obj["topic"]
                self.address = obj["address"]
    
    def get_channel(self, n):
        return self.channels[n]
        

    def __str__(self):
        if self.multichannel:
            return "[{}] \n\t * library: {} \n\t * repository: {} \n\t * arguments: {} \n\t * channels: ".format(self.name, self.library, self.folder, self.args)
        else: 
            return "[{}] \n\t * library: {} \n\t * repository: {} \n\t * role: {} \n\t * topic: {} \n\t * arguments: {} ".format(self.name, self.library,  self.folder, self.role, self.topic, self.args)

class Config():

    def __init__(self, path):
        self.parts = []
        self.downloads = []
        with open(path, 'r') as f:
            self.instance = json.load(f)

            # VALIDATE THE FILE STRUCTURE
            if _validate(self.instance, schema["file"], "file", tabs=0) is True:
                self.validate_parts()
                self.validate_downloads()
    
    def pretty_print(self):
        for item in self.parts:
            print(item)

            if item.multichannel:
                for channel in item.channels:
                    print("\t\t *", channel)

    def validate_downloads(self):
        if _validate(self.instance["downloads"], schema["downloads"], "downloads", tabs=0) is True:
            self.downloads = Downloads(self.instance["downloads"])
        else:
            print("Downloads are not valid")

    def validate_parts(self):
        partz = self.instance["parts"]

        # ITERATE TROUGH ALL COMPONENTS IN THE CONFIG FILE
        for item in partz:
            thing = partz[item]

            
            ## HANDLE MULTI CHANNEL COMPONENT 
            if "channel_no" in thing:
                
                # CREATE COMPONENT OBJECT
                part_obj = Part(obj = thing, name = item, multichannel = True)
    
<<<<<<< Updated upstream
                if _validate(thing, schema["complex_guts"], item, tabs = 1) is True:
=======
                if _validate(thing, schema["multi-channel"], item, tabs = 1) is True:
>>>>>>> Stashed changes
                    ch = thing["channels"]

                    for channel in ch:
                        if _validate(ch[channel], schema["channel"], channel, tabs = 2) is True:

                            # CREATE CHANNEL OBJECT AND ADD TO THE COMPONENT OBJECT
                            channel_obj = Channel(obj = ch[channel], name = channel)
                            part_obj.channels.append(channel_obj)
                
                self.parts.append(part_obj)

            ## HANDLE SINGLE CHANNEL COMPONENT 
            else:
<<<<<<< Updated upstream
                if _validate(thing, schema["simple_guts"], item, tabs = 1) is True:
=======
                if _validate(thing, schema["single-channel"], item, tabs = 1) is True:
>>>>>>> Stashed changes
                    ## create compoent here
                    part_obj = Part(obj = thing, name = item, multichannel = False)
                    self.parts.append(part_obj)
                    
<<<<<<< Updated upstream
    def get_components(self):
        return self.components
=======
    def get_part(self):
        return self.parts
>>>>>>> Stashed changes
