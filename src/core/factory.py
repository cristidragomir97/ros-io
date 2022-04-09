import json, importlib, importlib.machinery, sys, threading, os, rospy 

from core.library import Library, Package
from core.config import Config, Part
from core.utils import *

class Subscriber(threading.Thread):
    def __init__(self,  topic, message, callback):
        threading.Thread.__init__(self)
        self.killed = False
        rospy.Subscriber(topic, message, callback)

    def run(self):
        rospy.spin()

class Publisher(threading.Thread):
    def __init__(self, topic, message, callback, queue_size=100, rate=60):
        threading.Thread.__init__(self)

        pub = rospy.Publisher(topic, message, queue_size=queue_size)

        self.pub = pub
        self.callback = callback
      
    def run(self):  
        while True:
            ret = self.callback()
            self.pub.publish(ret)

class Factory():
    def __init__(self, library, config):
      
        self.threads = {}

        self.conf = config
        self.lib = library
        
        self.conf.pretty_print()
        
        self._setup()


    def import_package(self, package_name, package_source, arguments):
        loader = importlib.machinery.SourceFileLoader(package_name, package_source)

        # imports the python source file in the current context 
        module = loader.load_module()

        # basically retrieves constructor for the device 
        instance = getattr(module, package_name)
                
        # runs constructor, unpacks arguments, and initialises object
        instance = instance(*list(arguments.values()))

        return instance

    def get_ros_msg(self, msg):
        message = importlib.import_module(msg[0])
        message = getattr(message, msg[1])
        return message

    def get_callback(self, instance, package_callback):
            return getattr(instance, package_callback)

    def _setup(self):

        for part in self.conf.get_part():

            pkg = self.lib.get_package(part.library)
            pkg_instance = self.import_package(pkg.name, pkg.python, part.args)
            msg_instance = self.get_ros_msg(pkg.ros_message)

            if part.multichannel is True:
                for ch in range(0, part.channel_no):
                    channel = part.get_channel(ch)
                    thread_name = part.name + "_channel_" + channel.pin
                    thread_name = thread_name.lower().replace(" ", "_")

                    if channel.role == "publisher":
                        worker = Publisher(channel.topic, msg_instance, self.get_callback(pkg_instance,  pkg.callback[ch]))

                    elif channel.role == "subscriber":
                        worker = Subscriber(channel.topic, msg_instance, self.get_callback(pkg_instance, pkg.callback[ch]))

                    self.threads[thread_name] = {
                        'thread': worker,
                        'info': {
                            'name': thread_name,   
                            'info': pkg.info,
                            'library': part.library,
                            'role': channel.role 
                        }
                    }

                    worker.start()
            
            else:
                thread_name = part.name.lower().replace(" ", "_"),   

                if part.role == "publisher":
                    worker = Publisher(part.topic, msg_instance, self.get_callback(pkg_instance, pkg.callback))

                elif part.role == "subscriber":
                    worker = Subscriber(part.topic, msg_instance, self.get_callback(pkg_instance, pkg.callback))

                self.threads[thread_name] = {
                        'thread': worker,
                        'info': {
                            'name': thread_name,
                            'info': pkg.info,
                            'library': part.library,
                            'role': part.role 
                        }
                }

                worker.start()

    
    
    def reload(self):
        for thread in self.threads.values():
            if thread is not None:
                if thread.stopped() is not True:
                    thread.stop() 

    def threads(self):
        return self.threads

