
import pkg_resources, subprocess, sys, os, json
from git import Repo
from colorama import Fore 
from core.utils import *

class Package():
        def __init__(self, name, file):
            with open(file, 'r') as f:
                try:
                    contents = json.load(f)
                    self.python = file.split('.')[0] + '.py'
                    self.name = contents['name']
                    self.info = contents['info']
                    self.dependencies = contents["dependencies"]
                    self.callback = contents["callback"]
                    self.ros_message = contents["ros_message"]
                    
                except Exception as e:
                    logg(__name__, "ERROR", "Can't load {} - field is missing ".format(e))

        def __str__(self) -> str:
            return "* {} \n\t * info: {} \n\t * dependencies: {} \n\t * callback: {} \n\t * ros_message: {} \n\t * python: {} ".format(self.name, self.info, self.dependencies, self.callback, self.ros_message, self.python)


class Library():
    def clone_repos(self):
        for item in self.config.downloads.repos:
            repo = item[0]
            folder =  os.path.realpath(item[1])

            if not os.path.exists(folder):
                print("cloning {} to {}".format(repo, folder))
                Repo.clone_from(repo, folder)
            else:
                print("{} already exists, skipping".format(folder))

    def load_packages(self):
        library_dir = os.path.realpath('./library')
        library_folders = [f.path  for f in os.scandir(library_dir) if f.is_dir()]

        for full_path in library_folders:
            name = full_path.split("/").pop()
            file = full_path + '/{}.json'.format(name)

            if os.path.isfile(file):
                self.install_pip(name, file)
                self.packages[name] = Package(name, file)

            else:
                deeper_folders = [f.path  for f in os.scandir(full_path) if f.is_dir()]
                for deeper_folder in deeper_folders:
                    name = deeper_folder.split("/").pop()

                    if name != ".git":
                        json_filename = deeper_folder + '/{}.json'.format(name)
                        self.packages[name] = Package(name, json_filename)

                     
                        self.install_pip(name, json_filename)

    def install_pip(self, name, json_file):

        print("* installing pip dependencies for {}".format(name))
        for part in self.config.referenced:
            if part == name:
                with open(json_file) as json_file:
                    data = json.load(json_file)
                    for dep in data["dependencies"]:
                        if dep["type"] == "pip3":
                            pck = dep["package"]
            
                            if not self.is_installed(pck):
                                print("* installing {}".format(pck))
                                subprocess.check_call([sys.executable, "-m", "pip", "install", pck])

    def is_installed(self, name):
        return name in sorted(["%s" % (i.key) for i in pkg_resources.working_set])

 

    def __init__(self, config):
        print("{} \n ---------------------- \n LIBRARY \n ----------------------- \n {}".format(Fore.GREEN, Fore.RESET))
        print("{} * cloning repos .. {}".format(Fore.GREEN, Fore.RESET))
        
        self.packages = {}

        self.config = config
        
        self.clone_repos()

        print("{} * loading packages .. {}".format(Fore.GREEN, Fore.RESET))
        self.load_packages()

        for package in self.packages:
            print(self.packages[package])


    def has_package(self, package):
        return package in self.packages
    
    def get_package(self, package):
        return self.packages[package]
    