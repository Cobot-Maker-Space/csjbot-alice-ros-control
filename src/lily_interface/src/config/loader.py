
import os 
import yaml

class ConfigLoader(object):

    def __init__(self, state) -> None:
        super().__init__()

        self.load_config()
        self.config_type = 'main_connection' #default option - can be overridden for more complex configs 
        self.config_state = state # dev or prod as required

    def load_config(self):
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        config_yaml = open(os.path.join(__location__, 'robin.yaml'))
        self.config_yaml = yaml.load(config_yaml, Loader=yaml.FullLoader)
        
    def set_type(self, type):
        self.config_type = type

    def set_state(self, state):
        self.config_state = state

    def fetch_value(self, key):
        return self.config_yaml[self.config_type][self.config_state][key]

