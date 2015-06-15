from morse.builder.creator import SensorCreator

class KeySensor(SensorCreator):
    def __init__(self, name=None):
        SensorCreator.__init__(self, name, \
                               "baldr.sensors.keysensor.KeySensor",\
                               "KeySensor")

