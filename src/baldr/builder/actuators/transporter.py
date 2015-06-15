from morse.builder.creator import ActuatorCreator

class Transporter(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "baldr.actuators.transporter.Transporter",\
                                 "transporter")

