# sensor_processor/sensor_processor/actuator_controller.py

class ActuatorController:
    def __init__(self):
        self.is_active = False

    def activate(self):
        self.is_active = True
        print("Actuator activated!")

    def deactivate(self):
        self.is_active = False
        print("Actuator deactivated!")

    def toggle(self):
        self.is_active = not self.is_active
        if self.is_active:
            print("Actuator toggled to active!")
        else:
            print("Actuator toggled to inactive!")