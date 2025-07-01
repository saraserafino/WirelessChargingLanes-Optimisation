class EV: # Electric Vehicles
    def __init__(self, scalability):
        self.B0 = 0.35 # initial energy
        self.energy = 0.35 # initial state of energy
        if scalability is True: # For Braess Network
            self.Bmax = 1.0 # battery capacity
        else: # For grid networks
            self.Bmax = 0.5
        # Traffic parameters
        self.Va = 900 # m/min, free-flow speed
        self.Wa = 450 # m/min, backward speed
        self.Ka = 0.12 # vehicles/min, jam density for each link 𝑎
        self.Qa = 36 # vehicles * m / min^2, maximum flow capacity
        self.Ca = 0.04 # vehicles/min, critical density

        self.omega = 0.33 # amount of energy received when traversing oncharging link per time unit
        self.epsilon = 0.00018 # electricity consumption rate
    

class ICV: # Internal Combustion Vehicles
    def __init__(self, scalability):
        # Traffic parameters
        self.Va = 900 # m/min, free-flow speed
        self.Wa = 450 # m/min, backward speed
        self.Ka = 0.12 # vehicles/min, jam density for each link 𝑎
        self.Qa = 36 # vehicles * m / min^2, maximum flow capacity
        self.Ca = 0.04 # vehicles/min, critical density