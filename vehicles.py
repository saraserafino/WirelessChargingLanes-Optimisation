# Parent class
class vehicle:
    def __init__(self):
        # Traffic parameters
        self.Va = 900 # m/min, free-flow speed
        self.Wa = 450 # m/min, backward speed
        self.Ka = 0.12 # vehicles/min, jam density for each link 𝑎
        self.Qa = self.Ka * self.Wa * self.Va / (self.Va + self.Wa) # = 36 vehicles * m / min^2, maximum flow capacity
        self.Ca = self.Ka * self.Wa / (self.Va + self.Wa) # = 0.04 vehicles/min, critical density
        self.Da = 36 # vehicles/min, demand rate

# Child classes inheriting from vehicle
        
class EV(vehicle): # Electric Vehicles
    def __init__(self, scalability):
        super().__init__() # calls parent's constructor

        self.B0 = 0.35 # initial battery energy
        # Battery capacity is 1 for grid networks and 0.5 for Braess networks
        self.Bmax = 1.0 if scalability else 0.5

        self.omega = 0.33 # kwh/min, amount of energy received when traversing oncharging link per time unit
        self.epsilon = 1.8e-4 # kwh/m, electricity consumption rate
   
class ICV(vehicle): # Internal Combustion Vehicles
    def __init__(self):
        super().__init__() # calls parent's constructor