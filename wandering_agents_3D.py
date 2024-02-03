import Rhino.Geometry as rg
import random
import math


# Particle Class to populate with Agents
class ParticleSystem:
    
    def __init__(self, preyPopulation, predatorPopulation):
        self.Particles = []
        self.Attractor = rg.Point3d.Origin
        # populate with prey
        for i in range(0, preyPopulation):
            newParticle = Prey()
            newParticle.ParticleSystem = self
            self.Particles.append(newParticle)
        # populate with predators
        for i in range(0, predatorPopulation):
            newParticle = Predator()
            newParticle.ParticleSystem = self
            self.Particles.append(newParticle)
 
 # Update Agent System simultaneously    
    def Update(self):        
        distances = []  # matrix of all distances
        for particle in self.Particles:
            distanceList = []
            if len(distances) > 0:
                for d in distances:  # fill up lower triangle of matrix
                    distanceList.append(d[len(distances)])
            for j in range(len(distances), len(self.Particles)):  # only look at upper triangle
                distanceList.append(particle.Position.DistanceTo(self.Particles[j].Position))
            particle.Distances = distanceList
            distances.append(distanceList) 
            particle.Calculate()
        
            
        for particle in self.Particles:
            particle.Update()

# Parent Class to inherit Attributes and Methods all Fishes have in common
class Fish(object):
    
    def __init__(self):
        self.Position = rg.Point3d(random.uniform(0, boundarySize),\
                    random.uniform(0, boundarySize), random.uniform(0, boundarySize))
        self.Distances = []
        self.Maxspeed = 0
        self.Maxforce = 0
        self.Velocity = rg.Vector3d(0,0,0)
        self.History = [self.Position]
    
    def Calculate(self):
        pass
    
    def Update(self):
        self.Position += self.Velocity
        self.History.append(self.Position)
        
        if len(self.History) > 150:
            del self.History[0]
            
    def Containment(self):
        if self.Position.X < 0.0:
            self.Position.X = 0
            self.Velocity.X = -self.Velocity.X
        elif self.Position.X > boundarySize:
            self.Position.X = boundarySize
            self.Velocity.X = -self.Velocity.X
            
        if self.Position.Y < 0.0:
            self.Position.Y = 0
            self.Velocity.Y = -self.Velocity.Y
        elif self.Position.Y > boundarySize:
            self.Position.Y = boundarySize
            self.Velocity.Y = -self.Velocity.Y
            
        if self.Position.Z < 0.0:
            self.Position.Z = 0
            self.Velocity.Z = -self.Velocity.Z
        elif self.Position.Z > boundarySize:
            self.Position.Z = boundarySize
            self.Velocity.Z = -self.Velocity.Z
             
class Prey(Fish):  # inherits from object, so class type of instances can be checked in Ironpython 2.7

    def __init__(self):
            super(Prey, self).__init__() # super to be able to overwrite parent
            
            # movement limits
            self.Maxspeed = 0.05
            self.Flightspeed = 2 * self.Maxspeed
            self.Maxforce = 0.2
            
            # behavior weights
            self.wWander = 5.0
            self.wSchool = 1.0
            self.wAlign = 0.98
            self.wSeparate = 2.0
            self.wFlight = 1.0
            
            # initial velocity
            alpha = random.uniform(0, 6.28)
            self.Velocity = 0.1 * rg.Vector3d(math.cos(alpha), math.sin(alpha), -math.cos(alpha))
 
    def Calculate(self):        
        self.Wander(self.wWander)
        self.School(self.wSchool)
        self.Align(self.wAlign)
        self.Separate(self.wSeparate)
        self.Flight(self.wFlight)
        self.Attract(0.005)
        self.Containment()
                 
    def Align(self, weight):
        neighborDistance = 1
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            distance_to_neighbor = self.Distances[i]  # new implementation of diagonally checked distances
            if distance_to_neighbor > 0 and distance_to_neighbor < neighborDistance:
                sum += other.Velocity
                count += 1
        
        if count > 0:
            sum /= count
            sum.Unitize()
            sum *= self.Maxspeed
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce
            self.Velocity += (weight * steer)
            
    def Attract(self, weight):
        min_distance = 10.0
        distance_to_attractor = self.Position.DistanceTo(self.ParticleSystem.Attractor)
        if distance_to_attractor > min_distance:
            towards = self.ParticleSystem.Attractor - self.Position
            towards.Unitize()
            towards *= (distance_to_attractor - min_distance)
            steer = towards - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= (self.Maxforce / steer.Length)
            self.Velocity += (weight * steer)

    def Flight(self, weight):
        desiredSeparation = 5.0
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Predator':
                distance_to_neighbor = self.Distances[i]
                if distance_to_neighbor > 0 and distance_to_neighbor < desiredSeparation:
                    away = self.Position - other.Position
                    away.Unitize()
                    away /= distance_to_neighbor ** 2
                    sum += away
                    count += 1
        
        if count > 0:
            sum /= count
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce / steer.Length
            self.Velocity += (weight * steer)

    def School(self, weight):
        schoolingDistance = 15.0
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Prey':
                distance_to_neighbor = self.Distances[i]
                if distance_to_neighbor > 0 and distance_to_neighbor < schoolingDistance:
                    towards = other.Position - self.Position 
                    towards.Unitize()
                    towards *= (distance_to_neighbor / boundarySize)
                    sum += towards
                    count += 1
        
        if count > 0:
            sum /= count
            sum.Unitize()
            sum *= self.Maxspeed
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce / steer.Length
            self.Velocity += (weight * steer)        


    def Separate(self, weight):
        desiredSeparation = 0.5
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            distance_to_neighbor = self.Distances[i]
            if distance_to_neighbor > 0 and distance_to_neighbor < desiredSeparation:
                away = self.Position - other.Position
                away.Unitize()
                away /= distance_to_neighbor
                sum += away
                count += 1
        
        if count > 0:
            sum /= count
            sum.Unitize()
            sum *= self.Maxspeed
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce / steer.Length
            self.Velocity += (weight * steer)
                    
    def Wander(self, weight):
        self.Velocity.Rotate(weight * random.uniform(-0.2, 0.2), rg.Vector3d.ZAxis)
    
class Predator(Fish):  # inherits from object, so class type of instances can be checked in Ironpython 2.7

    def __init__(self):
        super(Predator, self).__init__() # super to be able to overwrite parent
        
        # movement limits
        self.Maxspeed = 0.025
        self.Attackspeed = 4 * self.Maxspeed
        self.Maxforce = 0.4
        
        # behavior weights
        self.wWander = 1.0
        self.wPursue = 1.0
        
        # initial velocity
        alpha = random.uniform(0, 6.28)
        self.Velocity = 0.1 * rg.Vector3d(math.cos(alpha), math.sin(alpha), -math.cos(alpha))
    
    def Calculate(self):        
        self.Wander(self.wWander)
        self.Pursue(self.wPursue)
        self.Containment()

    def Update(self):
        self.Kill() # makes sure that agents are only removed after all calculations finished
        
        self.Position += self.Velocity
        self.History.append(self.Position)
        
        if len(self.History) > 30:
            del self.History[0]  

    def Pursue(self, weight):
        pursueDistance = 5.0
        attackDistance = 0.75
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Prey':  # for whatever f***ing reason isinstance() doesn't work (⩺_
                distance_to_neighbor = self.Distances[i]
                if distance_to_neighbor >= attackDistance and distance_to_neighbor < pursueDistance:
                    towards = other.Position - self.Position
                    towards.Unitize()
                    towards /= distance_to_neighbor
                    sum += towards
                    count += 1                   
                
                elif distance_to_neighbor > 0 and distance_to_neighbor < attackDistance:
                    towards = (other.Position + other.Velocity) - self.Position
                    towards.Unitize()
                    towards /= distance_to_neighbor
                    sum += towards
                    count += 1
        
        if count > 0:
            sum /= count
            sum.Unitize()
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= (self.Maxforce / steer.Length)
            self.Velocity += (weight * steer)

    def Kill(self):
        neighborDistance = 0.1
        killProbability = 0.25
        survivors = []
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Prey':  # for whatever f***ing reason isinstance() doesn't work (⩺_⩹)
                distance_to_neighbor = self.Distances[i]
                if distance_to_neighbor < neighborDistance and killProbability > random.uniform(0.0, 1.0):
                    newParticle = Prey()
                    newParticle.ParticleSystem = self.ParticleSystem
                    survivors.append(newParticle)
                else:
                    survivors.append(other)
            else:
                survivors.append(other)
        self.ParticleSystem.Particles = survivors

    def Wander(self, weight):
        self.Velocity.Rotate(weight * random.uniform(-0.2, 0.2), rg.Vector3d.ZAxis)
 

# -------------------------------------------------------------------------------------------------------------------------
       
# MAIN SCRIPT:

# set size of boundary box
boundarySize = 30.0
attractorPt = rg.Point3d(boundarySize/2.0, boundarySize/2.0, boundarySize/2.0)

if iReset or not("myParticleSystem" in globals()):
    preyCount = 200
    predatorCount = 1
    myParticleSystem = ParticleSystem(preyCount, predatorCount)
    myParticleSystem.Attractor = attractorPt
else:
    myParticleSystem.Update()
    
# Output the visualization geometry
paths_Prey = [] 
paths_Predator = []
for particle in myParticleSystem.Particles:
    if 'Prey' in str(type(particle)):
        paths_Prey.append(rg.PolylineCurve(particle.History))
    else:
        paths_Predator.append(rg.PolylineCurve(particle.History))
boundingBox = rg.Box(rg.Plane.WorldXY, rg.Interval(0.0, boundarySize),\
    rg.Interval(0.0, boundarySize), rg.Interval(0.0, boundarySize)).ToBrep()
#paths += [edge.EdgeCurve for edge in boundingBox.Edges]
oPreyPath = paths_Prey
oPredatorPath = paths_Predator
oBoundaryBox = [edge.EdgeCurve for edge in boundingBox.Edges]
