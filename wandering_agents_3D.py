import Rhino.Geometry as rg
import random
import math

boundarySize = 10.0

class ParticleSystem:
    
    def __init__(self, preyPopulation, predatorPopulation):
        self.Particles = []
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
        
        if len(self.History) > 30:
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
            super(Prey, self).__init__()
            self.Maxspeed = 0.05
            self.Flightspeed = 2 * self.Maxspeed
            self.Maxforce = 0.2
            alpha = random.uniform(0, 6.28)
            self.Velocity = 0.1 * rg.Vector3d(math.cos(alpha), math.sin(alpha), -math.cos(alpha))
 
    def Calculate(self):        
        self.Wander()
        self.School()
        self.Align()
        self.Separate()
        self.Flight()
        self.Containment()
                 
    def Align(self):
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
            self.Velocity += steer

    def Flight(self):
        desiredSeparation = 1.0
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Predator':
                distance_to_neighbor = self.Position.DistanceTo(other.Position)
                if distance_to_neighbor > 0 and distance_to_neighbor < desiredSeparation:
                    away = self.Position - other.Position
                    away.Unitize()
                    away /= distance_to_neighbor
                    sum += away
                    count += 1
        
        if count > 0:
            sum /= count
            sum *= self.Flightspeed
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce / steer.Length
            self.Velocity += steer

    def School(self):
        schoolingDistance = 5.0
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == type(self).__name__:
                distance_to_neighbor = self.Position.DistanceTo(other.Position)
                if distance_to_neighbor > 0 and distance_to_neighbor < schoolingDistance:
                    towards = other.Position - self.Position 
                    towards.Unitize()
                    towards /= distance_to_neighbor
                    sum += towards
                    count += 1
        
        if count > 0:
            sum /= count
            sum.Unitize()
            sum *= self.Maxspeed
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce / steer.Length
            self.Velocity += steer        

    def Separate(self):
        desiredSeparation = 0.15
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            distance_to_neighbor = self.Position.DistanceTo(other.Position)
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
            self.Velocity += steer
                    
    def Wander(self):
        self.Velocity.Rotate(random.uniform(-0.2, 0.2), rg.Vector3d.ZAxis)
    
    # currently not used    
    # def Attract(self):        
    #     toAttractor = iAttractor - self.Position
    #     toAttractor *= 0.01 / toAttractor.Length
    #     self.Velocity += toAttractor
    #     self.Velocity *= 0.1 / self.Velocity.Length
    
class Predator(Fish):  # inherits from object, so class type of instances can be checked in Ironpython 2.7

    def __init__(self):
        super(Predator, self).__init__()
        self.Maxspeed = 0.025
        self.Attackspeed = 4 * self.Maxspeed
        self.Maxforce = 0.2
        alpha = random.uniform(0, 6.28)
        self.Velocity = 0.1 * rg.Vector3d(math.cos(alpha), math.sin(alpha), -math.cos(alpha))
    
    def Calculate(self):        
        self.Wander()
        #self.Align()
        #self.Separate()
        self.Attack()
        self.Containment()

    def Update(self):
        self.Kill() # makes sure that agents are only removed after all calculations finished
        
        self.Position += self.Velocity
        self.History.append(self.Position)
        
        if len(self.History) > 30:
            del self.History[0]  

    def Wander(self):
        self.Velocity.Rotate(random.uniform(-0.2, 0.2), rg.Vector3d.ZAxis)
        
    def Attack(self):
        attackDistance = 1.5
        sum = rg.Vector3d.Zero
        count = 0
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Prey':  # for whatever f***ing reason isinstance() doesn't work (⩺_
                distance_to_neighbor = self.Position.DistanceTo(other.Position)
                if distance_to_neighbor > 0 and distance_to_neighbor < attackDistance:
                    towards = other.Position - self.Position
                    towards.Unitize()
                    towards /= distance_to_neighbor
                    sum += towards
                    count += 1
        
        if count > 0:
            sum /= count
            sum.Unitize()
            sum *= self.Attackspeed
            steer = sum - self.Velocity
            if steer.Length > self.Maxforce:
                 steer *= self.Maxforce / steer.Length
            self.Velocity += steer
        else:
            self.Velocity.Unitize()
            self.Velocity *= self.Maxspeed
        
    def Kill(self):
        neighborDistance = 0.5
        killProbability = 0.25
        survivors = []
        for i, other in enumerate(self.ParticleSystem.Particles):
            if type(other).__name__ == 'Prey':  # for whatever f***ing reason isinstance() doesn't work (⩺_⩹)
                distance_to_neighbor = self.Position.DistanceTo(other.Position)
                if distance_to_neighbor < neighborDistance and killProbability > random.uniform(0.0, 1.0):
                    pass
                else:
                    survivors.append(other)
            else:
                survivors.append(other)
        self.ParticleSystem.Particles = survivors
       
# Main Script:
if iReset or not("myParticleSystem" in globals()):
    preyCount = 200
    predatorCount = 1
    myParticleSystem = ParticleSystem(preyCount, predatorCount)
else:
    myParticleSystem.Update()
# Output the visualization geometry
paths = [] 
paths2 = []
for particle in myParticleSystem.Particles:
    if 'Prey' in str(type(particle)):
        paths.append(rg.PolylineCurve(particle.History))
    else:
        paths2.append(rg.PolylineCurve(particle.History))
boundingBox = rg.Box(rg.Plane.WorldXY, rg.Interval(0.0, boundarySize),\
    rg.Interval(0.0, boundarySize), rg.Interval(0.0, boundarySize)).ToBrep()
paths += [edge.EdgeCurve for edge in boundingBox.Edges]
oGeometry = paths
oGeometry2 = paths2
