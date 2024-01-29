import Rhino.Geometry as rg
import random
import math

boundarySize = 20

class ParticleSystem:
    
    def __init__(self, particleCount):
        self.Particles = []
        for i in range(0, particleCount):
            newParticle = Particle()
            newParticle.ParticleSystem = self
            self.Particles.append(newParticle)
    
    def Update(self):
        for particle in self.Particles:
            particle.Calculate()
            
        for particle in self.Particles:
            particle.Update()

class Particle:

    def __init__(self):
        # Initilize the Position to a random point
        self.Position = rg.Point3d(random.uniform(0, boundarySize), random.uniform(0, boundarySize), 0.0)
        self.Maxspeed = 0.1
        self.Maxforce = 0.2
        
        # Initilize the Velocity to a random 2D Vector of length 0.1
        alpha = random.uniform(0, 6.28)
        self.Velocity = 0.1 * rg.Vector3d(math.cos(alpha), math.sin(alpha), 0.0)
        
        self.History = [self.Position]    
    
    def Calculate(self):        
        self.Wander()
        self.Align()
        self.Separate()
        self.Containment()
        
    def Update(self):
        self.Position += self.Velocity
        self.History.append(self.Position)
        
        if len(self.History) > 20:
            del self.History[0]
            
    def Align(self):
        neighborDistance = 4.0
        sum = rg.Vector3d.Zero
        count = 0
        for other in self.ParticleSystem.Particles:
            distance_to_neighbor = self.Position.DistanceTo(other.Position)
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
        
    def Separate(self):
        desiredSeparation = 2.0
        sum = rg.Vector3d.Zero
        count = 0
        for other in self.ParticleSystem.Particles:
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
                 steer *= self.Maxforce.Length / steer.Length
            self.Velocity += steer
        
    def Wander(self):
        self.Velocity.Rotate(random.uniform(-0.2, 0.2), rg.Vector3d.ZAxis)
        
    def Attract(self):        
        toAttractor = iAttractor - self.Position
        toAttractor *= 0.01 / toAttractor.Length
        self.Velocity += toAttractor
        self.Velocity *= 0.1 / self.Velocity.Length
        
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

# Main Script:

if iReset or not("myParticleSystem" in globals()):
    myParticleSystem = ParticleSystem(100)
else:
    myParticleSystem.Update()
    
# Output the visualization geometry
paths = [] 
for particle in myParticleSystem.Particles:
    paths.append(rg.PolylineCurve(particle.History))
oGeometry = paths

