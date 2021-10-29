import random
import numpy as np

# things to adjust:
# max_speed
# perception
# max force
# starting velocity
# starting acceleration

class Boid():
    def __init__(self, x, y, width, height):
        self.width = width
        self.height = height
        self.x = np.array([x, y])
        # randomly set velocity to -5 < v < 5
        self.v = np.array([(random.random()-0.5) * 10, (random.random()-0.5) * 10])
        # randomly set acceleration to -0.25 < v < 0.25
        self.a = np.array([(random.random()-0.5) / 2 ,(random.random()-0.5) / 2 ])

        self.max_speed = 5
        self.perception = 100
        self.max_force = 0.3


    def update(self):
        # update positions and velocities according to Newton's laws
        self.x += self.v
        self.v += self.a

        # limit speed by setting velocity to max_speed if v > max_speed (but keeping direction the same)
        if np.linalg.norm(self.v) > self.max_speed:
            self.v = self.v / np.linalg.norm(self.v) * self.max_speed

        self.a = np.array([0, 0], dtype='float64')


    # keep the boids inside the box
    def edges(self):
        if self.x[0] > self.width:
            self.x[0] = 0
        if self.x[0] < 0:
            self.x[0] = self.width
        if self.x[1] > self.height:
            self.x[1] = 0
        if self.x[1] < 0:
            self.x[1] = self.height


    def apply_behaviour(self, boids):
        alignment = self.align(boids)
        cohesion = self.cohesion(boids)
        separation = self.separation(boids)
        self.a += alignment
        self.a += cohesion
        self.a += separation


    def align(self, boids):
        steering = np.array([0,0], dtype='float64')
        total = 0
        avg_v = np.array([0,0], dtype='float64')

        # basically this finds the average direction of boids within perception radius
        # and sets avg_velocity to max_speed but in this direction
        # then return steering which is the difference between this and the current velocity of the boid
        for boid in boids:
            if np.linalg.norm(boid.x - self.x) < self.perception:
                avg_v = np.add(avg_v, boid.v)
                total += 1
            if total > 0:
                avg_v /= total
                avg_v = avg_v / np.linalg.norm(avg_v) * self.max_speed
                steering = avg_v - self.v

            return steering


    def cohesion(self, boids):
        # move towards local center of mass
        steering = np.array([0,0], dtype='float64')
        total= 0
        center_of_mass = np.array([0,0], dtype='float64')

        for boid in boids:
            if np.linalg.norm(boid.x - self.x) < self.perception:
                center_of_mass += boid.x
                total += 1
        if total > 0:
            center_of_mass /= total
            vec_to_com = center_of_mass - self.x
            if np.linalg.norm(vec_to_com) > 0:
                vec_to_com = vec_to_com / np.linalg.norm(vec_to_com) * self.max_speed
            steering = vec_to_com - self.v
            if np.linalg.norm(steering) > self.max_force:
                steering = (steering / np.linalg.norm(steering)) * self.max_force
        return steering


    def separation(self, boids):
        steering = np.array([0,0], dtype='float64')
        total = 0
        avg_vector = np.array([0,0], dtype='float64')
        for boid in boids:
            distance = np.linalg.norm(boid.x - self.x)
            if self.x[0] != boid.x[0] or self.x[1] != boid.x[1] and distance < self.perception:
                diff = self.x - boid.x
                diff /= distance
                avg_vector += diff
                total += 1

        if total > 0:
            avg_vector /= total
            if np.linalg.norm(steering) > 0:
                avg_vector = avg_vector / np.linalg.norm(steering) * self.max_speed
                steering = avg_vector - self.v
                if np.linalg.norm(steering) > self.max_force:
                    steering = steering / np.linalg.norm(steering) * self.max_force

        return steering
