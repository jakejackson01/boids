import pygame as pg
import random
from boid import Boid

# declare stuff
width, height = 500, 500
size = [width, height]
black = [0,0,0]
white = [255,255,255]

# create flock of Boids
flock = [Boid(random.random()*width, random.random()*height, width, height) for _ in range(30)]


# ----- START OF PYGAME ----- #
# this section plots the updated positions of all boids on the screen

pg.init()
screen = pg.display.set_mode(size)
screen.fill(white)


running = True
while running:

    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    # reset the screen
    screen.fill(white)

    # update positions of boids
    for boid in flock:
        boid.apply_behaviour(flock)
        boid.update()
        boid.edges()

    # redraw all boids in flock
    for boid in flock:
        pg.draw.circle(screen, black, (boid.x[0], boid.x[1]), 2)

    # refresh the screen
    pg.display.flip()

# ----- END OF PYGAME ----- #

# this is an edit