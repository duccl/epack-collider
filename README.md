# epuck-collider
Boxes check via E-Pack robot

# Objective
Complete the required exercise made by lecture CC7711 - INTELIGENCIA ARTIF.E ROBOTICA at [Centro Universtiário FEI](https://portal.fei.edu.br/).

It is based on move a box if it is light, was made using [Webots](https://cyberbotics.com/) and [E-Puck](http://www.e-puck.org/) robot

# Logic

To determine if a box is ligth, after the collision of the robot with some box on the world we check all boxes positions (that were defined previously and updated on the fly when it changes). If some have changed their position, very well, you hit a light box! So just fire up the leds and keep moving around...
