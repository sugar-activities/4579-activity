#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# SprayPlay
# Copyright (C) 2008, Brian Jordan, Gregory Jordan, Eric Jordan
# Copyright (C) 2012, Alan Aguiar
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Contact information:
# Brian Jordan <bcjordan@gmail.com>
# Gregory Jordan <gjuggler@gmail.com>
# Eric Jordan <ewjordan@gmail.com>
# Alan Aguiar <alanjas@gmail.com>

from __future__ import division

import os
import gtk
import pygame
import random
from math import *
from pygame import *

# To win
PUCKS_TO_WIN = 5

# Bullet constants
BULLET_RADIUS = 10.0
BULLET_SPEED = 4
BULLET_MASS = 0.2
AMMO_PER_PLAYER = 12

# Player rotation constants
PLAYER_ARM_LENGTH = 20.0
MAX_ROTATION = 80
ROTATION_SPEED = 10
ROTATION_DECAY = .4

# Reload constants
RELOAD_DELAY = 10
RELOAD_RANDOM = .4

# Playing field constants
GRAVITY_FORCE = .015
WALL_DAMPING = .6
WALL_FRICTION = .07
COLLECTION_WIDTH = 100
MAX_COLLECTION_VELOCITY = 100

# No strict constants
SCREEN_WIDTH = 0
SCREEN_HEIGHT = 0
CIRCLE_BORDER = 0
CIRCLE_RADIUS = 0
CIRCLE_LEFT = 0
CIRCLE_RIGHT = 0
GRAVITY_DIST = 0
SCALE = 1


def data_path(f):
    return os.path.abspath(os.path.join('./data/', f))

class StopGame(Exception):
    pass
    
def angleToDirection(angle):
    return [cos(angle),sin(angle)]

def lineNormal(la,lb):
    "Returns normal vector to line"
    lx = (lb[0]-la[0])/100.0 #for numerical reasons we divide down first, then normalize
    ly = (lb[1]-la[1])/100.0
    invnorml = 1.0/sqrt(lx*lx+ly*ly) #will blow up if two vertices are in same place!
    nx = ly*invnorml
    ny = -lx*invnorml
    return [nx,ny]

def consumeBullet(bullet,player):
    "Consumes a bullet from the playing field, sending it either to player a or b's stash"
    bullet.kill()
    player.collectBall(bullet)
    

def containBullet(bullet):
    global players
    if (bullet.y > CIRCLE_BORDER and bullet.y < SCREEN_HEIGHT-CIRCLE_BORDER and bullet.x > CIRCLE_BORDER+CIRCLE_RADIUS and bullet.x < SCREEN_WIDTH-CIRCLE_BORDER-CIRCLE_RADIUS):
        return
    xleft = (CIRCLE_BORDER+CIRCLE_RADIUS)
    dxleft = bullet.x - xleft
    xright = (SCREEN_WIDTH-CIRCLE_BORDER-CIRCLE_RADIUS)
    dxright = bullet.x - xright
    dyleft = bullet.y - (CIRCLE_BORDER+CIRCLE_RADIUS)
    dyright = dyleft
    dleftsqr = dxleft*dxleft+dyleft*dyleft
    drightsqr = dxright*dxright+dyright*dyright
    if dleftsqr < CIRCLE_RADIUS*CIRCLE_RADIUS or drightsqr < CIRCLE_RADIUS*CIRCLE_RADIUS:
        return
    if bullet.x < xleft:
        if (bullet.y > SCREEN_HEIGHT/2 - COLLECTION_WIDTH/2 and bullet.y < SCREEN_HEIGHT/2 + COLLECTION_WIDTH/2):
            vnorm = sqrt(bullet.vx*bullet.vx+bullet.vy*bullet.vy)
            if vnorm < MAX_COLLECTION_VELOCITY:
                consumeBullet(bullet,players[0])
        dist = sqrt(dleftsqr)
        direction = [ dxleft/(dist), dyleft/(dist) ]
        bullet.x -= (dist+1-CIRCLE_RADIUS)*direction[0]
        bullet.y -= (dist+1-CIRCLE_RADIUS)*direction[1]
        bulldotdir = bullet.vx*direction[0]+bullet.vy*direction[1]
        bullet.vx += -(1.0+WALL_DAMPING)*bulldotdir*direction[0]
        bullet.vy += -(1.0+WALL_DAMPING)*bulldotdir*direction[1]
        bullet.vx *= 1.0-WALL_FRICTION
        bullet.vy *= 1.0-WALL_FRICTION
    elif bullet.x > xright:
        if (bullet.y > SCREEN_HEIGHT/2 - COLLECTION_WIDTH/2 and bullet.y < SCREEN_HEIGHT/2 + COLLECTION_WIDTH/2):
            vnorm = sqrt(bullet.vx*bullet.vx+bullet.vy*bullet.vy)
            if vnorm < MAX_COLLECTION_VELOCITY:
                consumeBullet(bullet,players[1])
        dist = sqrt(drightsqr)
        direction = [dxright/(dist), dyright/(dist) ]
        bullet.x -= (dist+1-CIRCLE_RADIUS)*direction[0]
        bullet.y -= (dist+1-CIRCLE_RADIUS)*direction[1]
        bulldotdir = bullet.vx*direction[0]+bullet.vy*direction[1]
        bullet.vx += -(1.0+WALL_DAMPING)*bulldotdir*direction[0]
        bullet.vy += -(1.0+WALL_DAMPING)*bulldotdir*direction[1] 
        bullet.vx *= 1.0-WALL_FRICTION
        bullet.vy *= 1.0-WALL_FRICTION
    elif bullet.y < CIRCLE_BORDER:
        bullet.y = CIRCLE_BORDER+1
        bullet.vy = abs(bullet.vy)*WALL_DAMPING
        bullet.vx = (1.0-WALL_FRICTION)*bullet.vx
    else:
        bullet.y = SCREEN_HEIGHT - CIRCLE_BORDER-1
        bullet.vy = -abs(bullet.vy)*WALL_DAMPING
        bullet.vx = (1.0-WALL_FRICTION)*bullet.vx

def applyGravity(bullet):
    if bullet.x < GRAVITY_DIST:
        bullet.vx -= GRAVITY_FORCE
    elif bullet.x > SCREEN_WIDTH-GRAVITY_DIST:
        bullet.vx += GRAVITY_FORCE
    elif bullet.x < SCREEN_WIDTH/2.0:
        bullet.vx -= GRAVITY_FORCE/5.0
    else: bullet.vx += GRAVITY_FORCE/5.0

    
def forceField(puck):
    #print CIRCLE_RADIUS

    if (puck[1] > CIRCLE_BORDER and puck[1] < SCREEN_HEIGHT-CIRCLE_BORDER and puck[0] > CIRCLE_BORDER+CIRCLE_RADIUS and puck[0] < SCREEN_WIDTH-CIRCLE_BORDER-CIRCLE_RADIUS):
   
        return [0.0,0.0]
    xleft = (CIRCLE_BORDER+CIRCLE_RADIUS)
    dxleft = puck[0] - xleft
    xright = (SCREEN_WIDTH-CIRCLE_BORDER-CIRCLE_RADIUS)
    dxright = puck[0] - xright
    dyleft = puck[1] - (CIRCLE_BORDER+CIRCLE_RADIUS)
    dyright = dyleft
    dleftsqr = dxleft*dxleft+dyleft*dyleft
    drightsqr = dxright*dxright+dyright*dyright
    if dleftsqr < CIRCLE_RADIUS*CIRCLE_RADIUS or drightsqr < CIRCLE_RADIUS*CIRCLE_RADIUS:
        return [0.0,0.0]

    #Now the puck is definitely outside of the arena.
    #There are three different cases to deal with, left section, middle section, and right section.
    dist = 0
    direction = [0.0,0.0]
    if puck[0] < xleft:
        dist = sqrt(dleftsqr) - CIRCLE_RADIUS
        direction = [ dxleft/(dist+CIRCLE_RADIUS), dyleft/(dist+CIRCLE_RADIUS) ]
    elif puck[0] > xright:
        dist = sqrt(drightsqr) - CIRCLE_RADIUS
        direction = [dxright/(dist+CIRCLE_RADIUS), dyright/(dist+CIRCLE_RADIUS) ]
        
    elif puck[1] < CIRCLE_BORDER:
        dist = abs(CIRCLE_BORDER - puck[1])
        direction = [0.0, -1.0]
        
    else:
        dist = abs(puck[1]-SCREEN_HEIGHT+CIRCLE_BORDER)
        direction = [0.0,1.0]

    factor = -.001*dist
    return [direction[0]*factor,direction[1]*factor]


class Background(pygame.sprite.Sprite):

    group = pygame.sprite.Group()

    def __init__(self, w, h):
        super(Background, self).__init__(Background.group)
        self.image_tmp = pygame.image.load(data_path('bg.png')).convert()
        self.image = pygame.transform.scale(self.image_tmp, (w, h))
        self.image_tmp = None
        self.rect = self.image.get_rect()

class Intro(pygame.sprite.Sprite):
    def __init__(self,base,num_images,delay):
        pygame.sprite.Sprite.__init__(self)

        self.num_images = num_images
        self.delay = delay
        self.demos = [] # Array of demo images
        for i in range(num_images):
            img = pygame.image.load(data_path(base + str(i) + '.png')).convert_alpha()
            self.demos.append(img)
        self.image = self.demos[0]
        self.rect = self.image.get_rect()
        self.update(0)

    def update(self,delta):
        img_num = pygame.time.get_ticks()//(self.delay)%(self.num_images)
        img = self.demos[img_num]
        self.image = img
        self.rect = img.get_rect()
        self.rect.centery = SCREEN_HEIGHT / 2
        self.rect.centerx = SCREEN_WIDTH / 2

class Overlay(pygame.sprite.Sprite):

    def __init__(self):
        super(Overlay, self).__init__()
        self.rect = pygame.Rect(0,0,SCREEN_WIDTH,SCREEN_HEIGHT)
        self.image = pygame.Surface(self.rect.size).convert_alpha()
        self.alpha = 150
        self.image.fill((255,255,255,self.alpha))

        self.dying = 0

    def update(self,delta):
        self.image.fill((255,255,255,self.alpha))
        if self.dying:
            self.alpha *= .9
        if self.alpha < 10:
            self.kill()

class Bullet(pygame.sprite.Sprite):
    
    group = pygame.sprite.Group()
    orig_image = 0
    color = (255,255,255)
    speed = 0.2
    radius = BULLET_RADIUS
    mass = BULLET_MASS
    
    def __init__(self, pos, direction):
        super(Bullet, self).__init__( Bullet.group)
#        self.rect = pygame.Rect(0, 0, 2*self.radius, 2*self.radius)
        self.image = pygame.transform.scale(Bullet.orig_image, (int(2*self.radius),int(2*self.radius)))
        self.rect = self.image.get_rect()
#        self.image = pygame.Surface(self.rect.size).convert_alpha()
#        self.image.fill((0,0,0,0))
#        pygame.draw.circle(self.image, self.color,self.rect.center,int(self.radius))
#        pygame.draw.circle(self.image,(150,150,150),self.rect.center,int(self.radius-2))
        self.rect.center = pos
        self.direction = direction
        self.vx = direction[0]*self.speed
        self.vy = direction[1]*self.speed
        self.x = pos[0]
        self.y = pos[1]
        self.lastx = self.x
        self.lasty = self.y
        self.noUpdate = 0
        
    def update(self, delta):
        if not self.noUpdate:
            #self.rect.move_ip(delta*self.speed*self.direction[0], delta*self.speed*self.direction[1])
            self.lastx = self.x
            self.lasty = self.y
            self.x += delta*self.vx#speed*self.direction[0]
            self.y += delta*self.vy#speed*self.direction[1]
            self.constrain()
            applyGravity(self)
        self.rect.center = self.x,self.y

    def constrain(self):
        containBullet(self)
        

def crossing(pa, pb, ra, rb): #returns point of contact if lines p and r cross, 0 otherwise
    x1 = pa[0]
    y1 = pa[1]
    x2 = pb[0]
    y2 = pb[1]
    x3 = ra[0]
    y3 = ra[1]
    x4 = rb[0]
    y4 = rb[1]

    ua = ( (x4-x3)*(y1-y3)-(y4-y3)*(x1-x3) )
    ub = ( (x2-x1)*(y1-y3)-(y2-y1)*(x1-x3) )
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if (abs(denom) < .0001): return 0
    ua /= denom
    ub /= denom

    if 0 < ua and ua < 1 and 0 < ub and ub < 1:
        return [x1+ua*(x2-x1),y1+ua*(y2-y1)]
    else:
        return 0


#declare puck vertices counterclockwise (i think)
#also, these don't auto-close, so bear that in mind
    
class Puck(pygame.sprite.Sprite):
    group = pygame.sprite.Group()
    massPerVertex = 1.0
    color = [50,255,50]
    edgeWidth = 4
    bounceLoss = .3
    bounceFriction = .4
    fric = .02
    rotFric = .02
    puckTemplates = []

    def __init__(self,pos, vertices,bearing_radius):
        super(Puck,self).__init__(Puck.group)
        self.vertices = vertices
        minX = vertices[0][0]
        maxX = minX
        minY = vertices[0][1]
        maxY = minY
        sumX = 0
        sumY = 0
        for vertex in self.vertices:
            sumX += vertex[0]
            sumY += vertex[1]
            if vertex[0] > maxX: maxX = vertex[0]
            elif vertex[0] < minX: minX = vertex[0]
            if vertex[1] > maxY: maxY = vertex[1]
            elif vertex[1] < minY: minY = vertex[1]
        sumX -= vertices[0][0] #since we have to manually close objects, remove dupe entry
        sumY -= vertices[0][1]
        self.cmx = sumX/float(len(vertices)-1)
        self.cmy = sumY/float(len(vertices)-1)
        self.vcmx = 0
        self.vcmy = 0
        self.moment = 0
        self.width = maxX-minX
        self.height = maxY-minY
        for vertex in self.vertices:
            vertex[0] -= self.cmx
            vertex[1] -= self.cmy
            self.moment += Puck.massPerVertex*(vertex[0]*vertex[0]+vertex[1]*vertex[1])
        self.moment *= .3 #eh, why not, makes things more spinny
        self.rect = pygame.Rect(-(maxX-minX),-(maxY-minY),2*(maxX-minX),2*(maxY-minY))
        self.shape_rect = self.rect
        self.image = pygame.Surface((SCREEN_WIDTH,SCREEN_HEIGHT)).convert_alpha()
        self.bearing_radius = bearing_radius
        self.bearing_image = pygame.transform.scale(Bullet.orig_image,(int(2*self.bearing_radius),int(2*self.bearing_radius))).convert_alpha()
        
        self.theta = 0 #angle
        self.omega = 0 #angular velocity
        self.costheta = 1.0
        self.sintheta = 0.0
        self.mass = 6.0#self.massPerVertex*(len(self.vertices)-1)
        self.cmx = pos[0]
        self.cmy = pos[1]
        self.drawnvertices = self.vertices[:]
        self.expandEdges(BULLET_RADIUS*1.0)
        self.update(0)
        
    def update(self, delta):
        #print self.rect.center
        for obj in self.getVertices():
            impulse = forceField(obj)
            self.applyImpulse(impulse[0]*delta*.5, impulse[1]*delta*.5, obj[0], obj[1])
        self.cmx += delta*self.vcmx
        self.cmy += delta*self.vcmy
        self.theta += delta*self.omega
        self.costheta = cos(self.theta)
        self.sintheta = sin(self.theta)
        self.cosdtheta = cos(delta*self.omega)
        self.sindtheta = sin(delta*self.omega)
        self.omega *= 1.0-self.rotFric
        self.vcmx *= 1.0-self.fric
        self.vcmy *= 1.0-self.fric
        self.image.fill((0,0,0,0))
        self.image.get_rect().topleft = (0,0)
        r = pygame.Rect(self.cmx-self.width,self.cmy-self.height,2*self.width,2*self.height)
        pygame.draw.polygon(self.image,self.color+[200],self.getDrawnVertices()[:-1],0)
        pygame.draw.polygon(self.image,(0,0,0,255),self.getDrawnVertices()[:-1],3)
        self.image.blit(self.bearing_image,(r.centerx-self.bearing_radius,r.centery-self.bearing_radius))
#        pygame.draw.circle(self.image,self.color+[50],r.center,self.bearing_radius)
#        pygame.draw.circle(self.image,self.color+[255],r.center,self.bearing_radius,3)
        self.rect = self.image.get_rect()
        self.shape_rect = r

    
    #def getRect(self):
#        return pygame.Rect(self.cmx-self.width,self.cmy-self.height,2*self.width,2*self.height)
        #print self.rect

    def expandEdges(self,rad):
        #pushes out edges so that we can bypass "true" circle collision detection and still have it look pretty good
        verticesa = []
        for i in range(len(self.drawnvertices)-1):
            normal = lineNormal(self.drawnvertices[i+1],self.drawnvertices[i])
            normal[0] *= rad
            normal[1] *= rad
            verticesa.append( [self.drawnvertices[i  ][0]+normal[0],self.drawnvertices[i  ][1]+normal[1] ])
        verticesa.append( verticesa[0] )
        verticesb = []
        normal = lineNormal(self.drawnvertices[0],self.drawnvertices[len(verticesa)-2])
        normal[0] *= rad
        normal[1] *= rad
        verticesb.append( [self.drawnvertices[0][0]+normal[0],self.drawnvertices[0][1]+normal[1] ])
        for i in range(1,len(self.drawnvertices)):
            normal = lineNormal(self.drawnvertices[i],self.drawnvertices[i-1])
            normal[0] *= rad
            normal[1] *= rad
            verticesb.append( [self.drawnvertices[i  ][0]+normal[0],self.drawnvertices[i  ][1]+normal[1] ])
        self.vertices = []
        for i in range(len(verticesa)):
            self.vertices.append( verticesb[i] )#[ (verticesa[i][0]+verticesb[i][0])/2.0, (verticesa[i][1]+verticesb[i][1])/2.0 ] )
            self.vertices.append( verticesa[i] )

    def getVertices(self):
        vlist = []
        for vertex in self.vertices:
            myX = self.cmx + self.costheta*vertex[0] + self.sintheta*vertex[1]
            myY = self.cmy - self.sintheta*vertex[0] + self.costheta*vertex[1]
            vlist.append( [myX,myY] )
        return vlist

    def getDrawnVertices(self):
        vlist = []
        for vertex in self.drawnvertices:
            myX = self.cmx + self.costheta*vertex[0] + self.sintheta*vertex[1]
            myY = self.cmy - self.sintheta*vertex[0] + self.costheta*vertex[1]
            vlist.append( [myX,myY] )
        return vlist
    # myXdot = cmxdot + -thetadot sintheta vertex[0] + thetadot costheta vertex[1]

    def handleCollision(self, bullet, delta):
        if not 1:#(self.rect.collidepoint(bullet.x,bullet.y) or self.rect.collidepoint(bullet.lastx,bullet.lasty)):
            #print self.rect
            return 0
        else:
            vlist = self.getVertices()
            for i in range(len(vlist)-1):
                #First off we need the normal vector to surface
                lx = (vlist[i+1][0]-vlist[i][0])/100.0 #for numerical reasons we divide down first, then normalize
                ly = (vlist[i+1][1]-vlist[i][1])/100.0
                invnorml = 1.0/sqrt(lx*lx+ly*ly) #will blow up if two vertices are in same place!
                nx = ly*invnorml #maybe a sign error in these, check it out
                ny = -lx*invnorml

                #before the check, we've got to pre-alter the old velocity - note that we must fix this before we leave
                preX = bullet.x
                preY = bullet.y
                difftocmx = preX - self.cmx
                difftocmy = preY - self.cmy
                afterrotx = self.cmx + self.cosdtheta*difftocmx - self.sindtheta*difftocmy
                afterroty = self.cmy + self.sindtheta*difftocmx + self.cosdtheta*difftocmy
                bullet.x = afterrotx - delta*self.vcmx
                bullet.y = afterroty - delta*self.vcmy
                contact = crossing(vlist[i],vlist[i+1],[bullet.x+nx,bullet.y+ny],[bullet.lastx,bullet.lasty])
                bullet.x = preX
                bullet.y = preY
                #print contact
                if contact:
                    #Gotta resolve this nasty collision...

                    #Find the distance from cm to contact point
                    dzx = (contact[0] - self.cmx)/10.0
                    dzy = (contact[1] - self.cmy)/10.0
                    dz = 100*sqrt( dzx*dzx+dzy*dzy )
                    #Use this to find the velocity of that point on the rigid body

                    vzx = self.vcmx+self.omega*(-self.sintheta*dzx+self.costheta*dzy)
                    vzy = self.vcmy+self.omega*(-self.costheta*dzx-self.sintheta*dzy)
                    #Now transform the bullet velocity into this frame
                    dyx = (bullet.x-bullet.lastx)/delta
                    dyy = (bullet.y-bullet.lasty)/delta
                    dyrx = dyx-vzx #relative components of velocity
                    dyry = dyy-vzy
                    #print nx,ny,lx,ly,dyrx,dyry
                    dyperp = dyrx*nx + dyry*ny
                    if dyperp > 0:
                        #we _actually_ have a collision, since the normal component is positive
                        #Set new bullet location to collision point
                        bullet.x = contact[0]
                        bullet.y = contact[1]
                        bullet.rect.center = bullet.x, bullet.y
                        #Find velocity changes
                        dynorm = sqrt(dyx*dyx+dyy*dyy)+.001
                        dyperpx = dyperp*nx
                        dyperpy = dyperp*ny
                        dyparx = dyx-dyperpx
                        dypary = dyy-dyperpy
                        deltayperpx = -(2.0-self.bounceLoss)*dyperpx
                        deltayperpy = -(2.0-self.bounceLoss)*dyperpy
                        deltayparx = -self.bounceFriction*dyparx
                        deltaypary = -self.bounceFriction*dypary
                        #pygame.draw.line(screen,(255,255,255),(contact[0],contact[1]),(contact[0]+deltayperpx,contact[1]+deltayperpy))
                        #Now apply these velocity changes
                        bullet.vx += deltayperpx+deltayparx
                        bullet.vy += deltayperpy+deltaypary

                        #And apply the resulting impulse to the rigid body...
                        self.applyImpulse(-(deltayperpx+deltayparx)*bullet.mass,-(deltayperpy+deltaypary)*bullet.mass,contact[0],contact[1])
                        #bullet.x += delta*bullet.vx #Eh, gotta get it out of there somehow, right?
                        #bullet.y += delta*bullet.vy #If I wasn't too lazy I'd figure out how far to actually move it, but .1 of a frame is a decent hack for now
                        #print contact[0],contact[1]

    def applyImpulse(self, impx, impy, locx, locy):
        
        self.vcmx += impx / self.mass
        self.vcmy += impy / self.mass
        rx = locx-self.cmx
        ry = locy-self.cmy
        torqueimp = impx*ry-impy*rx
        self.omega += torqueimp / self.moment


#CLASS CHANGED BY ERIC 12:30
class Player(pygame.sprite.Sprite):

    star = 0
    group = pygame.sprite.Group()


    def __init__(self,pos,rot,player_num=1):
        super(Player,self).__init__(Player.group)

        player_filenames = ['player0.png','player1.png']
        self.orig_image = pygame.image.load(data_path(player_filenames[player_num])).convert_alpha()
        if not(SCALE == 1):
            w, h = self.orig_image.get_size()
            w = int(w / SCALE)
            h = int(h / SCALE)
            self.orig_image = pygame.transform.scale(self.orig_image, (w, h))

        player_color = [255,0,0]

        # Rotation and positioning of rect.        
        self.orig_rotate = rot # "baseline" CW rotation, in degrees
        self.position = pos
        self.image = pygame.transform.rotate(self.orig_image,-self.orig_rotate)
        self.rect = self.image.get_rect()
        self.rect.center = self.position

        # Player variables.
        self.theta = self.orig_rotate # *current* player rotation, in degrees CW
        self.dtheta = 0
        self.bulletSpeed = BULLET_SPEED
        #self.ammo = AMMO_PER_PLAYER #added when bullets are populated
        self.ammoList = pygame.sprite.Group()
        self.storedBulletPositions = []
        self.num_pucks = 0
        self.trophyCase = 0
        self.makeTrophyCase()
        pygame.draw.rect(self.trophyCase, (0,100,0,255), self.trophyCase.get_rect(),5)

        self.isHuman = 0
    
        # Boolean keystate values.
        self.turnRight = 0
        self.turnLeft = 0
        self.shootButton = 0
        self.reloaded = 1
        self.reloadTimer = 0
        #self.ammoList = []
        #self.ammo = 0 #reset below...
        self.generateStoredBulletPositions()

    def makeTrophyCase(self):
        self.trophyCase = pygame.Surface((200,60)).convert_alpha()
        self.trophyPos = self.trophyCase.get_rect()
        self.trophyPos.centerx = self.rect.centerx
        self.trophyPos.top = 10
        self.trophyCase.fill((0,0,0,0))
        pygame.draw.rect(self.trophyCase, (0,100,0,255), self.trophyCase.get_rect(),5)
    

    def reset(self):
        self.theta = self.orig_rotate
        self.dtheta = 0
        self.ammoList = pygame.sprite.Group()
        for i in range(AMMO_PER_PLAYER):
            bullet = Bullet( [0.0,0.0], [0.0,0.0] )
            consumeBullet(bullet, self)
            #self.ammoList.add(bullet)

    def update(self,delta):
        self.theta = self.theta + self.dtheta
        self.constrain()
        self.image = pygame.transform.rotate(self.orig_image,-self.theta)
        self.rect = self.image.get_rect()
        self.rect.center = self.position
        if not (self.turnRight or self.turnLeft):
            self.dtheta = self.dtheta * ROTATION_DECAY
        for i in range(len(self.ammoList.sprites())):
            self.ammoList.sprites()[i].x = self.getStoredBulletPos(i)[0]
            self.ammoList.sprites()[i].y = self.getStoredBulletPos(i)[1]

    def generateStoredBulletPositions(self):
        for i in range(2*AMMO_PER_PLAYER):
            self.storedBulletPositions.append( self.genStoredBulletPos(i) )

    def genStoredBulletPos(self,num):
        rad = CIRCLE_RADIUS+2*BULLET_RADIUS
        maxAngle = pi/2.0
        ang = maxAngle*( (num)/(2.0*AMMO_PER_PLAYER) )
        if num%2==0:
            ang += maxAngle/(2.0*AMMO_PER_PLAYER)
            ang *= -1
        yC = SCREEN_HEIGHT/2.0 + rad*sin(ang)
        if self.rect.center[0] < SCREEN_WIDTH/2:
            xCenter = (CIRCLE_BORDER+CIRCLE_RADIUS)
            xC = xCenter - rad*cos(ang)
        else:
            xCenter = (SCREEN_WIDTH-CIRCLE_BORDER-CIRCLE_RADIUS)
            xC = xCenter + rad*cos(ang)
        return [xC,yC]#self.rect.center[0]+diff, SCREEN_HEIGHT*( (num+.5)/(2.0*AMMO_PER_PLAYER+1) )]

    def getStoredBulletPos(self, num):
        return self.storedBulletPositions[num]

    def constrain(self):
        if (self.theta - self.orig_rotate) >= MAX_ROTATION:
            self.theta = self.orig_rotate + MAX_ROTATION
        if (self.theta - self.orig_rotate) <= -MAX_ROTATION:
            self.theta = self.orig_rotate - MAX_ROTATION
        if self.dtheta > ROTATION_SPEED:
            self.dtheta = ROTATION_SPEED
        if self.dtheta < -ROTATION_SPEED:
            self.dtheta = -ROTATION_SPEED

    def collectBall(self,bullet):
        bullet.noUpdate = 1
        self.ammoList.add(bullet)
        

    def hasAmmo(self):
        if len(self.ammoList.sprites()) > 0:
            returnbullet = self.ammoList.sprites()[0]
            returnbullet.kill()
            return returnbullet
        else:
            return 0

def genPucks(puckTemplates):
    myBuf = []
    for i in range(6+1):
        leg = 15
        if i%2==0: leg = 60
        myBuf.append( [leg*sin(2*pi*i/6.0),leg*cos(2*pi*i/6.0)] )
        
    puckTemplates.append(myBuf)

    myBuf = []
    for i in range(8+1):
        leg = 15
        if i%2==0: leg = 60
        myBuf.append( [leg*sin(2*pi*i/8.0),leg*cos(2*pi*i/8.0)] )
        
    puckTemplates.append(myBuf)
    
    myBuf = []
    for i in range(9+1):
        leg = 45
        if i%3==0: leg = 15
        myBuf.append( [leg*sin(2*pi*i/9.0),leg*cos(2*pi*i/9.0)] )

    puckTemplates.append(myBuf)

    myBuf = []
    for i in range(5+1):
        leg = 60
        myBuf.append( [leg*sin(2*pi*i/5.0),leg*cos(2*pi*i/5.0)] )

    puckTemplates.append(myBuf)

    myBuf = []
    for i in range(3+1):
        leg = 60
        myBuf.append( [leg*sin(2*pi*i/3.0),leg*cos(2*pi*i/3.0)] )
    puckTemplates.append(myBuf)


GAME_WON = 0

def newGame():
    global GAME_WON
    GAME_WON = 1
    for player in Player.group:
        player.makeTrophyCase()
        player.num_pucks = 0

def sendPuckToPlayer(puck,player):
    dest = (player.num_pucks * 30 + 10,10)
    scaled = pygame.transform.scale(player.star,(40,40))
    player.trophyCase.blit(scaled,dest)

    # Remove current puck from Puck.group
    Puck.group.remove(puck)

    player.num_pucks = player.num_pucks + 1
    print player.num_pucks
    newRound()

def newRound():
    
    for bullet in Bullet.group:
        bullet.kill()
    for player in Player.group:
        player.reset()
        if player.num_pucks == PUCKS_TO_WIN:
            newGame()
    # Add a new puck to the game.
    whichPuck = int( len(Puck.puckTemplates)*random.random() )
    Puck.group.add(Puck( pygame.display.get_surface().get_rect().center, Puck.puckTemplates[whichPuck],20 ))

def applyAI(player):
    #pick between turn left, turn right, shoot, and nothing
    for puck in Puck.group:
        cosx = cos((player.theta - 90)/180*pi)
        sinx = sin((player.theta - 90)/180*pi)
        bulletDir = [player.bulletSpeed * cosx,player.bulletSpeed * sinx]
        slope = bulletDir[1]/bulletDir[0]
        desiredSlope = (puck.cmy-player.rect.center[1])/(puck.cmx-player.rect.center[0])
        #print desiredAngle, ang
        #if player.rect.center[0]>SCREEN_WIDTH/2: desiredAngle -= pi
        if slope>desiredSlope+random.random():
            player.turnLeft = 1
            player.turnRight = 0
        elif slope<desiredSlope-random.random():
            player.turnRight = 1
            player.turnLeft = 0
        if  random.random()>.9:
            player.shootButton = 1
        else:
            player.shootButton = 0

players = []
 
class SprayPlay():
    
    def __init__(self):
        pass

    def run(self):
        global SCREEN_WIDTH, SCREEN_HEIGHT, CIRCLE_BORDER, CIRCLE_RADIUS, CIRCLE_LEFT, CIRCLE_RIGHT, GRAVITY_DIST, SCALE
        # Initialization
        pygame.init()
        screen = pygame.display.get_surface()
        SCREEN_WIDTH = screen.get_width()
        SCREEN_HEIGHT = screen.get_height()
        background = Background(SCREEN_WIDTH, SCREEN_HEIGHT)

        # Playing field constants
        CIRCLE_BORDER = SCREEN_HEIGHT / 20
        CIRCLE_RADIUS = int((SCREEN_HEIGHT-2*CIRCLE_BORDER)/2)
        CIRCLE_LEFT = [CIRCLE_BORDER + CIRCLE_RADIUS, CIRCLE_BORDER + CIRCLE_RADIUS]
        CIRCLE_RIGHT = [SCREEN_WIDTH - CIRCLE_BORDER - CIRCLE_RADIUS, CIRCLE_BORDER + CIRCLE_RADIUS]
        
        if SCREEN_WIDTH == 1200:
            SCALE = 1
            GRAVITY_DIST = CIRCLE_BORDER + 175
            CONST = 80
        else:
            SCALE = 1200.0 / SCREEN_WIDTH
            GRAVITY_DIST = CIRCLE_BORDER + int(SCREEN_WIDTH * 175.0 / 1200.0)
            CONST = int(SCREEN_WIDTH * 80.0 / 1200.0)

        # Initial game state.
        overlay = Overlay()
        #intro = Intro()
        keyboard_intro = Intro('keyboard-',4,1000)
        intro_group = pygame.sprite.Group(overlay,keyboard_intro)

        clock = pygame.time.Clock()
        
        # Bullet image.
        Bullet.orig_image = pygame.image.load(data_path('ball.png')).convert_alpha()

        # Star image.
        Player.star = pygame.image.load(data_path('star.png')).convert_alpha()
        
        # Players.
        global players
        a = Player((CIRCLE_BORDER + CONST,SCREEN_HEIGHT//2),90,0)
        b = Player((SCREEN_WIDTH-CIRCLE_BORDER - CONST,SCREEN_HEIGHT//2),-90,1)
        players = [a,b]

        # Puck defs
        genPucks(Puck.puckTemplates)

        newRound()
        try:
            while True:
                delta = clock.tick(20)

                #delta = 1.0 / 25.0
                #GTK events
                while gtk.events_pending():
                    gtk.main_iteration()

                for evt in pygame.event.get():
                    if evt.type == pygame.QUIT:
                        raise StopGame
                    elif evt.type == pygame.KEYDOWN:
                        if not overlay.dying:
                            overlay.dying = 1
                            keyboard_intro.kill()
                        if evt.key == pygame.K_F5:
                            newGame()
                        if evt.key == pygame.K_ESCAPE:
                            #raise StopGame
                            newGame()
                        if (evt.key == pygame.K_RIGHT) or (evt.key == pygame.K_KP9):
                            b.turnRight = 1
                            b.isHuman = 1
                        if (evt.key == pygame.K_LEFT) or (evt.key == pygame.K_KP3):
                            b.turnLeft = 1
                            b.isHuman = 1
                        if (evt.key == pygame.K_RETURN) or (evt.key == pygame.K_KP7):
                            b.shootButton = 1
                            b.isHuman = 1
                            overlay.kill()
                        if (evt.key == pygame.K_d) or (evt.key == pygame.K_KP2):
                            a.turnRight = 1
                            a.isHuman = 1
                        if (evt.key == pygame.K_a) or (evt.key == pygame.K_KP8):
                            a.turnLeft = 1
                            a.isHuman = 1
                        if (evt.key == pygame.K_SPACE) or (evt.key == pygame.K_KP6):
                            a.shootButton = 1
                            a.isHuman = 1
                    elif evt.type == pygame.KEYUP:
                        if (evt.key == pygame.K_RIGHT) or (evt.key == pygame.K_KP9):
                            b.turnRight = 0
                            b.isHuman = 1
                        if (evt.key == pygame.K_LEFT) or (evt.key == pygame.K_KP3):
                            b.turnLeft = 0
                            b.isHuman = 1
                        if (evt.key == pygame.K_RETURN) or (evt.key == pygame.K_KP7):
                            b.shootButton = 0
                            b.isHuman = 1
                        if (evt.key == pygame.K_d) or (evt.key == pygame.K_KP2):
                            a.turnRight = 0
                            a.isHuman = 1
                        if (evt.key == pygame.K_a) or (evt.key == pygame.K_KP8):
                            a.turnLeft = 0
                            a.isHuman = 1
                        if (evt.key == pygame.K_SPACE) or (evt.key == pygame.K_KP6):
                            a.shootButton = 0
                            a.isHuman = 1
                    elif evt.type == pygame.VIDEORESIZE:
                        pygame.display.set_mode(event.size, pygame.RESIZABLE)

                if not a.isHuman:
                    applyAI(a)
                if not b.isHuman:
                    applyAI(b)

                # inject round-wining logic here.
                for puck in Puck.group:
                    if puck.cmx > SCREEN_WIDTH - GRAVITY_DIST:
                        # Player 0 gets it.
                        sendPuckToPlayer(puck,players[0])
                    elif puck.cmx < GRAVITY_DIST:
                        # Player 1 gets it.
                        sendPuckToPlayer(puck,players[1])
                

                # inject AI logic here.

                for player in players:
                    if player.shootButton and player.reloaded:
                        bull = player.hasAmmo()
                        if bull:
                            bull.kill()
                            cosx = cos((player.theta - 90)/180*pi)
                            sinx = sin((player.theta - 90)/180*pi)
                            bulletDir = [player.bulletSpeed * cosx,player.bulletSpeed * sinx]
                            x = player.rect.center[0] + cosx * PLAYER_ARM_LENGTH
                            y = player.rect.center[1] + sinx * PLAYER_ARM_LENGTH
                            Bullet( (x,y),bulletDir )
                            #player.ammo -= 1
                            player.reloaded = 0
                            player.reloadTimer = int((1.0 + RELOAD_RANDOM*2*(random.random()-.5))*RELOAD_DELAY)
                    if not player.reloaded:
                        player.reloadTimer -= 1
                    if player.reloadTimer == 0:
                        player.reloaded = 1
                    if player.turnRight:
                        player.dtheta = player.dtheta + ROTATION_SPEED / 15.0
                    if player.turnLeft:
                        player.dtheta = player.dtheta - ROTATION_SPEED / 15.0
                                    

                # Update phase
                Bullet.group.update(delta)
                Player.group.update(delta)
                Puck.group.update(delta)

                #ADDED BY ERIC 12:30
                for player in players:
                    player.ammoList.update(delta)

                for bullet in Bullet.group:
                    for puck in Puck.group:
                        puck.handleCollision(bullet,delta)
                        
                # Draw phase
                Background.group.draw(screen)
                Bullet.group.draw(screen)
                Player.group.draw(screen)
                Puck.group.draw(screen)
                #ADDED BY ERIC 12:30
                for player in players:
                    player.ammoList.draw(screen)
                    screen.blit(player.trophyCase,player.trophyPos)
                intro_group.update(delta)
                intro_group.draw(screen)
                
                pygame.display.flip()
        except StopGame:
            pass

if __name__ == '__main__':
    g = SprayPlay()
    g.run()

