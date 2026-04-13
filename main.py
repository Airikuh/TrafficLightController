# University of Michigan Dearborn ECE 554: Embedded Systems
# Final Project: Traffic Light with Real Time Operating System Scheduler (Simulated since not using Hardware)

# Current Requirements Implemented: Simulated RTOS Task Scheduling, Normal Traffic Phases
# Emergency Vehicles: Have Priority Over all Other Requests, Emergency Override does not create Conflicting Green Lights, All Other Vehicles are Stopped when Emergency Vehicles are Crossing.
# Pedestrians: WALK Phase Becomes Priority when Button is Pressed, Repeated Button Presses are Ignored while Active, Minimum Crossing Time is 20 seconds, Pedestrian Crossings include Countdown and Status Indicators 

# To Control the Simulation: Press 'P' for Pedestrian Request (Pedestrian presses button), Press 'E' to Envoke/Spawn Emergency Vehicle, Press 'R' to Spawn a Normal Vehicle from a Random Direction (If you do not press R, 
# normal vehicles appear at random times), Press 'esc' to Stop the Simulation. 

import time
import pygame
import math
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
import cv2
import string
import pandas as pd
import warnings
import random
import math
from enum import Enum, auto
pygame.init()

#REDO OF THE OTHER CODE


width, height = 1200, 800
SCREEN = pygame.display.set_mode((width, height))
pygame.display.set_caption("RTOS Traffic Light with Emergency + Pedestrians")
CLOCK = pygame.time.Clock()

FONT = pygame.font.SysFont("arial", 20, bold=True)
SMALL_FONT = pygame.font.SysFont("arial", 16)
BIG_FONT = pygame.font.SysFont("arial", 28, bold=True)


# Colors for Pygame GUI and Visuals
white = (255, 255, 255)
black = (0, 0, 0)
gray = (130, 130, 130)
dark_gray = (60, 60, 60)
road = (45, 45, 45)
lane = (230, 230, 230)
yellow = (255, 215, 0)
red = (220, 50, 50)
green = (50, 200, 70)
blue = (80, 150, 255)
light_blue = (150, 200, 255)
orange = (255, 150, 40)
grass = (70, 140, 70)
cross = (245, 245, 245)
color_of_pedestrian = (255, 105, 180)


# For the Roads/Calculations
road_W = 220
lane_W = 28
center_X = width // 2
center_Y = height // 2

vertical_road = pygame.Rect(center_X - road_W // 2, 0, road_W, height)
horizontal_road = pygame.Rect(0, center_Y - road_W // 2, width, road_W)
INTERSECTION = pygame.Rect(center_X - road_W // 2, center_Y - road_W // 2, road_W, road_W)

# Style the Crosswalks with the Traffic lights
north_cross = pygame.Rect(center_X - road_W // 2, center_Y - road_W // 2 - 30, road_W, 24)
south_cross = pygame.Rect(center_X - road_W // 2, center_Y + road_W // 2 + 6, road_W, 24)
west_cross = pygame.Rect(center_X - road_W // 2 - 30, center_Y - road_W // 2, 24, road_W)
east_cross = pygame.Rect(center_X + road_W // 2 + 6, center_Y - road_W // 2, 24, road_W)

# Stop lines
n_s_stop_N_Y = center_Y - road_W // 2 - 15
n_s_stop_S_Y = center_Y + road_W // 2 + 15
e_w_stop_W_X = center_X - road_W // 2 - 15
e_w_stop_E_X = center_X + road_W // 2 + 15


# Timing constants
green_n_s = 12.0
green_e_w = 12.0
yellow_timer = 3.0
timer_all_red = 1.5
min_ped_timer = 20.0
EMERGENCY_CLEARANCE = 1.2
spawn_interval = 1.5

lane_max_cars = 12

# Enums
class SignalState(Enum):
    red = auto()
    yellow = auto()
    green = auto()

class Phase(Enum):
    green_n_s = auto()
    yellow_n_s = auto()
    ew_green = auto()
    yellow_e_w = auto()
    all_red = auto()
    PED_WALK = auto()
    EMERGENCY = auto()

class Direction(Enum):
    n_s = auto()
    s_n = auto()
    w_e = auto()
    e_w = auto()

# Helper functions
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def draw_text(surface, text, font, color, pos, center=False):
    img = font.render(text, True, color)
    rect = img.get_rect()
    if center:
        rect.center = pos
    else:
        rect.topleft = pos
    surface.blit(img, rect)

def lerp(a, b, t):
    return a + (b - a) * t

# Vehicle class to represent motor vehicles
class Vehicle:
    def __init__(self, direction, emergency=False):
        self.direction = direction
        self.emergency = emergency
        self.length = 42 if not emergency else 54
        self.width = 20 if not emergency else 24
        self.speed = 110 if not emergency else 180
        self.color = blue if not emergency else red
        self.siren_timer = 0.0
        if direction == Direction.n_s:
            self.x = center_X - lane_W // 2 - 6
            self.y = -80
            self.vx, self.vy = 0, self.speed
        elif direction == Direction.s_n:
            self.x = center_X + lane_W // 2 + 6
            self.y = height + 80
            self.vx, self.vy = 0, -self.speed
        elif direction == Direction.w_e:
            self.x = -80
            self.y = center_Y + lane_W // 2 + 6
            self.vx, self.vy = self.speed, 0
# Else we are going from East to West
        else:  
            self.x = width + 80
            self.y = center_Y - lane_W // 2 - 6
            self.vx, self.vy = -self.speed, 0
        self.removed = False

    def first_position(self):
        if self.direction == Direction.n_s:
            return self.y + self.length / 2
        if self.direction == Direction.s_n:
            return self.y - self.length / 2
        if self.direction == Direction.w_e:
            return self.x + self.length / 2
        return self.x - self.length / 2

    def stop_line_position(self):
        if self.direction == Direction.n_s:
            return n_s_stop_N_Y
        if self.direction == Direction.s_n:
            return n_s_stop_S_Y
        if self.direction == Direction.w_e:
            return e_w_stop_W_X
        return e_w_stop_E_X

    def signal_group(self):
        if self.direction in (Direction.n_s, Direction.s_n):
            return "NS"
        return "EW"

    def is_inside_intersection(self):
        #rect = self.rect()
        #return INTERSECTION.colliderect(rect)
    # Cars were freezing after they passed the intersection. Changed to this line
        return INTERSECTION.colliderect(self.rect())

    def passed_intersection(self):
        if self.direction == Direction.n_s:
            return self.y > height + 120
        if self.direction == Direction.s_n:
            return self.y < -120
        if self.direction == Direction.w_e:
            return self.x > width + 120
        return self.x < -120

    def rect(self):
        if self.direction in (Direction.n_s, Direction.s_n):
            return pygame.Rect(int(self.x - self.width / 2), int(self.y - self.length / 2), self.width, self.length)
        else:
            return pygame.Rect(int(self.x - self.length / 2), int(self.y - self.width / 2), self.length, self.width)

# Added the following 4 functions to prevent cars from freezing after they passed the intersection (normal vehicle spawn was causing it too)
#Determine if a normal vehicle has cross the stop line or note
    def has_crossed_stop_line(self):
        front = self.first_position()
        stop_pos = self.stop_line_position()
        if self.direction == Direction.n_s:
            return front > stop_pos
        if self.direction == Direction.s_n:
            return front < stop_pos
        if self.direction == Direction.w_e:
            return front > stop_pos
        return front < stop_pos

# Function to determine if a normal vehicle is moving toward the stop line (or else is moving away from it)
    def is_approaching_stop_line(self):
        return (not self.is_inside_intersection()) and (not self.has_crossed_stop_line())

# Function to determine the distance to the vehicle in the 1st position
    def distance_to_lead_vehicle(self, other):
        if self.direction == Direction.n_s and other.y > self.y:
            return other.y - self.y
        if self.direction == Direction.s_n and other.y < self.y:
            return self.y - other.y
        if self.direction == Direction.w_e and other.x > self.x:
            return other.x - self.x
        if self.direction == Direction.e_w and other.x < self.x:
            return self.x - other.x
        return None

# One update caused Emergency Vehicles to Freeze behind normal vehicles. 
# This Function was added to prevent that. 
    def blocked_by_lead_vehicle(self, vehicles, controller):
# Emergency vehicles now get a protected corridor during EMERGENCY mode. Prevents them from entering queue behind normal vehicles
        if self.emergency and controller.phase == Phase.EMERGENCY:
            return False
        for other in vehicles:
            if other is self or other.direction != self.direction or other.removed:
                continue
            distance = self.distance_to_lead_vehicle(other)
            if distance is None:
                continue
# Prevents cars beyond the intersection moving away from it from freezing in place
            if other.has_crossed_stop_line() and not other.is_inside_intersection():
                if distance > QUEUE_GAP:
                    continue
            if distance < QUEUE_GAP:
                return True
        return False





    def can_move(self, controller, vehicles):
# EMERGENCY VEHICLE PHASE: Emergency Vehicle gets exclusive path
        if self.emergency:
            if controller.phase != Phase.EMERGENCY:
# Other cars can still approach the intersection, however must stop at the line (had issues with cars going through green light, but that is reserved for the Emergency Vehicles)
                if not self.is_inside_intersection():
                    stop_pos = self.stop_line_position()
                    front = self.first_position()
                    if self.direction == Direction.n_s and front >= stop_pos:
                        return False
                    if self.direction == Direction.s_n and front <= stop_pos:
                        return False
                    if self.direction == Direction.w_e and front >= stop_pos:
                        return False
                    if self.direction == Direction.e_w and front <= stop_pos:
                        return False
                return True
            return True
# EMERGENCY VEHICLE PHASE: Outside vehicles cannot enter, vehicles within the intersection must clear it.
        if controller.phase == Phase.EMERGENCY:
            if self.is_inside_intersection():
                return True
            stop_pos = self.stop_line_position()
            front = self.first_position()
            if self.direction == Direction.n_s and front >= stop_pos:
                return False
            if self.direction == Direction.s_n and front <= stop_pos:
                return False
            if self.direction == Direction.w_e and front >= stop_pos:
                return False
            if self.direction == Direction.e_w and front <= stop_pos:
                return False
# Had to specify queue spacing needed to remain normal when approaching the stop line. 
            for other in vehicles:
                if other is self or other.direction != self.direction:
                    continue
                if self.direction == Direction.n_s and other.y > self.y:
                    if other.y - self.y < 60:
                        return False
                elif self.direction == Direction.s_n and other.y < self.y:
                    if self.y - other.y < 60:
                        return False
                elif self.direction == Direction.w_e and other.x > self.x:
                    if other.x - self.x < 60:
                        return False
                elif self.direction == Direction.e_w and other.x < self.x:
                    if self.x - other.x < 60:
                        return False
            return True
# Stop during pedestrian walk, but let vehicles already in the intersection clear
        if controller.phase == Phase.PED_WALK:
            if self.is_inside_intersection():
                return True
            stop_pos = self.stop_line_position()
            front = self.first_position()
            if self.direction == Direction.n_s and front >= stop_pos:
                return False
            if self.direction == Direction.s_n and front <= stop_pos:
                return False
            if self.direction == Direction.w_e and front >= stop_pos:
                return False
            if self.direction == Direction.e_w and front <= stop_pos:
                return False
        group = self.signal_group()
        sig = controller.ns_signal if group == "NS" else controller.ew_signal
        allow = (sig == SignalState.green)
# If not green and not already inside, the vehicle must stop at line
        if not allow and not self.is_inside_intersection():
            stop_pos = self.stop_line_position()
            front = self.first_position()
            if self.direction == Direction.n_s and front >= stop_pos:
                return False
            if self.direction == Direction.s_n and front <= stop_pos:
                return False
            if self.direction == Direction.w_e and front >= stop_pos:
                return False
            if self.direction == Direction.e_w and front <= stop_pos:
                return False
# Queue Spacing (i.e cars following behind another)
        for other in vehicles:
            if other is self or other.direction != self.direction:
                continue
            if self.direction == Direction.n_s and other.y > self.y:
                if other.y - self.y < 60:
                    return False
            elif self.direction == Direction.s_n and other.y < self.y:
                if self.y - other.y < 60:
                    return False
            elif self.direction == Direction.w_e and other.x > self.x:
                if other.x - self.x < 60:
                    return False
            elif self.direction == Direction.e_w and other.x < self.x:
                if self.x - other.x < 60:
                    return False
        return True

    def update(self, dt, controller, vehicles):
        if self.emergency:
            self.siren_timer += dt
        if self.can_move(controller, vehicles):
            self.x += self.vx * dt
            self.y += self.vy * dt
        if self.passed_intersection():
            self.removed = True

    def draw(self, surface):
        r = self.rect()
        pygame.draw.rect(surface, self.color, r, border_radius=5)
        pygame.draw.rect(surface, black, r, 2, border_radius=5)
        if self.direction in (Direction.n_s, Direction.s_n):
            pygame.draw.rect(surface, light_blue, (r.x + 3, r.y + 5, r.w - 6, 8))
        else:
            pygame.draw.rect(surface, light_blue, (r.x + 5, r.y + 3, 8, r.h - 6))
        if self.emergency:
            flash_on = int(self.siren_timer * 8) % 2 == 0
            if flash_on:
                if self.direction in (Direction.n_s, Direction.s_n):
                    pygame.draw.rect(surface, blue, (r.centerx - 8, r.y + 2, 6, 6))
                    pygame.draw.rect(surface, white, (r.centerx + 2, r.y + 2, 6, 6))
                else:
                    pygame.draw.rect(surface, blue, (r.x + 2, r.centery - 8, 6, 6))
                    pygame.draw.rect(surface, white, (r.x + 2, r.centery + 2, 6, 6))


# Pedestrian class to represent pedestrians
class Pedestrian:
    def __init__(self, start_pos, end_pos, duration):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.duration = duration
        self.elapsed = 0.0
        self.finished = False

    def update(self, dt):
        self.elapsed += dt
        if self.elapsed >= self.duration:
            self.finished = True

    def draw(self, surface):
        t = clamp(self.elapsed / self.duration, 0.0, 1.0)
        x = lerp(self.start_pos[0], self.end_pos[0], t)
        y = lerp(self.start_pos[1], self.end_pos[1], t)
# Can change here if want something better than a stick figure
        pygame.draw.circle(surface, color_of_pedestrian, (int(x), int(y - 10)), 5)
        pygame.draw.line(surface, color_of_pedestrian, (x, y - 5), (x, y + 10), 2)
        pygame.draw.line(surface, color_of_pedestrian, (x, y), (x - 7, y + 5), 2)
        pygame.draw.line(surface, color_of_pedestrian, (x, y), (x + 7, y + 5), 2)
        pygame.draw.line(surface, color_of_pedestrian, (x, y + 10), (x - 6, y + 18), 2)
        pygame.draw.line(surface, color_of_pedestrian, (x, y + 10), (x + 6, y + 18), 2)


# RTOS task model
class Task:
    def __init__(self, name, period, priority, callback):
        self.name = name
        self.period = period
        self.priority = priority   # lower number = higher priority
        self.callback = callback
        self.accumulator = 0.0

    def tick(self, dt):
        self.accumulator += dt

    def ready(self):
        return self.accumulator >= self.period

    def run(self):
        self.accumulator -= self.period
        self.callback()

# Real Time Operating System Class
class RTOS:
    def __init__(self):
        self.tasks = []

    def add_task(self, task):
        self.tasks.append(task)

    def update(self, dt):
        for t in self.tasks:
            t.tick(dt)
        ready_tasks = [t for t in self.tasks if t.ready()]
        ready_tasks.sort(key=lambda t: t.priority)
# Run all ready tasks in priority order
        for task in ready_tasks:
            task.run()


# Traffic Controller class
class TrafficController:
    def __init__(self):
        self.phase = Phase.green_n_s
        self.phase_timer = 0.0
        self.all_red_reason = None
        self.ns_signal = SignalState.green
        self.ew_signal = SignalState.red
        self.ped_request = False
        self.ped_active = False
        self.ped_countdown = 0.0
        self.ped_status = "DON'T WALK"
        self.pedestrians = []
        self.emergency_request = False
        self.emergency_direction = None
        self.emergency_active = False

    def request_pedestrian(self):
# Ignore repeated presses while active
# One of the requirement specifications
        if self.ped_active:
            return
        self.ped_request = True

    def request_emergency(self, direction):
        self.emergency_request = True
        self.emergency_direction = direction

    def set_ns(self, state):
        self.ns_signal = state

    def set_ew(self, state):
        self.ew_signal = state

    def set_all_red(self):
        self.set_ns(SignalState.red)
        self.set_ew(SignalState.red)

    def start_ped_walk(self):
        self.phase = Phase.PED_WALK
        self.phase_timer = 0.0
        self.ped_active = True
        self.ped_request = False
        self.ped_countdown = min_ped_timer
        self.ped_status = "WALK"
# Two animated pedestrians crossing in opposite directions
# Can change this here if something else if preferred
        self.pedestrians = [
            Pedestrian(
                (center_X - road_W // 2 + 20, center_Y - road_W // 2 - 18),
                (center_X + road_W // 2 - 20, center_Y - road_W // 2 - 18),
                min_ped_timer
            ),
            Pedestrian(
                (center_X + road_W // 2 - 20, center_Y + road_W // 2 + 18),
                (center_X - road_W // 2 + 20, center_Y + road_W // 2 + 18),
                min_ped_timer
            ),
        ]
        self.set_all_red()

    def start_emergency_phase(self):
        self.phase = Phase.EMERGENCY
        self.phase_timer = 0.0
        self.emergency_active = True
        self.ped_active = False
        self.ped_status = "DON'T WALK"
        self.pedestrians = []
# Exclusive green only for requested direction group
        if self.emergency_direction in (Direction.n_s, Direction.s_n):
            self.set_ns(SignalState.green)
            self.set_ew(SignalState.red)
        else:
            self.set_ns(SignalState.red)
            self.set_ew(SignalState.green)

    def end_emergency_phase(self):
        self.emergency_active = False
        self.emergency_request = False
        self.emergency_direction = None
        self.phase = Phase.all_red
        self.phase_timer = 0.0
        self.all_red_reason = "post_emergency"
        self.set_all_red()

    def update(self, dt, vehicles):
        self.phase_timer += dt
# Emergency request always has top priority
        if self.emergency_request and self.phase != Phase.EMERGENCY:
# Transition safely through all-red first if needed
            if self.phase != Phase.all_red:
                self.phase = Phase.all_red
                self.phase_timer = 0.0
                self.all_red_reason = "emergency"
                self.set_all_red()
        if self.phase == Phase.green_n_s:
            self.set_ns(SignalState.green)
            self.set_ew(SignalState.red)
            self.ped_status = "DON'T WALK"
            if self.phase_timer >= green_n_s:
                self.phase = Phase.yellow_n_s
                self.phase_timer = 0.0
        elif self.phase == Phase.yellow_n_s:
            self.set_ns(SignalState.yellow)
            self.set_ew(SignalState.red)
            if self.phase_timer >= yellow_timer:
                self.phase = Phase.all_red
                self.phase_timer = 0.0
                self.all_red_reason = "to_ew"
                self.set_all_red()
        elif self.phase == Phase.ew_green:
            self.set_ns(SignalState.red)
            self.set_ew(SignalState.green)
            self.ped_status = "DON'T WALK"
            if self.phase_timer >= green_e_w:
                self.phase = Phase.yellow_e_w
                self.phase_timer = 0.0
        elif self.phase == Phase.yellow_e_w:
            self.set_ns(SignalState.red)
            self.set_ew(SignalState.yellow)
            if self.phase_timer >= yellow_timer:
                self.phase = Phase.all_red
                self.phase_timer = 0.0
                self.all_red_reason = "to_ns"
                self.set_all_red()
        elif self.phase == Phase.all_red:
            self.set_all_red()
            if self.phase_timer >= (EMERGENCY_CLEARANCE if self.all_red_reason == "emergency" else timer_all_red):
                if self.all_red_reason == "emergency" and self.emergency_request:
                    self.start_emergency_phase()
                elif self.ped_request:
                    self.start_ped_walk()
                elif self.all_red_reason in ("to_ew",):
                    self.phase = Phase.ew_green
                    self.phase_timer = 0.0
                else:
                    self.phase = Phase.green_n_s
                    self.phase_timer = 0.0
        elif self.phase == Phase.PED_WALK:
            self.set_all_red()
            self.ped_countdown = max(0.0, min_ped_timer - self.phase_timer)
            self.ped_status = f"WALK {math.ceil(self.ped_countdown)}"
            for ped in self.pedestrians:
                ped.update(dt)
            self.pedestrians = [p for p in self.pedestrians if not p.finished]
            if self.phase_timer >= min_ped_timer:
                self.ped_active = False
                self.ped_status = "DON'T WALK"
                self.phase = Phase.green_n_s
                self.phase_timer = 0.0
        elif self.phase == Phase.EMERGENCY:
            self.ped_status = "DON'T WALK"
# Must stay in emergency mode until no emergency vehicles remain in or approaching intersection
            active_emergency = False
            for v in vehicles:
                if v.emergency and v.direction == self.emergency_direction:
# Relevant if not fully past the scene
                    if not v.removed:
                        active_emergency = True
                        break
            if not active_emergency:
                self.end_emergency_phase()
    def draw_status(self, surface):
        draw_text(surface, f"Phase: {self.phase.name}", BIG_FONT, black, (20, 20))
        draw_text(surface, f"Pedestrian: {self.ped_status}", FONT, black, (20, 60))
        draw_text(surface, "Controls: P = pedestrian request | E = emergency | R = normal vehicle", SMALL_FONT, black, (20, 95))
        if self.ped_request and not self.ped_active:
            draw_text(surface, "Ped request registered", SMALL_FONT, orange, (20, 120))
        if self.emergency_request:
            draw_text(surface, "EMERGENCY PRIORITY ACTIVE", FONT, red, (20, 145))


# Vehicle System Class to Spawn/Create Motor and Emergency Vehicles
class VehicleSystem:
    def __init__(self, controller):
        self.controller = controller
        self.vehicles = []
        self.spawn_timer = 0.0

    def spawn_normal(self):
        direction = random.choice(list(Direction))
        self.vehicles.append(Vehicle(direction, emergency=False))

    def spawn_emergency(self):
        direction = random.choice(list(Direction))
        self.vehicles.append(Vehicle(direction, emergency=True))
        self.controller.request_emergency(direction)

    def update(self, dt):
        self.spawn_timer += dt
        if self.spawn_timer >= spawn_interval:
            self.spawn_timer = 0.0
            if random.random() < 0.65:
                self.spawn_normal()
        for v in self.vehicles:
            v.update(dt, self.controller, self.vehicles)
        self.vehicles = [v for v in self.vehicles if not v.removed]

    def draw(self, surface):
        for v in self.vehicles:
            v.draw(surface)


# Draw the Intersection
def draw_dashed_center_line(surface, start, end, vertical=False):
    dash_len = 20
    gap = 12
    if vertical:
        y = start
        while y < end:
            pygame.draw.line(surface, yellow, (center_X, y), (center_X, min(y + dash_len, end)), 3)
            y += dash_len + gap
    else:
        x = start
        while x < end:
            pygame.draw.line(surface, yellow, (x, center_Y), (min(x + dash_len, end), center_Y), 3)
            x += dash_len + gap

def draw_crosswalk(surface, rect, vertical=False):
    stripe = 8
    gap = 8
    if vertical:
        y = rect.y
        while y < rect.y + rect.h:
            pygame.draw.rect(surface, cross, (rect.x, y, rect.w, stripe))
            y += stripe + gap
    else:
        x = rect.x
        while x < rect.x + rect.w:
            pygame.draw.rect(surface, cross, (x, rect.y, stripe, rect.h))
            x += stripe + gap

def draw_signal(surface, x, y, state, label):
    housing = pygame.Rect(x, y, 26, 78)
    pygame.draw.rect(surface, black, housing, border_radius=6)
    colors = [dark_gray, dark_gray, dark_gray]
    if state == SignalState.red:
        colors[0] = red
    elif state == SignalState.yellow:
        colors[1] = yellow
    elif state == SignalState.green:
        colors[2] = green
    for i, c in enumerate(colors):
        pygame.draw.circle(surface, c, (x + 13, y + 13 + i * 24), 9)
    draw_text(surface, label, SMALL_FONT, black, (x - 8, y + 84))

def draw_ped_signal(surface, x, y, active, countdown_text):
    box = pygame.Rect(x, y, 70, 46)
    pygame.draw.rect(surface, black, box, border_radius=4)
    pygame.draw.rect(surface, white, box.inflate(-4, -4), border_radius=4)
    if active:
        text = "WALK"
        color = green
    else:
        text = "WAIT"
        color = red
    draw_text(surface, text, SMALL_FONT, color, (x + 10, y + 4))
    draw_text(surface, countdown_text, SMALL_FONT, black, (x + 10, y + 23))

def draw_scene(surface, controller, vehicle_system):
    surface.fill(grass)
# Draw the Roads
    pygame.draw.rect(surface, road, vertical_road)
    pygame.draw.rect(surface, road, horizontal_road)
# Draw the Lane markings
    draw_dashed_center_line(surface, 0, height, vertical=True)
    draw_dashed_center_line(surface, 0, width, vertical=False)
# Draw the Edge lane lines
    pygame.draw.line(surface, lane, (center_X - road_W // 2 + 2 * lane_W, 0), (center_X - road_W // 2 + 2 * lane_W, height), 2)
    pygame.draw.line(surface, lane, (center_X + road_W // 2 - 2 * lane_W, 0), (center_X + road_W // 2 - 2 * lane_W, height), 2)
    pygame.draw.line(surface, lane, (0, center_Y - road_W // 2 + 2 * lane_W), (width, center_Y - road_W // 2 + 2 * lane_W), 2)
    pygame.draw.line(surface, lane, (0, center_Y + road_W // 2 - 2 * lane_W), (width, center_Y + road_W // 2 - 2 * lane_W), 2)
# Draw the Stop lines
    pygame.draw.line(surface, white, (center_X - road_W // 2, n_s_stop_N_Y), (center_X + road_W // 2, n_s_stop_N_Y), 3)
    pygame.draw.line(surface, white, (center_X - road_W // 2, n_s_stop_S_Y), (center_X + road_W // 2, n_s_stop_S_Y), 3)
    pygame.draw.line(surface, white, (e_w_stop_W_X, center_Y - road_W // 2), (e_w_stop_W_X, center_Y + road_W // 2), 3)
    pygame.draw.line(surface, white, (e_w_stop_E_X, center_Y - road_W // 2), (e_w_stop_E_X, center_Y + road_W // 2), 3)
# Draw the Crosswalks
    draw_crosswalk(surface, north_cross, vertical=False)
    draw_crosswalk(surface, south_cross, vertical=False)
    draw_crosswalk(surface, west_cross, vertical=True)
    draw_crosswalk(surface, east_cross, vertical=True)
# Draw the Signals
    draw_signal(surface, center_X - road_W // 2 - 55, center_Y - road_W // 2 - 95, controller.ns_signal, "NS")
    draw_signal(surface, center_X + road_W // 2 + 25, center_Y + road_W // 2 + 15, controller.ns_signal, "NS")
    draw_signal(surface, center_X + road_W // 2 + 25, center_Y - road_W // 2 - 95, controller.ew_signal, "EW")
    draw_signal(surface, center_X - road_W // 2 - 55, center_Y + road_W // 2 + 15, controller.ew_signal, "EW")
# Draw the Pedestrian signal boxes
    ped_count = str(math.ceil(controller.ped_countdown)) if controller.ped_active else "--"
    draw_ped_signal(surface, center_X - 150, center_Y - road_W // 2 - 90, controller.ped_active, ped_count)
    draw_ped_signal(surface, center_X + 80, center_Y + road_W // 2 + 45, controller.ped_active, ped_count)
# Draw the Pedestrians
    for p in controller.pedestrians:
        p.draw(surface)
# Draw the Vehicles
    vehicle_system.draw(surface)
# Draw the UI text
    controller.draw_status(surface)


def main():
    controller = TrafficController()
    vehicle_system = VehicleSystem(controller)
    rtos = RTOS()
# RTOS periodic tasks
    rtos.add_task(Task("traffic_control", 0.05, 1, lambda: controller.update(0.05, vehicle_system.vehicles)))
    rtos.add_task(Task("vehicle_system", 0.02, 2, lambda: vehicle_system.update(0.02)))
    rtos.add_task(Task("background", 0.10, 3, lambda: None))  # placeholder low-priority task
    running = True
    sim_accumulator = 0.0
    while running:
        frame_dt = CLOCK.tick(60) / 1000.0
        sim_accumulator += frame_dt
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_p:
                    controller.request_pedestrian()
                elif event.key == pygame.K_e:
                    vehicle_system.spawn_emergency()
                elif event.key == pygame.K_r:
                    vehicle_system.spawn_normal()
        while sim_accumulator >= 0.01:
            rtos.update(0.01)
            sim_accumulator -= 0.01
        draw_scene(SCREEN, controller, vehicle_system)
        pygame.display.flip()
    pygame.quit()

if __name__ == "__main__":
    main()

