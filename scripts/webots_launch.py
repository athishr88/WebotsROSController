#!/usr/bin/env python3.8

import os

path_to_webots_world = "/home/athishr88/Desktop/scrap/just_arena/worlds/empty.wbt"
command = "webots " + path_to_webots_world
os.system(command)
os.system("python3 controller_for_moose.py")