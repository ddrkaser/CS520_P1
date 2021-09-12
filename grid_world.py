#!/usr/bin/env python3

import pandas as pd
import numpy as np

# creating gridworld
g_length = 40
g_width = 40
p = 0.3
g= np.random.choice([0,1],g_length*g_width, [1-p,p]).reshape(g_length,g_width)
