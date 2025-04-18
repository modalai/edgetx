#!/usr/bin/env python3

import glob
import os

for f in glob.glob("*.bmp") + glob.glob("*.BMP"):
    t = f.title()
    if f != t:
        os.rename(f, t)
