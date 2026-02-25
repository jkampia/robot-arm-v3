COLOR_RGB = {
    'red':       (255, 0, 0),
    'green':     (0, 255, 0),
    'blue':      (0, 0, 255),
    'yellow':    (255, 255, 0),
    'cyan':      (0, 255, 255),
    'magenta':   (255, 0, 255),
    'white':     (255, 255, 255),
    'black':     (0, 0, 0),
    'orange':    (255, 165, 0),
    'purple':    (128, 0, 128),
    'gray':      (128, 128, 128),
    'light_gray': (200, 200, 200),
    'dark_gray': (64, 64, 64),
    'brown':     (139, 69, 19),
    'pink':      (255, 192, 203),
    'lime':      (0, 255, 0),
    'navy':      (0, 0, 128),
    'teal':      (0, 128, 128),
    'olive':     (128, 128, 0),
    'cream':     (241, 240, 239)

}

def pickRandomColor():
    """
    Pick a random color from the COLOR_RGB dictionary.
    """
    import random
    color_name = random.choice(list(COLOR_RGB.keys()))
    return color_name, COLOR_RGB[color_name]
