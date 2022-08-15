import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Operator overloading

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Point(x, y)

    def __truediv__(self, other):
        x = self.x/other
        y = self.y/other

        return Point(x, y)

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Point(x, y)

    def __rmul__(self, other):
        x = self.x * other
        y = self.y * other
        return Point(x, y)

    def __mul__(self, other):
        x = self.x * other
        y = self.y * other
        return Point(x, y)

    def __str__(self):
        return "({0},{1})".format(self.x, self.y)
    

def magnitude(p):
    """Returns magnitude of a vector"""
    return math.sqrt(p.x**2 + p.y**2)

def slope(p1, p2):
    p = p2 - p1
    return (p.y/p.x)

def getPerpendicularVector(A, B, C):
    """Returns vector peperndicular to the angle bisector of the angle inscribed between neighboring points."""

    AB = B-A
    BC = C-B

    v1 = (AB)/(magnitude(AB))
    v2 = (BC)/(magnitude(BC))

    try:
        return ((v1+v2)/magnitude(v1+v2))
    except:
        return Point(1, 0)