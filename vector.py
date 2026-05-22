import math


class Vector2D:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Vector2D({self.x}, {self.y})"

    def __str__(self):
        return f"Vector({self.x}, {self.y})"

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self

    def __mul__(self, other):
        return Vector2D(self.x * other, self.y * other)

    def __truediv__(self, other):
        if other == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / other, self.y / other)

    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    def __lt__(self, other):
        return self.length() < other.length()

    def __gt__(self, other):
        return self.length() > other.length()

    def get_angle(self):
        if self.x == 0 and self.y == 0:
            return 0
        return math.atan2(self.y, self.x)

    def set_angle(self, angle):
        mag = self.length()
        self.x = mag * math.cos(angle)
        self.y = mag * math.sin(angle)

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def normalized(self):
        mag = self.length()
        if mag == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / mag, self.y / mag)

    def sin(self):
        mag = self.length()
        if mag == 0:
            return 0
        return self.y / mag

    def cos(self):
        mag = self.length()
        if mag == 0:
            return 0
        return self.x / mag

    @staticmethod
    def minimal(vector1, vector2):
        return vector1 if vector1 < vector2 else vector2

    @staticmethod
    def maximal(vector1, vector2):
        return vector1 if vector1 > vector2 else vector2

    @staticmethod
    def distance(vector1, vector2):
        return (vector1 - vector2).length()

    @staticmethod
    def perpendicular(vector):
        return Vector2D(-vector.y, vector.x)

    @staticmethod
    def list(vector):
        return [vector.x, vector.y]