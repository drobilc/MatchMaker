class Face(object):

    def __init__(self, topLeft=(0, 0), bottomRight=(0, 0)):
        self.x1 = topLeft[0]
        self.y1 = topLeft[1]
        self.x2 = bottomRight[0]
        self.y2 = bottomRight[1]
    
    def left(self):
        return self.x1
    
    def right(self):
        return self.x2
    
    def top(self):
        return self.y1
    
    def bottom(self):
        return self.y2
    
    def center_point(self):
        return ((self.x2 + self.x1) / 2.0, (self.y2 + self.y1) / 2.0)
    
    def to_list(self):
        return [self.x1, self.y1, self.x2, self.y2]