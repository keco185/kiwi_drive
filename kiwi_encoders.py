class kiwi_encoders:
    """
    Determines the robot position by keeping track of encoder displacements
    """
    def __init__(self, encoder1, encoder2, encoder3):
        self.x = 0.0
        self.y = 0.0
        self.r = 0.0
        self.encoder1 = encoder1
        self.encoder2 = encoder2
        self.encoder3 = encoder3
        self.encoder1.set_update_func(lambda dist, old_dist: self.update_pos(dist-old_dist,0,0))
        self.encoder2.set_update_func(lambda dist, old_dist: self.update_pos(0,dist-old_dist,0))
        self.encoder3.set_update_func(lambda dist, old_dist: self.update_pos(0,0,dist-old_dist))

    def update_pos(self, d1, d2, d3):
        """
        Use new changes in encoder distances to update robot position
        """
        # TODO: Rotate dx and dy vectors by the angle r
        self.x += (2*d3-d1-d2)
        self.y += (d2-d1)
        self.r += (d1+d2+d3)

    def get_pos(self):
        """
        Get the lastest robot position estimate
        """
        return (self.x/3, 3**0.5*self.y/3, self.r/3)
