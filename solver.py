import kiwisolver

fudge_factor = 0.01

class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        self.solver = kiwisolver.Solver()
        
    def solve(self):
        joints = self.joints
        layers = self.layers
        
        
        for l in layers:
            l = self.define_shape(l)
            
            if l.fixed == True:
                self.add_fixed(l)
                
        print(self.solver.dumps())
        
        print(joints)
        for jointlayer in joints:
            print(jointlayer)
            for jp in joints[jointlayer]:
                for l in layers:
                    print(l.name)
                    #print(l)
                    for p in l.straight_pairs:
                        if(self.encloses(p, jp)):
                            print(jointlayer, l.name)
                            #joint_assoc[jointlayer].append(l.name)
        #print(joint_assoc)
        return layers
        
    #Todo: only constrain first point to global coordinate
    #Or: constrain bounding box to also fix rotation
    def add_fixed(self, layer):
        i = 0
        
        for p in layer.straight_pairs:
            #path4581_p1_x
            name = layer.name + '_p' + str(i) + '_'
            nameX = name + 'x'
            nameY = name + 'y'
            x = kiwisolver.Variable(nameX)
            y = kiwisolver.Variable(nameY)
            self.solver.addConstraint((x == p[0]) | "strong")
            self.solver.addConstraint((y == p[1]) | "strong")
            i += 1
        return layer
            
    # Give points relative constraints to previous point 
    def define_shape(self, layer):
        i = 0
        x_p = 0
        y_p = 0
        xp_p = 0.0
        yp_p = 0.0
        
        for p in layer.straight_pairs:
            #path4581_p1_x
            name = layer.name + '_p' + str(i) + '_'
            nameX = name + 'x'
            nameY = name + 'y'
            x = kiwisolver.Variable(nameX)
            y = kiwisolver.Variable(nameY)
            xp = p[0]
            yp = p[1]
            if i > 0:
                x_constraint = (x == (x_p + (xp - xp_p)))
                y_constraint = (y == (y_p + (yp - yp_p)))
                self.solver.addConstraint(x_constraint | "strong")
                self.solver.addConstraint(y_constraint | "strong")
                x_p = x
                y_p = y
                xp_p = xp
                yp_p = yp
            i += 1
        return layer

        
    #Naive bounding box implementation
    def encloses(self, point, joint):
        box = joint.bbox()
        if(point[0] >= box[0]):
            if(point[0] <= box[1]):
                if(point[1] >= box[2]):
                    if(point[1] <= box[3]):
                        return True;
        return False