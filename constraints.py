import kiwisolver

fudge_factor = 0.01
strength_define = "strong"
strength_fixed = "strong"
strength_joint = "medium"

class Constraints(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        self.solver = kiwisolver.Solver()
        self.variables = {}
        
    def solve(self):
        joints = self.joints
        layers = self.layers
        
        
        for l in layers:
            #Todo: check that shape actually needs constraints before defining
            l = self.define_shape(l)
            
            #Todo: Finish / test
            if l.fixed == True:
                self.add_fixed(l)
        
        print(joints)
        for jointlayer in joints:
            print(jointlayer)
            for jp in joints[jointlayer]:
                for l in layers:
                    print(l.name)
                    #print(l)
                    i = 0
                    for p in l.straight_pairs:
                        if(self.encloses(p, jp)):
                            print(jointlayer, l.name, i, p)
                            self.add_joint(jointlayer, l.name, i, p)
                            #joint_assoc[jointlayer].append(l.name)
                        i += 1
        #print(joint_assoc)
                
        print(self.solver.dumps())
        print(self.solver)
        print(dir(self.solver))
        print(dir(kiwisolver.Variable()))
        print(layers)
        # print(dir(kiwisolver.Expression())) #needs term
        # print(dir(kiwisolver.Term()))   #needs variable
        print(dir(kiwisolver.Term(self.variables['path4518_p3_y'])))
     
        print(self.variables)
        print(self.variables['path4518_p3_y'])
        print(self.variables['path4518_p3_y'].value())
        print(self.variables['path4518_p3_x'].value())
        print(self.variables['path4520_p1_y'])
        print(self.variables['path4520_p1_y'].value())
        print(self.variables['path4520_p1_x'].value())
        self.solver.updateVariables()
        print(self.variables['path4518_p3_y'])
        print(self.variables['path4518_p3_y'].value())
        print(self.variables['path4518_p3_x'].value())
        print(self.variables['path4520_p1_y'])
        print(self.variables['path4520_p1_y'].value())
        print(self.variables['path4520_p1_x'].value())
        return self.incorporate(self.variables, layers)
        # return layers
        
    def incorporate(self, variables, layers):
        a = 0
        for l in layers:
            print(l.name)
            #print(l)
            i = 0
            for p in l.straight_pairs:
                # print(l.name, i, p)
                # self.add_joint(jointlayer, l.name, i, p)
                layername = l.name + '_p' + str(i) + '_'
                layernameX = layername + 'x'
                layernameY = layername + 'y'
                print(l.name, layernameX, i, p, variables[layernameX].value())
                #joint_assoc[jointlayer].append(l.name)
                layers[a].straight_pairs[i][0] = variables[layernameX].value()
                layers[a].straight_pairs[i][1] = variables[layernameY].value()
                i += 1
            a += 1
        return layers
        
        
    def add_joint(self, jname, lname, pname, p):

        #path4581_p1_x
        layername = lname + '_p' + str(pname) + '_'
        layernameX = layername + 'x'
        layernameY = layername + 'y'
        # lx = kiwisolver.Variable(layernameX)
        # ly = kiwisolver.Variable(layernameY)
        lx = self.variables[layernameX]
        ly = self.variables[layernameY]
        
        jointname = 'joint_' + str(jname) + '_'
        jointnameX = jointname + 'x'
        jointnameY = jointname + 'y'
        if jointnameX not in self.variables.keys():
            jx = kiwisolver.Variable(jointnameX)
            jy = kiwisolver.Variable(jointnameY)
            self.variables[jointnameX] = jx
            self.variables[jointnameY] = jy
        else:
            jx = self.variables[jointnameX]
            jy = self.variables[jointnameY]
        
        self.solver.addConstraint((lx == jx) | strength_joint)
        self.solver.addConstraint((ly == jy) | strength_joint)
            
            
            
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
            self.solver.addConstraint((x == p[0]) | strength_define)
            self.solver.addConstraint((y == p[1]) | strength_define)
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
            self.variables[nameX] = x
            self.variables[nameY] = y
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