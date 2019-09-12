import kiwisolver


class Solver(object):
    def __init__(self, joints, layers):
        self.joints = joints
        self.layers = layers
        
    def solve(self):
        joints = self.joints
        layers = self.layers
        #joint_assoc = {}
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

        
    #Naive bounding box implementation
    def encloses(self, point, joint):
        box = joint.bbox()
        if(point[0] >= box[0]):
            if(point[0] <= box[1]):
                if(point[1] >= box[2]):
                    if(point[1] <= box[3]):
                        return True;
        return False