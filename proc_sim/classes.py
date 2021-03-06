
""" holds data for a face corner, assumed to be at sphere surface """
class Vertex(object):

    def __init__(self, px, py, pz):
        self.x = px
        self.y = py
        self.z = pz

    def unitize(self):
        """normalize to unit length"""
        len = sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        self.x = self.x / len
        self.y = self.y / len
        self.z = self.z / len

    def uni_scale(self, s):
        self.x = self.x * s
        self.y = self.y * s
        self.z = self.z * s

    def dist2(self, v):
        """ return squared distance with vertex v """
        dx = self.x - v.x
        dy = self.y - v.y
        dz = self.z - v.z
        return dx * dx + dy * dy + dz * dz

    def printme(self):
        print "vertex index %d %f %f %f" % (self.i, self.x, self.y, self.z)

        
    def rot(self, a, l, m, n):
        """ rotate this vector through angle a
        around axis l, m, n using
        Rodrigues formula https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula """
        # axis should be unit vector
        x, y, z = self.x, self.y, self.z
        # could do this so much better in numpy!
        self.x = x*cos(a) + (1 - cos(a))*(l*l*x + l*m*y + l*n*z) + (m*z - n*y)*sin(a)
        self.y = y*cos(a) + (1 - cos(a))*(m*l*x + m*m*y + m*n*z) + (n*x - l*z)*sin(a)
        self.z = z*cos(a) + (1 - cos(a))*(n*l*x + n*m*y + n*n*z) + (l*y - m*x)*sin(a)
       

#""" face divided into four subfaces """
# call with 3 corner Vertexes of icosahedral face
class Quadface:

    def __init__(self, va, vb, vc):

          #self.v = [va.unitize(), vb.unitize(), vc.unitize()]
        va.unitize()
        vb.unitize()
        vc.unitize()
        # pre-rotate so top, bottom plane parallel to xy axis
        self.v = [va, vb, vc]
        #self.vi = [va.i, vb.i, vc.i]  
        self.f = []  # list of subfaces
        self.i = -1  # index such that qfaces[i]=this

        # now subdivide
        # add midpoint v3
        self.add_midpoint_vertex(va, vb)
        # add midpoint v4
        self.add_midpoint_vertex(vb, vc)
        # add midpoint v5
        self.add_midpoint_vertex(va, vc)

        # now add subfaces

        #        va
        #       /R\
        #     v3---v5
        #    / G\W/ B\
        #   vb---v4--vc
        self.add_subface(self.v[0], self.v[3], self.v[5])
        # middle, through midpoints
        self.add_subface(self.v[3], self.v[4], self.v[5])
        self.add_subface(self.v[3], self.v[1], self.v[4])
        self.add_subface(self.v[4], self.v[2], self.v[5])

        # color by native orientation
        self.f[0].c = color(128, 0, 0)  # red face
        self.f[1].c = color(128, 128, 128)  # white face
        self.f[2].c = color(0, 128, 0)  # green face
        self.f[3].c = color(0, 0, 128)  # blue face

    def add_subface(self, v1, v2, v3):
        self.f.append(subFace(v1, v2, v3))

    def add_midpoint_vertex(self, v1, v2):
        tx = v1.x + v2.x
        ty = v1.y + v2.y
        tz = v1.z + v2.z
        tv = Vertex(tx, ty, tz)
        tv.unitize()
        self.v.append(tv)


class subFace:

    """ A subface is a a channel, has RGB color, sensor value, and spherical coordinates
        There are four subfaces per quadface"""

    def __init__(self, va, vb, vc):
        # make a new subface from these vectors
        self.v = [va, vb, vc]  # list of vertices
        # unit vector from origin through midpoint of subface
        self.nv = Vertex((va.x + vb.x + vc.x),
                         (va.y + vb.y + vc.y),
                         (va.z + vb.z + vc.z))
        self.nv.unitize()  # normalize to get unit vector
          # spherical coordinates of this face
        self.th = acos(self.nv.z)
        self.ph = atan2(self.nv.x, self.nv.y)
        # indices of neighbors sharing sides (added later)
        self.en = None
        # list of neighboring subfaces
        self.n = []
        # unique numerical index (added later)
        self.i = None

        

        # reaction-diffusion coefficients
        self.rc = RDConstants()

        self.U = 1.0
        self.V = 0.0
        
        self.nextU = 0.0  # temproary placeholder for UV calculations
        self.nextV = 0.0  # temproary placeholder for UV calculations
        # precompute some multiplicative constants
        self.dcV = self.rc.diffusion_rate_V*self.rc.del_T/self.rc.del_X2
        self.dcU = self.rc.diffusion_rate_U*self.rc.del_T/self.rc.del_X2
        self.rcUV = self.rc.del_T*self.rc.reaction_rate 

    def init_UV(self):
        if self.i < 20 :
            self.U = 1+random(-.01, .01);
            self.V = 0+random(-.01, .01);
        else:
            self.U =  0+random(-.01, .01);
            self.V = .5+random(-.01, .01);

        self.F = .04 + self.i*.05/80;

    def diffuse(self):
        """ one pass of diffusion"""
        self.nextV = self.V + self.dcV * self.lapV()
        self.nextU = self.U + self.dcU * self.lapU()

    def update(self):
        self.U = self.nextU
        self.V = self.nextV

    def react(self):
        """ one pass of reaction"""
        tempU = self.U
        self.U += self.rcUV*(-self.U*self.V*self.V + self.F*(1-self.U))
        self.V += self.rcUV*(tempU*self.V*self.V - (self.F+self.rc.k)*self.V)
            
    def lapU(self):
        """ compute laplacian for U variable of this chan"""
        l = -3*self.U;
        for n in self.n:
            l += n.U
        return l

    def lapV(self):
        """ compute laplacian for V variable of this subface """
        l = -3*self.V;
        for n in self.n:
            l += n.V
        return l

    # wave equation
    def init_wave(self):
        # if self.i < 20:
        self.U = 0.0 
        #self.V = 0 + random(-0.5, 0.5)
        self.V = 0


    def update_wave(self, damp=0.999):
        if self.i == 0 and False:
            print "iter: nv %f  V %f  U %f" % (self.nextV, self.V, self.U)
        self.U =  self.V
        self.V = damp * self.nextV    
 

    # http://www.mtnmath.com/whatrh/node66.html
    def iter_wave(self):
        speed = 0.05
        V = speed * speed * (self.lapV())
        V += 2.0 * self.V
        # print "V2:" + str(V)
        # U is [t-1] term
        V -= self.U
        self.nextV = V


    def printme(self):
        # for debugging, print some things
        print "index %d" % self.i
        print "neighbors: " + str(self.en)


##########################################################################
class Ommatid:

    """ Ommatid  triangulated icosahedron. 80 faces, subdivided from 20 icosahedral faces"""

    def __init__(self, order=0):

        # Icosahedral vertices, only really need these for construction
        self.order = order
        self.verts = []
        self.faces = []  # Icosahedral faces, only used for construction
        # quad (icosahedral) faces, each face has 4 subface channels

        self.qfaces = []
        self.sfaces =[] # for subdivided faces
        self.chan = []   # subface channels, each has sensor/RGB
        self.init()
        for f in self.chan:
            f.init_UV()




    def find_adjacent(self, c):
        from collections import Counter
        """ find adjacent chan faces by comparing vertices"""
        en = []  # list of neighbors
        print "searching nabes of chan %d" % c.i
        # list all qfs sharing vectors with this qf/
        m = []  # list of chans that share at least one vector with this
        for v in c.v:
            # print "  matching " + str(i)
            for c2 in self.chan:
                for v2 in c2.v:
                    if v.dist2(v2) < 0.01:
                        # vectors match, append this face
                        # print "matched " + str(q2.vi)
                        m.append(c2.i)
        print "   matched " + str(m)
        # Now have list of chans sharing vectors in m. Neighbors will share
        # exactly two vertices and will have multiplicity 2 in this list.
        # find them using Counter:
        ncount = Counter(m)
        en = []
        for k, kcount in ncount.iteritems():
            if kcount == 2:
                en.append(k)
        c.en = en
        c.n = []
        for i in c.en:
            c.n.append(self.chan[i])
        print "neighbors: " + str(c.en)

    def add_qface(self, vi1, vi2, vi3):
        """ given 3 vertex indexes, add the quadface"""

        qf = Quadface(self.verts[vi1],
                      self.verts[vi2],
                      self.verts[vi3])
        self.qfaces.append(qf)

    def init(self):
        """ construct icosahedron and subdivide faces """
        phi = (sqrt(5) + 1) / 2.0

        self.verts.append(Vertex(-1, phi, 0))
        self.verts.append(Vertex(1, phi, 0))
        self.verts.append(Vertex(-1, -phi, 0))
        self.verts.append(Vertex(1, -phi, 0))

        self.verts.append(Vertex(0, -1, phi))
        self.verts.append(Vertex(0, 1, phi))
        self.verts.append(Vertex(0, -1, -phi))
        self.verts.append(Vertex(0, 1, -phi))

        self.verts.append(Vertex(phi, 0, -1))
        self.verts.append(Vertex(phi, 0, 1))
        self.verts.append(Vertex(-phi, 0, -1))
        self.verts.append(Vertex(-phi, 0, 1))

        # add indexes
        for i, v in enumerate(self.verts):
            v.i = i
            # pre-rotate so planes are parallel
            v.rot(20*PI/180, 0,1,0)

            # v.printme()

        # 5 faces around point 0
        self.add_qface(0, 11, 5)
        self.add_qface(0, 5, 1)
        self.add_qface(0, 1, 7)
        self.add_qface(0, 7, 10)
        self.add_qface(0, 10, 11)

        # 5 adjacent faces
        self.add_qface(5, 11, 4)
        self.add_qface(1, 5, 9)
        self.add_qface(7, 1, 8)
        self.add_qface(11, 10, 2)
        self.add_qface(10, 7, 6)

        # 5 faces around point 3
        self.add_qface(3, 9, 4)
        self.add_qface(3, 4, 2)
        self.add_qface(3, 2, 6)
        self.add_qface(3, 6, 8)
        self.add_qface(3, 8, 9)

        # 5 adjacent faces
        self.add_qface(4, 9, 5)
        self.add_qface(2, 4, 11)
        self.add_qface(6, 2, 10)
        self.add_qface(8, 6, 7)
        self.add_qface(9, 8, 1)

        # make lists of subfaces and vertices

        # now subdivide each subface
        if True:
            for i, qf in enumerate(self.qfaces):
                qf.i = i
                qf.order = 1
                for sf in qf.f:
                    qf = Quadface(sf.v[0], sf.v[1], sf.v[2])
                    self.sfaces.append(qf)

        #for i, qf in enumerate(self.qfaces):

        for i, qf in enumerate(self.sfaces):
            qf.i = i
            for subface in qf.f:
                self.chan.append(subface)


        # label channels with index
        for i, c in enumerate(self.chan):
            c.i = i

        for c in self.chan:
            self.find_adjacent(c)

        for i in range(4):
            self.chan[i].printme()


    def gray_scott(self):
        for c in self.chan:
            c.diffuse()
        for c in self.chan:
            c.update()    
        for c in self.chan:
            c.react()
    
    def iter_wave(self):
        """compute one iterationof the wave equation"""
        for c in self.chan:
            c.iter_wave()
        for c in self.chan:
            c.update_wave()



"""holds constants for reaction-diffusion"""


class RDConstants(object):

    def __init__(self):
        self.reaction_rate = .4
        self.diffusion_rate_U = .04
        self.diffusion_rate_V = .01
        self.del_T = 4
        self.del_X2 = 1.0
    
        self.F = .04
        self.k = .06
        
    def incrF(self,incr):
        self.F += incr
    
