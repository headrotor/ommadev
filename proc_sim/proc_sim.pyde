
from classes import Vertex as vert
from classes import Ommatid
import cPickle as pickle


verts = []
faces = []
ffaces = []  # fat faces, each face has subfaces



def setup():
    global cindex
    global om
    #globals, ugly but need them here
    cindex = 0
    # if no pickle file, compute ommatid and pickle it
    #if True:
    om2 = Ommatid()
    pickle.dump(om2, open( "omma.p", "wb" ) )
    # else load it from file
    #else:
    om = pickle.load(open( "omma.p", "rb" ) )
        
            
    size(800, 600, P3D)
    for f in om.chan:
        f.c = color(0)
    print cindex

def incr_cc(c):
    return c + 1

def draw():
    global cindex
    global om
    background(64)
    textureMode(NORMAL)
    lights()
    noStroke()
    pushMatrix()
    translate(width / 2, height / 2)
    rotation = 0
    zoom = 8
    rotateY(map(mouseX, 0, width, 0, 2 * PI) + rotation)
    rotateX(map(mouseY, 0, height, 0, 2 * PI))
    scale(width / zoom, width / zoom, width / zoom)
    # box(100)
    for v in om.verts:
        draw_vert(v)
    # for f in faces:
    #  draw_face(f)
    n = 0
    # for i in range(20):
    #  draw_face(faces[n + i])
    if mousePressed:
        force_vals()
    om.gray_scott()
    for qf in om.qfaces:
        draw_fface(qf)
    popMatrix()
    # print cc3

def mouseClickedOLD():
    #global cindex
    om.chan[cindex].c = color(0)
    for n in om.chan[cindex].n:
        n.c = color(0)
    cindex = cindex + 1
    println(cindex)
    if cindex >= len(om.chan):
        cindex = 0
    om.chan[cindex].c = color(64, 32, 32)
    for n in om.chan[cindex].n:
        n.c = color(128)

 

#def mouseClicked():

def force_vals():
    global cindex
    om.chan[cindex].U = 1.0
    om.chan[cindex].V = 0.0
    for n in om.chan[cindex].n:
        n.U = 1.0
        n.V = 0.0
    

def keyPressed():
    global cindex
    cindex = cindex + 1
    println(cindex)
    if cindex >= len(om.chan):
        cindex = 0

def draw_vert(v):
    # for debugging, put a box at this vertex
    pushMatrix()
    translate(v.x, v.y, v.z)
    box(0.1)
    popMatrix()

def draw_fface(qf):
    # textureMode(NORMAL);
    # f.v[0].printme();
    noStroke()

    for f in qf.f:
    # draw this face in its proper color...
        #fill(f.c)
        fill(color(f.U*255, 0, f.V*255));
        beginShape(TRIANGLE_STRIP)
        for j in range(3):
            vertex(f.v[j].x, f.v[j].y, f.v[j].z)
        vertex(f.v[0].x, f.v[0].y, f.v[0].z)
        endShape()


def draw_face(f):

    beginShape(TRIANGLE_STRIP)
    # noStroke();
    # vertex(f.v[0].x,f.v[0].y,f.v[0].z);
    # vertex(f.v[1].x,f.v[1].y,f.v[1].z);
    # vertex(f.v[2].x,f.v[2].y,f.v[2].z);
    # vertex(f.v[0].x,f.v[0].y,f.v[0].z);
    vertex(verts[f.v[0]].x, verts[f.v[0]].y, verts[f.v[0]].z)
    vertex(verts[f.v[1]].x, verts[f.v[1]].y, verts[f.v[1]].z)
    vertex(verts[f.v[2]].x, verts[f.v[2]].y, verts[f.v[2]].z)
    vertex(verts[f.v[0]].x, verts[f.v[0]].y, verts[f.v[0]].z)

    endShape()
    # f.printme()

