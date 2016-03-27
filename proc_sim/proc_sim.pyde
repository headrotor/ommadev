
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
    if True:
        om = Ommatid(3)
        pickle.dump(om, open( "omma.p", "wb" ) )
        print "dumping pickle file"
        # else load it from file
    else:
        om = pickle.load(open( "omma.p", "rb" ) )
        print "reading pickle file"
            
    size(600, 600, P3D)
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
    zoom = 4
    rotateX(PI/2)
    rotateZ(map(mouseX, 0, width, 0, 2 * PI) + rotation)
    rotateX(map(mouseY, 0, height, 0, 2 * PI))
    scale(width / zoom, width / zoom, width / zoom)
    # box(100)
    for v in om.verts:
        #v.rot(map(mouseX, 0, width, 0, 2 * PI), 0,1,0)
        draw_vert(v)
    # for f in faces:
    #  draw_face(f)
    n = 0
    draw_zboxes()
    draw_yboxes()
    if mousePressed:
        force_vals()
    #om.gray_scott()
    om.iter_wave()
    for qf in om.qfaces:
        #draw_fface(qf)
        draw_qface(qf)
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

def force_vals_GS():
    global cindex
    om.chan[cindex].U = 1.0
    om.chan[cindex].V = 0.0
    for n in om.chan[cindex].n:
        n.U = 1.0
        n.V = 0.0

def force_vals():
    om.chan[cindex].V += 100.0
    if om.chan[cindex].V > 255:
        om.chan[cindex].V = 255
    

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
    
def draw_zboxes():
    for i in range(100):
        pushMatrix()
        translate(0, 0, (i-50)*.2)
        box(0.1)
        popMatrix()

def draw_yboxes():
    for i in range(100):
        pushMatrix()
        translate(0, (i-50)*.12, 0)
        fill(0,0,128)
        box(0.1)
        popMatrix()
        

def draw_fface(qf):
    # textureMode(NORMAL);
    # f.v[0].printme();
    noStroke()

    for i, f in enumerate(qf.f):
    # draw this face in its proper color...
        #fill(color(f.U*255, 0, f.V*255));
        sc = 2.0
        fill(color(sc*f.V, 80, 255-(sc*f.V)));
        beginShape(TRIANGLE_STRIP)
        for j in range(3):
            vertex(f.v[j].x, f.v[j].y, f.v[j].z)
        vertex(f.v[0].x, f.v[0].y, f.v[0].z)
        endShape()


def draw_qface(qf):

    fill(128)
    beginShape(TRIANGLE_STRIP)
    # noStroke();
    # vertex(f.v[0].x,f.v[0].y,f.v[0].z);
    # vertex(f.v[1].x,f.v[1].y,f.v[1].z);
    # vertex(f.v[2].x,f.v[2].y,f.v[2].z);
    # vertex(f.v[0].x,f.v[0].y,f.v[0].z);
    vertex(qf.v[0].x, qf.v[0].y, qf.v[0].z)
    vertex(qf.v[1].x, qf.v[1].y, qf.v[1].z)
    vertex(qf.v[2].x, qf.v[2].y, qf.v[2].z)
    vertex(qf.v[0].x, qf.v[0].y, qf.v[0].z)

    endShape()
    # f.printme()

