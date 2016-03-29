#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
geomhack.py: first cut at using geometry from omma_geom

"""

import serial
import sys
import time


from math import sin
from math import cos
from math import pi as PI

import RPi.GPIO as GPIO
 
import opc

import colorsys
import random



#from omma_geom import Vertex as vert
from omma_geom import Ommatid


# while True:
# 	for i in range(numLEDs):
# 		pixels = [ (0,0,0) ] * numLEDs
# 		pixels[i] = (255, 255, 255)
# 		client.put_pixels(pixels)
# 		time.sleep(0.01)


_LIST_PORTS_OK = True

#try:  # this is only in python 2.6, and fails on some 64 machines
#    import serial.tools.list_ports
#except ImportError:
#    _LIST_PORTS_OK = False



class SerialError(Exception):
    """ custom exception for handing serial port errors"""
    def __init__(self, port="???", string="unknown"):
         self.port = port #offending serial port
         self.string = string #descriptive string
    def __str__(self):
         return repr("SerialError on port %s: %s" % (self.port,self.string))


class SerialIO(object):
    """serial_io: Wrapper for pyserial module"""
    def __init__(self, port, baudrate, timeout=0.1, echo=False, xonxoff=False ):

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.echo = echo
        self.stdout = False
        self.ser = None
        self.status = 'OK'
        self.reopen()

    def reopen(self,timeout=0.1, ser_args = {}):
        """ open or reopen the given port, additional args in ser_args dict """
        self.timeout = timeout
        # first close it if it's already open

        # do we want stdout (for debug)?
        if self.port == 'stdout':
            self.stdout = True
            self.ser = sys.stdout
            self.status = "OK"
            return

        if self.ser is not None:
            try:
                self.ser.close()
                self.ser = None
            except serial.SerialException, v:
                print repr(v)
                self.status = 'error'

        if True:
            try:  # xonxoff=True hoses binary polling of Faulhabers!
                self.ser = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout, xonxoff=False,
                                         **ser_args)
            except serial.SerialException, v:
                self.status = 'error'
                print str(v)
            else:
                print str(self.ser)
        return self.status

    def really_flush_input(self):
        """ because ser.flushInput() doesn't seem to work :/"""
        if self.stdout:
            return
        #self.ser.flushInput()
        #time.sleep(0.01)
        readstr = ''
        incount = self.ser.inWaiting()
        #readstr += self.ser.read(incount)
        #incount = self.ser.inWaiting()
        while incount > 0:
            readstr += self.ser.read(incount)
            print '%d chars gobbled "%s"' % (incount,repr(readstr))
            time.sleep(0.01)
            incount = self.ser.inWaiting()

    def checkwrite(self, string, flush=True):
        """ write the given string to the serial device, raising errors """
        try:
            self.ser.write(string)
        except serial.serialutil.SerialTimeoutException as v:
            print str(v)
            raise SerialError(self.port,str(v))
        except serial.SerialException as v:
            print str(v)
            raise SerialError(self.port,str(v))
        if flush:
            if self.stdout:
                sys.stdout.flush()
            else:
                self.ser.flushOutput()

#    def read(self, nb):
#        return self.ser.read(nb)

    def checkread(self,n,where = ""):
        """returns string actually read, exception if error"""
        if self.stdout:
            return ""
        try:
            readstr = self.ser.read(n)
        except serial.SerialException as v:
            print str(v)
            raise SerialError(self.port,str(v))
        if len(readstr) != n:
            #print where + " err, got '%s'" % readstr
            raise SerialError(self.port,where + "truncated Response")
            return ""
        return (readstr)

    def readline_n(self,n):
        """Tries to read a line up to \n or \r or n chars."""
        incount = 0
        instr = ''
        looping = True
        while looping:
            try:
                char = self.ser.read(1)
            except serial.SerialException as v:
                raise SerialError(self.port, "readline read char error")
            instr += char
            incount += 1
            if incount >= n:
                return instr
            if char == '\n' or char == '\r':
                return instr

#     def saferead(self):
#         """returns string actually read, exception if error"""
#         if self.stdout:
#             return ""
# #        try:
#         readstr = self.ser.read()
# #        except serial.SerialException as v:
# #            print str(v)
# #            raise SerialError(self.port,str(v))
#         if len(readstr) != n:
#             print "err, got '%s'" % readstr
#             #raise SerialError(self.port,"No Response")
#             return ""
#         return (readstr)


def GetPortList(plist):
    """ return a list of available serial ports"""
    if len(plist) < 1:
        # if no list of portnames, make one
        plist = []
    OKports = []
    for port in plist:
        ser = None
        try:
            ser = serial.Serial(port, 9600)
        except serial.SerialException, v:
            pass
        else:
            OKports.append(ser.name)
        finally:
            if ser:
                ser.close()
    return OKports



def rangemap(val, thresh):
    """ map sensor range to pixel range"""
    if val < thresh:
        return 0
    val = val *15
    if val > 255:
        val = 255
    return val


class Ommahard(Ommatid):
    """ ommatid hardware, extends Ommatid geometry """
    def __init__(self):
        Ommatid.__init__(self)
        self.fc = opc.Client('localhost:7890')
        self.nfaces = 19
        self.ins = ''
        # lookup list of face names by index
        self.facemap = [ chr(i + ord('a'))  for i in range(self.nfaces)]
        self.boardmap = [ 'x'  for i in range(self.nfaces)]
        #print str(self.facemap)
        self.numLEDs = 80
        # array of pixels in *physical* order, (not logical)
        self.px = [ [0,0,0] ] * self.numLEDs
        # tricksy here: no physical board 18 (out of 20) so remap board 19 channels to
        # board 18
        self.set_HSVsphere(0)
        self.print_lats()
        self.init_cmap()

    def r2d(self,r):
        return 180*r/PI

    def print_lats(self):
         for qf in self.qfaces:
            print "board %d lat: %f long: %f " % (qf.i, self.r2d(qf.lat), self.r2d(qf.lng))
            if qf.i == 5 or False:
                for f in qf.f:
                    print "face %d lat: %f long: %f " % (f.i, self.r2d(f.lat), self.r2d(f.lng))
        
    def init_cmap(self):
        """ map pixel and sensor index to channel (logical) index."""


        self.cmap = []
        for qf in self.qfaces:
            qf.a = 'x'

        self.qfaces[0].a = 'A'

        for n, qf in enumerate(self.qfaces):
            if qf.a != 'x':
                print "MAP: i:%02d n%02d -- %s" % (qf.i, n, qf.a)
        
        #for f in 'abcdefghijklmnopqrs':
        #    for i in range(4):
        #        c = 4*self.face2n(f) + i
        #        print "mapping index %d to chan %d" % (i, c)

        i = 0
        # for qf in self.qfaces:
        #     # qf is quadface
        #     for c in qf.f:
        #         self.cmap.append(c.i)
        #         print "mapping index %d to chan %d" % (i, c.i)
        #         i = i + 1

                
    def set_HSVsphere(self,offset):
        """ color chan according to HSV, map longitude to hue,
        latitude to saturation. Subclass of ommatid """     
        # processing
        for c in self.chan:
            # hue -- floating [0-1]
            hh = (c.ph + PI)/(2.0*PI)
            # brightness
            bb = c.th/(PI) 
            hh = hh + offset
            if hh > 1.0:
                # wrap around
                hh -= 1.0

            # if bb > 50:
            #     c.c = color(hh,map(bb, 50, 100, 100,0 ),bb)
            # else:
            #     c.c = colorsys.hsv_to_rgb(hh,100,bb)

            c.c = colorsys.hsv_to_rgb(hh,1.0,bb)


        
    def swoop(self,speed=1.0,color=(1.0,1.0,1.0)):
        # swoop brightness up and down over the given number of seconds
        for up in range(0,255,4):
            
            self.px = [ [int(color[0]*up),
                         int(color[1]*up),
                         int(color[2]*up)] ]* self.numLEDs
            self.fc.put_pixels(self.px)
            time.sleep(speed/(2*255.0))
        for i in range(0,255,4):
            dn = 255 - i
            self.px = [ [int(color[0]*dn),
                         int(color[1]*dn),
                         int(color[2]*dn)] ]* self.numLEDs
            self.fc.put_pixels(self.px)
            time.sleep(speed/(2*255.0))
    
    def face2n(self, f):
        # get face index int from label f, faces indexed from 0
        return ord(f.lower()) - ord('a') 

    def n2face(self, i):
        # get face label f from index int from label f
       return self.facemap[i]

    def accum_string(self, the_str):   
        #print "parsing '%s'" % the_str
        retstr = ''
        try:
            if len(self.ins) == 0:
            # start new string, get remainder of string past start char
                self.ins = '<' +  the_str.split('<')[1]
            else:
                self.ins = self.ins + the_str
                
            #print "added '%s'" % self.ins
        except (ValueError, IndexError):
            print "parse error, so far " + str(self.ins)
            self.ins  = ""
            return ""

        if self.ins[-1] == '>':
            retstr = self.ins
            self.ins = ''
        return retstr
             
    def get_sensor_values(self, instr):
        # assumes values are already cooked and coming out in 
        try:
            # convert alpha address to integer face
            face = self.face2n(instr[1])
        except (ValueError, IndexError):
            return -1, [-1, 0, 0, 0]
        s = []
        # parse 4 hex ints out of return string
        for n in range(2, 9, 2):
            try:
                s.append(int(instr[n:n+2],16))
                #print "s is " + str(s)
            except (ValueError, IndexError):
                return -1, [-2, 0, 0, 0]
        return face, s

    def hsv2rgb(self,h,s,v):
        return tuple(i * 255 for i in colorsys.hsv_to_rgb(h/255.0,s/255.0,v/255.0))

def r2d(r):
    return 180*r/PI

def rdist(lat1, lng1, lat2, lng2):
    return(sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2)*cos(lng1 - lng2))

def pchanll(ch):
    print " pchan:%02d lat: %f lng %f" % (ch.i,ch.lat,ch.lng)
    for cn in ch.n:
        print "    pnabe:%02d lat: %f lng %f" % (cn.i,cn.lat,cn.lng)

def pchan(ch, pi):
    ch.pi = pi
    print " pchan:%02d pi: %02d " % (ch.i, ch.pi)
    for cn in ch.n:
        print "     pnabe:%02d pi %02d" % (cn.i, cn.pi)

if __name__ == '__main__':
    import kbhit
    import cPickle as pickle
    port_list = GetPortList([])
    print str(port_list)


    kb = kbhit.KBHit()

    
    #kb.set_normal_term()


    # to use Raspberry Pi board pin numbers
    GPIO.setmode(GPIO.BOARD)
 
    # set up the GPIO channels - one input and one output
    GPIO.setup(40, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    ser = SerialIO("/dev/ttyUSB0",250000)


    # if no pickle file, compute ommatid and pickle it
    if False:
        omma = Ommahard()
        pickle.dump(omma, open( "ommah.p", "wb" ) )
        print "saving pickle"
    # else load it from file
    else:
        print "loading from pickle"
        omma = pickle.load(open( "ommah.p", "rb" ) )
    

    # ok, this is confusing AF but here goes: pixel output string and
    # sensor input string are 1-D arrays in PHYSICAL order.  omma
    # channels and faces are 1D arrays in LOGICAL order (basically in
    # order of construction, which is kind of random. thus omma.chan
    # is a 1D array ofchannels in logical order, ch.i is index into
    # this (e.g. ch = omma.chan[3] then ch.i == 3)

    # to map between them, we use chan_map and chan_imap dicts.
    # PHYSICAL = log2phys[LOGICAL]
    # LOGICAL  = phys2log[PHYSICAL]
    # ch.i  == log2phys[ch.pi]
    # ch.pi == phys2log[ch.i]
    
    # index of face to debug, indexed from a=0
    dface = -1
    if len(sys.argv) > 1:
        dface = omma.face2n(str(sys.argv[1]))
    print " debug face %s (%d)" % (omma.n2face(dface), dface)

    numLEDs = 80

    # attach these to channels eventually
    s =  [0.0 for i in range(numLEDs)]  # current sensor readings
    sa = [0.0 for i in range(numLEDs)]  # average sensor readings
    pv = [0.0 for i in range(numLEDs)]  # pixel value, use for color mapping
    rm = [0.0 for i in range(numLEDs)]  # range map, current norm sensor val

    u = [0.0 for i in range(numLEDs)]  # wave equation
    up = [0.0 for i in range(numLEDs)]  # wave equation
    um = [0.0 for i in range(numLEDs)]  # wave equation

    red = [0.0 for i in range(numLEDs)]  # for mode 3 drawing
    whi = [0.0 for i in range(numLEDs)]  #

    fval = [0] * numLEDs # pixel values for faces, long term average

    tc = 0.9995 # time constant for running average
    
    # color output for pixels
    pixels = [ [0,0,0] ] * numLEDs

    clean = 0

    delay = 0.0010
    frame_delay = 0.00
    sensor_threshold = 8

    
    # mode change code: look at sum of top channels
    mode = 0
    new_mode = True
    mode_count = 0

    #omma.swoop()

    faces = 'abcdefghijklmnopqrs'
    phase_advance = 0.0
    lat_count = 0 # interactive count
    lng_count = 0 # interactive count
    board_count0 = 0
    board_count1 = 0
    chan_count =0
    board_lats = [[5],[0, 15, 16],[1,4,6,8,10,11],[2,3,12,14,17,19],[7,9,13]]

    # inverse map of logical board map to physical board map
    # board_map[i] = b where i is physical address and b is logical address
    # that is, omma.qfaces[b] has physical address (face address) i. 

    board_map = {i:-1 for i in range(20)}

    # top board
    board_map[5] = 0

    #top ring of three
    board_map[0] = 7
    board_map[15] = 1
    board_map[16] = 4

    #upper ring of six
    board_map[1]  = 8
    board_map[4]  = 9
    board_map[6]  = 2
    board_map[8]  = 3
    board_map[10] = 5
    board_map[11] = 6

    #lower ring of six
    board_map[2]  = 17
    board_map[3]  = 15
    board_map[12] = 14
    board_map[14] = 12
    board_map[17] = 11
    board_map[19] = 18

    # bottom ring of three
    board_map[7]  = 16
    board_map[9]  = 13
    board_map[13] = 10
    
    # bottom board 18 is not used

    # invert board map such that ib[i] = b 
    # ib_map = {v: k for k, v in board_map.items()}

    # for i in range(19):
    #     b = board_map[i]
    #     qf = omma.qfaces[b]
    #     qf.bm = b # board map
    #     print "lb %2d i: %2d qf.i: %2d lat: %f lng %f " % (qf.bm, i, qf.i, r2d(qf.lat), r2d(qf.lng))

    #print repr(ib_map)
    #print repr(board_map)
    
    for blist in board_lats:
        for b in blist:
            qf = omma.qfaces[b]
            for n, ch in enumerate(qf.f):
                ch.bm = b # board map, logical index
                ch.l = n # physical position on board (0, 1, 2, 3: 0 is center)
            
    # for i in chan_map:
    #     ch = omma.chan[i]
       #print "chan: %d bm: %d pi %d n%d lat: %f long: %f"  % (ch.i, ch.bm, ch.pi, ch.l, r2d(ch.lat), r2d(ch.lng))
        
    chan_map = {i:-1 for i in range(20)}
    # lists of logical channel indices of channels at same latitude, in increasing latitude.
    # each list ordered by min-to-max longitide (at same latitude)
    chan_lats = [[21],
                 [22, 20, 23],
                 [67, 2, 3, 63, 60, 66],
                 [65, 1, 61],
                 [32, 19, 6, 26,43, 46],
                 [64, 0, 62],
                 [33, 17, 5, 25, 41, 45],
                 [47,35,16,4,27,42],
                 [34, 18, 7, 24, 40, 44]]
    
    # lats[0], top of sphere
    chan_map[21] = 0 
    pchanll(omma.chan[21])
    
    # lats[1], adjacent to top board
    chan_map[22] = 3 
    pchan(omma.chan[22], 3)
    chan_map[20] = 1 
    pchan(omma.chan[20], 1)
    chan_map[23] = 2
    pchan(omma.chan[23], 2)
    
    # lats[2]
    chan_map[67] = 29 
    pchan(omma.chan[67], 29)
    chan_map[2]  = 6
    pchan(omma.chan[2], 6)
    chan_map[3]  = 5
    pchan(omma.chan[3], 5)
    chan_map[63] = 18
    pchan(omma.chan[63], 18)
    chan_map[60] = 17
    pchan(omma.chan[60], 17)
    chan_map[66] = 30 
    pchan(omma.chan[66], 30)
    
     # lats[3]
    chan_map[65]  = 28
    pchan(omma.chan[65], 28)
    chan_map[1]   = 4
    pchan(omma.chan[1], 4)
    chan_map[61]  = 16 
    pchan(omma.chan[61], 16)

    # lats[4]
    chan_map[32] = 33
    pchan(omma.chan[32], 33)
    chan_map[19] = 37
    pchan(omma.chan[19], 37)
    chan_map[6]  = 9
    pchan(omma.chan[6], 9)
    chan_map[26] = 13
    pchan(omma.chan[26], 13)
    chan_map[43] = 21
    pchan(omma.chan[43], 21)
    chan_map[46] = 25
    pchan(omma.chan[46], 25)

    for i in chan_lats[5]:
        pchan(omma.chan[i],0)

    for i in chan_lats[6]:
        pchan(omma.chan[i],0)
        
    exit(0)

    
    # lats[5]
    chan_map[64]  = 31
    chan_map[0]   = 7
    chan_map[62]  = 19 

    # lats[6]
    chan_map[33] = 34
    chan_map[17] = 39
    chan_map[5]  = 10
    chan_map[25] = 15
    chan_map[41] = 22
    chan_map[45] = 27

    # lats[7]
    chan_map[47] = 32
    chan_map[35] = 36
    chan_map[16] = 8
    chan_map[4]  = 12
    chan_map[27] = 20
    chan_map[42] = 24

    # lats[8], bottom row on top half, just above equator
    chan_map[34] = 35
    chan_map[18] = 38
    chan_map[7]  = 11
    chan_map[24] = 14
    chan_map[40] = 23
    chan_map[44] = 26

    # bottom half: reverse longitude
    #lats[9]
    chan_lats.append([59,76, 8, 12, 70, 50])
    chan_map[59] = 71 
    chan_map[76] = 62
    chan_map[8]  = 59
    chan_map[12] = 50
    chan_map[70] = 47
    chan_map[50] = 74

    chan_lats.append([48,56,79,10,15,71])
    #lats[10]
    chan_map[48] = 68
    chan_map[56] = 60
    chan_map[79] = 56
    chan_map[10] = 48
    chan_map[15] = 44
    chan_map[71] = 72

    chan_lats.append([57,77, 9, 13, 69, 49])
    # lats[11]
    chan_map[57] = 70
    chan_map[77] = 63
    chan_map[9]  = 58
    chan_map[13] = 51
    chan_map[69] = 46
    chan_map[49] = 75

    chan_lats.append([52, 30, 36])
    # lats[12]
    chan_map[52] = 67
    chan_map[30] = 55
    chan_map[36] = 43

    chan_lats.append([58, 78, 11, 14, 68, 51])
    # lats[13]
    chan_map[58] = 69
    chan_map[78] = 61
    chan_map[11] = 57
    chan_map[14] = 49
    chan_map[68] = 45
    chan_map[51] = 73

    chan_lats.append([29, 53, 37])
    # lats[14]
    chan_map[29] = 64
    chan_map[53] = 52
    chan_map[37] = 40

    chan_lats.append([55,31, 28, 38, 39, 54])
    # lats[15]
    chan_map[55] = 65
    chan_map[31] = 66
    chan_map[28] = 53
    chan_map[38] = 54
    chan_map[39] = 41
    chan_map[54] = 42

    # logical board 18 (max latitude on bottom) is not physically
    # present, map to dummy physical channel -1
    # lats[16]
    chan_map[72] = -1
    chan_map[73] = -1
    chan_map[74] = -1
    chan_map[75] = -1

    #for lat in chan_lats:
    #    print "lat" + repr(lat)
    

    
    log2phys = []
    phys2log = []
    for ch in omma.chan:
        ch.pi = chan_map[ch.i]
        log2phys.append(ch.pi)
        #print "chan: %d pi %d lat: %f long: %f"  % (ch.i, ch.pi, r2d(ch.lat), r2d(ch.lng))
    

    #exit(0)
        
    while(True):
        # one method: subtract mean of "off" channels
        # these are good

        omma.set_HSVsphere(phase_advance)        
        phase_advance += 0.05
        g = 0 # global index
        for f in faces:

            #ser.ser.write("<%cc>" % (f));
            ser.ser.write("<%cs>" % (f));
            time.sleep(delay)
            nready = ser.ser.inWaiting()
            while nready > 0:
                instring = ser.ser.read(11).strip()
                res  = ""
                if len(instring) != 11:
                    print "error (face %s?)" % f
                    print "got: " + str(instring)
                else:
                    sys.stdout.flush()
                res = omma.accum_string(instring)
                if len(res) > 0:
                    if('x' == f):
                        print "cookde: " + res
                        sys.stdout.flush()
                    chan, svals = omma.get_sensor_values(res)
                    #figure out start index for this face
                    starti = 4*(chan - omma.face2n('a'))
                    if(chan == dface and False):
                        print "parsde: si=%d %s" % (starti, str(svals))
                        sys.stdout.flush()
                    for i in range(4):
                        c = starti + i
                        # accumulate long-term average
                        if clean < 30: # initialize at startup
                            sa[c] = svals[i]
                        else:
                            sa[c] = tc*float(sa[c]) + (1 - tc)*float(svals[i])

                        # subtract long-term average from instant value
                        fval[c] = svals[i] - sa[c]
                        # rangemap for this pixel, instantaneous sense value
                        rm[c] = rangemap(fval[c],sensor_threshold)
                        if rm[c] > 0:
                            pv[c] = max(pv[c], rm[c])
                            if pv[c] > 255:
                                pv[c] = 255
                        else:
                            pv[c] = 0.92*pv[c]

                nready = ser.ser.inWaiting()

        ########## OK got sensor value in pv[c], mapped to pixels[c]
        ## use ommamap to get channel i for index c

        # for c in omma.chan:
        #     #print "setting chan %d to color %s" % (c.i, str(c.c)) 
        #     pixels[c.i] = [255* n for n in c.c]

        # for qf in omma.qfaces:
        #     # sum rangemap for this face
        #     fv = 0
        #     for n, ch in enumerate(qf.f):
        #         c = ch.i
        #         #print "ch %d %s" % (ch.i, n)
        #         #pixels[c] = ch.c
        #         fv += float(rm[c])
        #         fv = int(fv/3)
        #         if qf.a == 'A':
        #             pixels[4*qf.i + 0] = (255,255,255)
        #             pixels[4*qf.i + 1] = (255,255,255)
        #             pixels[4*qf.i + 2] = (255,255,255)
        #             pixels[4*qf.i + 3] = (255,255,255)
        #         else:
        #             pixels[4*qf.i + 0] = (0,0,0)
        #             pixels[4*qf.i + 1] = (0,0,0)
        #             pixels[4*qf.i + 2] = (0,0,0)
        #             pixels[4*qf.i + 3] = (0,0,0)

        redpix = -1
        if mode == 0: # light up and report touched face
            print_hit = False
            if kb.kbhit():
                c = kb.getch()
                if c == 'n':
                    lat_count += 1
                    if lat_count >= len(chan_lats):
                        lat_count = 0
                    print "lat count " + str(lat_count)
                    print "lats" + repr(chan_lats[lat_count])
                    lng_count = 0
                    print_hit = True
                elif c == 'm':
                    print_hit = True
                    lng_count += 1
                    if lng_count >= len(chan_lats[lat_count]):
                        lng_count = 0
                    print "lng count " + str(lng_count)
                    ch = omma.chan[chan_lats[lat_count][lng_count]]
                    print "chan:%02d pi: %d face:%02d" % (ch.i,ch.pi,qf.i)
                    print "lat: %f long: %f" % (ch.lat, ch.lng)
                    for cn in ch.n:
                        print "  c-nabe:%02d pi: %d lat: %f lng %f" % (cn.i,cn.pi,cn.lat,cn.lng)
                        #print "  rdist %f" % rdist(ch.lat, ch.lng, cn.lat,cn.lng)
                        print "  edist %f" % ch.nv.dist2(cn.nv)

                elif c == 'z':
                    print_hit = True
                    board_count0 += 1
                    if board_count0 >= len(board_lats):
                        board_count0 = 0
                    print "b0 count " + str(board_count0)
                    board_count1 = 0

                elif c == 'x':
                    print_hit = True
                    board_count1 += 1
                    if board_count1 >= len(board_lats[board_count0]):
                        board_count1 = 0
                    b = board_lats[board_count0][board_count1]    
                    qf = omma.qfaces[b]
                    print "b1: %2d i %2d b: %2d lng %f " % (board_count1, qf.i, b, r2d(qf.lng)) 

                elif c == 'v':
                    chan_count += 1
                    if chan_count >= len(omma.chan):
                        chan_count = 0
                    print "chan_count :" + str(chan_count)
                    ch = omma.chan[chan_count]
                    print " vchan:%02d pi: %02d lat: %f lng %f" % (ch.i,ch.pi,cn.lat,cn.lng)
                    for cn in ch.n:
                        if cn.pi >=0:
                            print " vnabe:%02d pi: %02d lat: %f lng %f" % (cn.i,cn.pi,cn.lat,cn.lng)
                            print "  edist %f" % ch.nv.dist2(cn.nv)                    
            for ch in omma.chan:
                pixels[ch.pi] = (0,0,0)
                
            for i in chan_lats[lat_count]:
                ch = omma.chan[i]                
                pixels[ch.pi] = (0,0,128)

            i = chan_lats[lat_count][lng_count]
            ch = omma.chan[i]
            pixels[ch.pi] = (255, 0 ,0)
            # for cn in ch.n:
            #     if cn.pi >=0:
            #         #print "nabe:%02d pi: %02d " % (cn.i, cn.pi)
            #         # if physical channel exists
            #         pixels[cn.pi] = (0,255,0)
                
            ch = omma.chan[chan_count]
            if ch.pi >= 0:
                pixels[ch.pi] = (0,255,0)
                for cn in ch.n:
                    if cn.pi >=0:
                        #print "  c-nabe:%02d pi: %02d " % (cn.i, cn.pi)
                        # if physical channel exists
                        pixels[cn.pi] = (255,0,255)

            for ch in omma.chan:
                fv = float(rm[ch.pi])
                if fv > 99:
                    print "touch:%02d pi: %d face:%02d" % (ch.i,ch.pi,qf.i)
                    print "lat: %f long: %f" % (ch.lat, ch.lng)
                    pixels[ch.pi] = (255,255,255)
                    for cn in ch.n:
                        print "  t-nabe:%02d pi: %d lat: %f lng %f" % (cn.i,cn.pi,cn.lat,cn.lng)
                        if cn.pi >=0:
                            pixels[cn.pi] = (0,255,0)

        elif mode == 1 and False: # rgbW face triangles
            for qf in omma.qfaces:
                # sum rangemap for this face
                fv = 0
                for n, ch in enumerate(qf.f):
                    c = ch.i
                    #print "ch %d %s" % (ch.i, n)
                    #pixels[c] = ch.c
                    fv += float(rm[c])
                fv = int(fv/3)
                if fv < 10:
                    fv = 0
                pixels[4*qf.i + 0] = (0, fv, 0)
                pixels[4*qf.i + 1] = (fv, 0,  0)
                pixels[4*qf.i + 2] = (0, 0, fv)
                pixels[4*qf.i + 3] = (0, 0, fv)

                # for n, ch in enumerate(qf.f):
                #     c = ch.i
                #     pixels[c] = (0, fv, 0)
                         
       #  if(chan == dface):
       #      #print "%s rm:%d pv:%d" % (f, rm[c], pv[c]) 
       #      pass

       #  #print "r %2d %2d %2d %2d" % (s[0], s[1], s[2], s[3])
       #  #print "a %2d %2d %2d %2d" % (sa[0], sa[1], sa[2], sa[3])
       #  if(chan == dface or 0):
       #      print "%s %2d %2d %2d %2d" % (f, fval[starti], 
       #                                    fval[starti + 1], 
       #                                    fval[starti + 2], 
       #                                    fval[starti + 3])
       #  #print str(pixels)
       # #g = (f-1)*4 + chan
       #  #s[g] = val
       #  #print str(cook_sensor_values(instr))

        mode_count += 1
        if mode_count > 255:
            mode_count = 255

        clean += 1
        if clean > 100:
            clean = 100;

        #print str(GPIO.input(40))
        if GPIO.input(40) == 0:
            clean = 8
            omma.swoop(0.2)
            # try a clean shutdown
            from subprocess import call
            result = call(['sudo', '/sbin/shutdown', '--poweroff',  'now'])
            print result
            
        # detect palm-on-top bu sum 
        mode_detect = sum(rm[0:40])
        #print "mode: %d %d" % (mode_detect, mode_count)
        if mode_detect > 4000 and new_mode:
            print str(mode_detect)
            mode += 1
            # reset accumulated values

            new_mode = False
            #if mode > 2:
            if mode > 0: # only one mode: test
                mode = 0
            print str(mode_detect)
            if mode == 0:
                omma.swoop(1.0,(0.5, 0.5, 0.5))
                print "mode 0: white draw"
            elif mode == 1: #"white random red"
                omma.swoop(1.0,(1, 0, 0))
                print "mode 1: blue"
            elif mode == 2:
                omma.swoop(1.0,(0, 1, 1))
                print "mode 2 mag: "

            pv = [0.0 for i in range(numLEDs)]  
            rm = [0.0 for i in range(numLEDs)]  
            red = [0.0 for i in range(numLEDs)]  
            svals = [0 for i in range(numLEDs)]  
            mode_count = 0
        elif mode_detect < 100:
            new_mode = True
        sys.stdout.flush()
        if mode == 2:
            for i in range(1, 78):
                up[i] = 0.95*( -um[i] + 2.0*u[i] + \
                    0.05*(u[i-1] - 2*u[i] + u[i+1])) 
          # insert boundary conditions:
            #up[0] = 0
            #up[79] = 0
            for i in range(len(up)):
                um[i] = u[i]
                u[i] = up[i]

        if mode_count > 2:
            omma.fc.put_pixels(pixels)
        time.sleep(frame_delay)


