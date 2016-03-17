#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
 ----------------------------------------------------------------------
  serialIO: wrappers for pyserial module for exception handling, etc.
  All serial access should be through these wrappers.
 ----------------------------------------------------------------------

"""

import serial
import sys
import time

import RPi.GPIO as GPIO
 
import opc

import colorsys
import random



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


class Ommatid(object):
    def __init__(self):
        self.fc = opc.Client('localhost:7890')
        self.nfaces = 19
        self.ins = ''
        # lookup list of face names by index
        self.facemap = [ chr(i + ord('a'))  for i in range(self.nfaces)]
        #print str(self.facemap)
        self.numLEDs = 80
        # array of pixels
        self.px = [ [0,0,0] ] * self.numLEDs


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


if __name__ == '__main__':
    port_list = GetPortList([])
    print str(port_list)


    # to use Raspberry Pi board pin numbers
    GPIO.setmode(GPIO.BOARD)
 
    # set up the GPIO channels - one input and one output
    GPIO.setup(40, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    ser = SerialIO("/dev/ttyUSB0",250000)

    omma = Ommatid()

    # index of face to debug, indexed from a=0
    dface = -1
    if len(sys.argv) > 1:
        dface = omma.face2n(str(sys.argv[1]))
    print " debug face %s (%d)" % (omma.n2face(dface), dface)

    numLEDs = 80


    s =  [0.0 for i in range(numLEDs)]  # current sensor readings
    sa = [0.0 for i in range(numLEDs)]  # average sensor readings
    pv = [0.0 for i in range(numLEDs)]  # pixel value, use for color mapping
    rm = [0.0 for i in range(numLEDs)]  # range map, current norm sensor val

    u = [0.0 for i in range(numLEDs)]  # wave equation
    up = [0.0 for i in range(numLEDs)]  # wave equation
    um = [0.0 for i in range(numLEDs)]  # wave equation

    red = [0.0 for i in range(numLEDs)]  # for mode 3 drawing
    whi = [0.0 for i in range(numLEDs)]  #

    fval = [0] * numLEDs # pixel values for faces

    tc = 0.9995 # time constant for running average


    
    # color output for pixels
    pixels = [ [0,0,0] ] * numLEDs

    # map sensor array to pixels
    lmap = range(numLEDs)
    #lmap[2] = 1
    #lmap[1] = 0


    ### sensors indexed


    ### LEDs indexed same as sensors

          #   3
          #   0 
          # 2   1

# fundamental indexing: ADC0-ADC3, top (LED side) view, "|" is RS485
       
          #   |1
          #   3
          #   0 
          # 2   1


    #atc = 0.95 # averaging time constant for sensor average

    clean = 0


    # mode change code: look at sum of top channels
    mode = 1
    new_mode = True
    mode_count = 0
    omma.swoop()
    faces = 'abcdefghijklmnopqrs'
    #bottom half
    #faces = 'klmnpqs'
    #faces = 'abcdefghijklmno'
    #faces = 'o'



    while(True):
        # one method: subtract mean of "off" channels
        # these are good
        delay = 0.0010
        frame_delay = 0.00

        #delay = 0.005
        #frame_delay = 0.000
        # addresses of face boards. 
        #faces = 'klmnopqrs
        #faces = 'abcdefghij'
        g = 0 # global index
        #for f in faces:
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
                        if clean < 10: # initialize at startup
                            sa[c] = svals[i]
                        else:
                            sa[c] = tc*float(sa[c]) + (1 - tc)*float(svals[i])

                        # subtract long-term average from instant value
                        fval[c] = svals[i] - sa[c]
                        # rangemap for this pixel, instantaneous sense value
                        rm[c] = rangemap(fval[c],5)
                        if rm[c] > 0:
                            pv[c] = max(pv[c], rm[c])
                            if pv[c] > 255:
                                pv[c] = 255
                        else:
                            pv[c] = 0.92*pv[c]
                        # now do mode-dependent coloring here
                        if mode_count > 3: # hack to zero after swoop
                            if mode == 0: # white draw
                                pixels[c] = omma.hsv2rgb(0,0,max(pv[c], rm[c]))
                            elif mode == 1: # red draw
                                red[c] = max(red[c], rm[c])
                                if red[c] > 10:
                                    pixels[c] = (red[c], 0, 0)
                                else:
                                    if random.random() < 0.05:
                                        pv[c] = 200
                                    pixels[c] = omma.hsv2rgb(0,0,pv[c]) 
                            elif mode == 2: # wave equation
                                # 1d wave equation
                                if rm[c] > 10:
                                    u[c] = rm[c]
                                #if up[c] > 0:
                                pixels[c] = omma.hsv2rgb(int(0.3*(up[c]) + 128), 128, 100)
                        else:
                            pv[c] = 0.0
                            rm[c] = 0.0
                            pixels[c] = (0, 0, 0)
                        #pixels[c + 40] = omma.hsv2rgb(pv[c],255,rm[c])

                    if(chan == dface):
                        #print "%s rm:%d pv:%d" % (f, rm[c], pv[c]) 
                        pass
                    
                    #print "r %2d %2d %2d %2d" % (s[0], s[1], s[2], s[3])
                    #print "a %2d %2d %2d %2d" % (sa[0], sa[1], sa[2], sa[3])
                    if(chan == dface or 0):
                        print "%s %2d %2d %2d %2d" % (f, fval[starti], 
                                                      fval[starti + 1], 
                                                      fval[starti + 2], 
                                                      fval[starti + 3])
                    #print str(pixels)
                   #g = (f-1)*4 + chan
                    #s[g] = val
                    #print str(cook_sensor_values(instr))
                    nready = ser.ser.inWaiting()
                    sys.stdout.flush()


        mode_count += 1
        if mode_count > 255:
            mode_count = 255

        clean += 1
        if clean > 10:
            clean = 10;

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
            if mode > 2:
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


