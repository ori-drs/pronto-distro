#!/usr/bin/python

import os
import sys
import binascii
import types
from numpy import *
import re
import getopt
import copy
import scipy.io.matlab.mio
import lcm

from lcm import EventLog
from bot_core.pose_t import pose_t
from bot_core.image_t import image_t

def shouldAddImage(pose,poseList,thresh,globalCheck):
    if thresh>0 and len(poseList)>0:
        if globalCheck:
            for p in poseList[::-1]:
                d = linalg.norm(array(p.pos)-array(pose.pos))
                if d<thresh:
            	   return False
        elif linalg.norm(array(poseList[-1].pos)-array(pose.pos))<thresh:
            return False
    return True

def deleteStatusMsg(statMsg):
    if statMsg:
        sys.stderr.write("\r")
        sys.stderr.write(" " * (len(statMsg)))
        sys.stderr.write("\r")
    return ""

longOpts = ["help", "channels", "ignore"]
def usage():
    pname, sname = os.path.split(sys.argv[0])
    sys.stderr.write("usage: % s %s < filename > \n" % (sname, str(longOpts)))
    print """
    -h --help                 print this message
    -c --channels=chan        Parse channels that match Python regex [chan] defaults to ["IMAGE_.*"]
    -d --dist=d               Enforce that images taken [d] from previous
    -D --global_dist=d        Enforce minimum distance between ANY images is [d]
    -v                        Verbose
    """


    sys.exit()


try:
    opts, args = getopt.gnu_getopt(sys.argv[1:], "hvc:d:D:", longOpts)
except getopt.GetoptError, err:
    # print help information and exit:
    print str(err) # will print something like "option -a not recognized"
    usage()
if len(args) != 1:
    usage()
#default options
fname = args[0]
outFname = fname
outFname = outFname.replace(".", "_")
outFname = outFname.replace("-", "_")
outFname = outFname + ".mat"
verbose = False
channels = "IMAGE_.*"
dist_thresh =-1
globalCheck = False
for o, a in opts:
    if o == "-v":
        verbose = True
    elif o in ("-h", "--help"):
        usage()
    elif o in ("-c", "--channels="):
        channels = a
    elif o in ("-d", "--dist="):
        dist_thresh = float(a)
    elif o in ("-D", "--global_dist="):
        dist_thresh = float(a)
        globalCheck = True
    else:
        print "unhandled option"
        usage()
        

fullPathName = os.path.abspath(outFname)
dirname = os.path.dirname(fullPathName)
outBaseName = os.path.basename(outFname).split(".")[0] + "_images"
fullBaseName = dirname + "/" + outBaseName

imageDir = fullBaseName

channels = re.compile(channels)
log = EventLog(fname, "r")

sys.stderr.write("Enforcing a distance of %f between images\n" % dist_thresh)
sys.stderr.write("opened % s, outputing to % s\n" % (fname, imageDir))

lc = lcm.LCM()

msgCount = 0
statusMsg = ""
startTime = 0
imageChannels = []
lastPose =pose_t()
lastPose.utime =-1;

imageFnames = {}
imagePoses = {}
imageMeta = {}
imagesWritten =0
for e in log:
    if msgCount == 0: 
        startTime = e.timestamp
        
    if e.channel=="POSE":
        lastPose = pose_t.decode(e.data);
    elif lastPose.utime<0:
        continue       
 
    msgCount = msgCount + 1
    if (msgCount % 5000) == 0:
        statusMsg = deleteStatusMsg(statusMsg)
        statusMsg = "read % d messages, % d %% done, %d images written" % (msgCount, log.tell() / float(log.size())*100,imagesWritten)
        sys.stderr.write(statusMsg)
        sys.stderr.flush()

    #if (msgCount>=100000):
	#break

    isImage = channels.match(e.channel)
    if not isImage:
        continue
    elif not (e.channel in imageChannels):
        #first image on this channel... do some setup
        imageChannels.append(e.channel)
        imageFnames[e.channel] = []
        imagePoses[e.channel] = []
        imageMeta[e.channel] = []
        if not os.path.isdir("%s" %(imageDir)):
            os.mkdir("%s" %(imageDir))
        if not os.path.isdir("%s/%s" %(imageDir,e.channel)):
            os.mkdir("%s/%s" %(imageDir,e.channel))
    if not shouldAddImage(lastPose,imagePoses[e.channel],dist_thresh,globalCheck):
        continue
    #TODO: filter on poses...    
    im = image_t.decode(e.data)
    imFileName = "%s/%s/%s_%07d.jpg" % (imageDir,e.channel, e.channel, len(imagePoses[e.channel]))
    imageFnames[e.channel].append(imFileName)
    imagePoses[e.channel].append(copy.deepcopy(lastPose))
    imMetaData = "%d %f %f %f %f %f %f %f %f" %(im.utime,lastPose.pos[0],lastPose.pos[1],lastPose.pos[2], \
                                                     lastPose.orientation[0],lastPose.orientation[1],lastPose.orientation[2],lastPose.orientation[3], \
                                                    (e.timestamp - startTime) / 1e6)
#    if e.channel=="IMAGE_LEFT":
#        p = copy.deepcopy(lastPose)
#        p.pos=(p.pos[0],p.pos[1],0)
#        lc.publish("POSE",p.encode())
#    print e.channel, imMetaData
    imageMeta[e.channel].append(imMetaData);
    imFile = open(imFileName, "wb")
    imFile.write(im.data)
    imFile.close()
    imagesWritten = imagesWritten+1;
    
        
    

deleteStatusMsg(statusMsg)            
            
sys.stderr.write("loaded all %d messages, saving to % s\n" % (msgCount, outFname))

        
mfile = open(dirname + "/" + outBaseName + ".m", "w")
loadFunc = """function [imFnames,imMeta]=%s()
imFnames = struct();
imMeta = struct();
""" % (outBaseName)

for k, v in imageFnames.items():
    loadFunc += "imFnames.%s ={'%s'}; \n" % (k, "',\n'".join(v))
for k, v in imageMeta.items():
    loadFunc += "imMeta.%s =[%s]; \n" % (k, ";\n".join(v))
    
    
mfile.write(loadFunc);
mfile.close()

