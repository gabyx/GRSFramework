#!/usr/bin/env python 
import os, subprocess;
from optparse import OptionParser



parser = OptionParser()
parser.add_option("-i", "--input", dest="input", help="Input file  (.tiff or something similar)", metavar="FILE")
parser.add_option("-o", "--output", dest="output", help="Output file  (.tiff or something similar)", metavar="OUTPUT")
parser.add_option("-t", "--text", dest="texts", action="append", help="Text to annotate, split newline with ';'", metavar="TEXT")
parser.add_option("-a", "--add", dest="add", default="", help="additional 'convert' arguments", metavar="JPG/TIF")                  

(opts, args) = parser.parse_args()

formatQuality = 100;

if not opts.input:
    raise NameError("No input file")

if not opts.output:
    (head,ext) = os.path.splitext(opts.input);
    
    [f,number] = head.rsplit("_",1);
    opts.output = f+"_annot"+"_"+number;
    if opts.format:
        opts.output+="."+opts.format;
    else:
        opts.output+=ext;
    
if not opts.texts:
    raise NameError("No annotation texts")
    
texts = [];
for t in opts.texts:
    tt = t.split(";")
    for ttt in tt:
        texts.append(ttt);

print("Texts: " + str(texts))

fontSize = 25;
lineWidth= 22;
approxCharSize = 14;

marginWidth = 10;
marginHeight = 10;

minBox = (10,10);
maxBox = (200,50);

# offset from text to text

maxCharacters = 0;
for t in texts:
    maxCharacters = max(maxCharacters,len(t));

boxSize= (maxCharacters * approxCharSize + 2*marginWidth, len(texts)*lineWidth + 2*marginHeight )
maxBox = [sum(x) for x in zip(minBox,boxSize)];

## Execute annotation command
command = ["convert"] + [os.path.abspath(opts.input)] + \
["-fill"] + ['rgba(0,0,50%,0.5)'] + ["-stroke"] + [r'rgba(0,0,100%,1)'] + ["-draw"] + ['rectangle %i,%i %i,%i' % (minBox[0],minBox[1], maxBox[0], maxBox[1]) ];

yCoord = minBox[1] + marginHeight + lineWidth;
for t in texts:
    command += ["-fill"] + ["gray(100%)"] + ["-stroke"] + ["gray(100%)"] +["-family"] + ["Latin Modern Mono Slanted"]   \
                + ["-pointsize"] + ["%i" % fontSize] + ["-draw"] +["text %i,%i \'%s\'" % (minBox[0]+marginWidth,yCoord,t)];
    yCoord+= lineWidth;

command += ["-quality"] + ["%i" % formatQuality] 

if opts.add:
  command += [opts.add]

command += [os.path.abspath(opts.output)] ;
print(command)
print(r" ".join(command))


subprocess.call(command);