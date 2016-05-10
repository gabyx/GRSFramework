#!/usr/bin/env python3 

import sys,os, subprocess, ctypes
import graphviz as gv

if sys.version_info[0] != 3:
    print("This script is only python3 compatible!")
    exit(1)

from argparse import ArgumentParser

from attrdict import AttrMap
import xml.etree.ElementTree as ET

import commentjson
import jsonpickle
jsonpickle.load_backend('commentjson', 'dumps', 'loads', ValueError)

def jsonParse(s):
  return jsonpickle.decode(s) 
        
def jsonLoad(filePath):
  f = open(filePath)
  return jsonParse(f.read())

def insertLinebreak(string, every=10):
    lines = []
    for i in range(0, len(string), every):
        # quote every string into 'text'
        lines.append(string[i:i+every])
    
        print(repr(lines))
    
    if lines:
      lines[0] = "'" + lines[0];
      lines[-1] = lines[-1]+ "'";
        
    return r'\n'.join(lines)

def quote(s):
    
    # first replace all ampersands /if there are any
    s = s.replace("&","&amp;")
    
    rep = { '"'  : "&quot;",
            "'"  : "&apos;",
            "<"  : "&lt;",
            ">"  : "&gt;",
        }
    for key, r in rep.items():  
        s = s.replace( key, r )
    
    return s.replace(r"\n", '<BR ALIGN="LEFT"/>')



def addTools(graph, tools, styles, general):
    l = []
    
    for tool in tools.values():
#        attributesText = []
#        for attName,value in tool.items():
#            if attName not in [ "type", "id","maxIns","maxOuts"] :
#                attributesText += [(attName+ ": " + quote(value)]
        
#       label += "| {{" + "|".join(["in %i" % i for i in range(0,tool.maxIns)]) + "} | {" +  "|".join(["out %i" % i for i in range(0,tool.maxOuts)]) + "}}"
        style = styles["nodes"].copy()

        attributesText = "\n".join([u"""<TR><TD ALIGN="RIGHT">%s :</TD> <TD ALIGN="LEFT">%s<BR ALIGN="LEFT"/></TD></TR> """ 
            % (attName,quote(insertLinebreak(value,int(general["linebreaks"])))) 
            for attName,value in tool.items() if attName not in [ "type", "id","maxIns","maxOuts"]  ])
        
        if not attributesText:
            attributesText =""
        else:
            attributesText= u"""
                              <TR>
                               <TD COLSPAN="2">
                                    <TABLE BORDER="0">
                                    <TR>
                                        <TD COLSPAN="2">Attributes</TD>
                                    </TR>
                                    %s
                                    </TABLE>
                                 </TD>
                              </TR> 
                              """ % attributesText

        
        title = tool.type + " [%s]" % tool.id
        
        

        if tool.maxOuts:
            outsHTML = "\n".join([ u"""<TR><TD ALIGN="RIGHT" VALIGN="MIDDLE" PORT="o%i" >%i \u2022</TD></TR>""" % (i,i) for i in range(0,tool.maxOuts) ])
        else:
            outsHTML = """<TR><TD></TD></TR>"""
         
        if tool.maxIns:                
            insHTML = "\n".join([ u"""<TR><TD ALIGN="LEFT" VALIGN="MIDDLE" PORT="i%i">\u2022 %i</TD></TR>""" % (i,i) for i in range(0,tool.maxIns) ])  
        else:
            insHTML = """<TR><TD></TD></TR>"""
        
       
        
        if "groupId" in tool and tool.groupId == "Frame":
            if "toolNodes-Frame" in styles:
                style.update(styles["toolNodes-Frame"])
        else:
            if "toolNodes-Body" in styles:
                style.update(styles["toolNodes-Body"])             
        
        label = u"""<<TABLE BORDER="2" CELLBORDER="0" CELLSPACING="0" CELLPADDING="0" BGCOLOR="{fillcolor}"> 
                      <TR VALIGN="MIDDLE">
                        <TD COLSPAN="2" CELLPADDING="4" VALIGN="MIDDLE" FIXEDSIZE="FALSE" WIDTH="{width}" HEIGHT="{height}"><FONT POINT-SIZE="12"><B>{title}</B></FONT></TD>
                      </TR>
                      <HR/>
                        {attributesText}
                      <TR>
                        <TD ALIGN="LEFT"  CELLPADDING="2"><B>Inputs</B></TD>
                        <TD ALIGN="RIGHT" CELLPADDING="2"><B>Outputs</B></TD>
                      </TR>
                      <TR>
                        <TD>
                            <TABLE BORDER="0" CELLPADDING="0">{insHTML}</TABLE>
                        </TD>
                        <TD>
                            <TABLE BORDER="0" CELLPADDING="0">{outsHTML}</TABLE>
                         </TD>
                      </TR> 
                   </TABLE>>""".format( fillcolor = style["fillcolor"], 
                                              title=title, 
                                              attributesText=attributesText, 
                                              insHTML=insHTML, 
                                              outsHTML=outsHTML,
                                              width=general["nodesWidth"],
                                              height=general["nodesHeight"]
                                            
                                      )
        
        
        style.update({"label": label})        
        
        l.append( (tool.id , style ) )
    
    
    addNodes( graph, l );
    
    
def addConnections(graph, getters, writters, styles):
        
        l=[]
        for get in getters: 
            style = {
                        "tooltip" : "in: %s -get-> out: %s" % (get.fromSocket, get.outSocket) }
            if "getEdges" in styles:
                style.update(styles["getEdges"])
            
            l.append( ( ("%s:o%s:e" % (get.outNode,get.outSocket), "%s:i%s:w" % (get.fromNode,get.fromSocket)) , style ) )
            
        addEdges( graph,l );
        
        l=[]
        for write in writters:
            style = {
                   #"label": "write" ,
                   "tooltip" : "out: %s -write-> in: %s" % (write.outSocket, write.toSocket)}
            if "writeEdges" in styles:
                style.update(styles["writeEdges"])
                
            l.append( (("%s:o%s:e" % (write.outNode,write.outSocket), "%s:i%s:w" % (write.toNode,write.toSocket)), style ) )
            
        addEdges( graph,l );


def addNodes(graph, nodes):
    for n in nodes:
        if isinstance(n, tuple):
            graph.node(n[0], **n[1])
        else:
            graph.node(n)
    return graph

def addEdges(graph, edges):
    for e in edges:
        if isinstance(e[0], tuple):
            graph.edge(*e[0], **e[1])
        else:
            graph.edge(*e)
    return graph


def applyStyles(graph, styles):
    graph.graph_attr.update(
        ('graph' in styles and styles['graph']) or {}
    )
    graph.node_attr.update(
        ('nodes' in styles and styles['nodes']) or {}
    )
    graph.edge_attr.update(
        ('edges' in styles and styles['edges']) or {}
    )
    return graph

def main():
         
    parser = ArgumentParser()
    
    parser.add_argument("-l", "--renderLogic", dest="renderLogic", 
        help="The converter logic xml to visualize", metavar="<path>", required=True)
    
    parser.add_argument("-s", "--style", dest="styleFile", 
        help="The style file (.json) for the visualization", metavar="<path>", required=False)
    
    parser.add_argument("-o", "--outputFile", dest="outputFile", default="./graph.svg" ,
        help="""Output file path where to store the visualization (supported all suffixes from graphviz""", metavar="<path>", required=True)
    
    
    # Open the xml
    opts= AttrMap(vars(parser.parse_args()))
    xmlDoc = ET.parse(opts.renderLogic)

    logicN = xmlDoc.find("Logic")
    if not logicN:
        raise ValueError("Logic xml: %s has no Logic node!" % opts.renderLogic)
    
    logic = AttrMap()
    for el in logicN:

        if el.tag not in logic:
            if el.tag == "Tool":
                logic[el.tag] = {}
            else:
                logic[el.tag] = []
        
        d = AttrMap({"maxOuts" : 0, "maxIns" : 0})
        
        if el.tag == "Tool":
            logic[el.tag][ int(el.attrib["id"]) ] =  d         
        else:
            logic[el.tag].append(d)
            
        for a in el.attrib:
            d[a] = el.attrib[a]
            
    if "Get" not in logic:
        logic["Get"] = [];
    if "Write" not in logic:
        logic["Write"] = [];
        
    # determine the maximal out and in ports of nodes from the getters and writters    
    for c in logic["Get"]:
            outNode = int(c.outNode)
            fromNode = int(c.fromNode)
            outSocket = int(c.outSocket)
            fromSocket = int(c.fromSocket)
            logic.Tool[outNode].maxOuts = max(logic.Tool[outNode].maxOuts, outSocket+1)
            logic.Tool[fromNode].maxIns = max(logic.Tool[fromNode].maxIns, fromSocket+1)
    for c in logic["Write"]:
            outNode = int(c.outNode)
            toNode = int(c.toNode)
            outSocket = int(c.outSocket)
            toSocket = int(c.toSocket)
            logic.Tool[outNode].maxOuts = max(logic.Tool[outNode].maxOuts, outSocket+1)
            logic.Tool[toNode].maxIns = max(logic.Tool[toNode].maxIns, toSocket+1)
              
        
    # make graphviz graph
    ext = os.path.splitext(opts.outputFile)[1][1:]
    graph = gv.Digraph(name=os.path.basename(opts.outputFile) , format=ext )
    
    if opts.styleFile:
      # load styles from json file
      d = jsonLoad(opts.styleFile)
      styles = d["styles"]
      general = d["general"]
    else:
      # default styles if no json file is given
      general = {
                  "nodesHeight" : 10,
                  "nodesWidth"  : 10,
                  "linebreaks"  : 10
                }
      styles = {
          'graph': {
              'fontname': 'Verdana',
              'label': 'logic execution graph for %s ' % os.path.basename(opts.renderLogic),
              'fontsize': '16',
              'fontcolor': 'white',
              'bgcolor': '#333333',
              'rankdir': 'LR',
              "nodesep": "0.1",
              "ranksep": "1",
              "mindist" : "0",
              "splines": "spline"
  #            "ratio" : "0.1",
  #            "splines" : "ortho"
          },
          'nodes': {
              'fontname': 'Verdana',
              'shape': 'plaintext',
              'fontcolor': 'white',
  #            'color': 'white',
  #            'style': 'filled',
              'fillcolor': '#006699',
          },
          'edges': {
              'style': 'solid',
              'color': 'white',
              'arrowhead': 'open',
              'fontname': 'Verdana',
              'fontsize': '10',
              'fontcolor': 'white',
              "weight" : "0"
          },
          'toolNodes-Body': {
              'fontname': 'Verdana',
              'shape': 'plaintext',
              'fontsize': '10',
              'fontcolor': 'white',
  #            'color': 'white',
  #            'style': 'filled',
              'fillcolor': '#006699'      
          },
          'toolNodes-Frame': {
              'fontname': 'Verdana',
              'shape': 'plaintext',
              'fontsize': '10',
              'fontcolor': 'white',
  #            'color': 'white',
  #            'style': 'filled',
              'fillcolor': 'green'       
          },
          
          'getEdges': {
              'style': 'solid',
              'color': 'blue',
              'penwidth':'2',
              'arrowhead': 'open',
              'fontname': 'Verdana',
              'fontsize': '12',
              'fontcolor': 'white'       
          },
          'writeEdges': {
              'style': 'solid',
              'color': 'grey',
              'penwidth':'2',
              'arrowhead': 'open',
              'fontname': 'Verdana',
              'fontsize': '12',
              'fontcolor': 'white'       
          }
      }    
    
    addTools(graph,logic["Tool"],styles,general)
    addConnections(graph,logic["Get"], logic["Write"],styles)
        
    graph.graph_attr.update(
        ('graph' in styles and styles['graph']) or {}
    )
    
    f=graph.render(filename=os.path.basename(opts.outputFile).split(".")[0])
    
    print("Output saved to: %s" % f)
    
if __name__ == "__main__":
   sys.exit(main());