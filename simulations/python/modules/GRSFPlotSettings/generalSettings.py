
def cm2inch(value):
      return value/2.54
      
def cm2pt(value):
        return cm2inch(value)*72

# in pt
defaultLineSettings = {
  "thin" : cm2pt(0.2/10),
  "extra-thin" :  cm2pt(0.1/10),
  "semi-thick" : cm2pt(0.3/10),
  "thick" : cm2pt(0.4/10),
  "extra-thick" : cm2pt(0.6/10)
}

defaultFontSize = 12
defaultMarkerSize = cm2pt(1.2/10)



distinctColors = {  "colors3" : ["#D7CF00","#008C1C","#006E8F"] ,
                    "colors3light" : ["#FFF854","#00FF33","#7CD1FF"],
                    "blackredgreen" : ["#000000","#FF0000","#006600"], 
                    "blackredgreenlight" : ["#737373","#FF8080","#66ff66"] }


def defaultFormatAxes(*axesArgs):
      for axes in axesArgs:
        if not isinstance(axes,list):
            axes =  [axes]
            
        for ax in axes:
            l = ax.get_legend()
            if l is not None:
                l.get_frame().set_linewidth(defaultLineSettings["thin"])
                
            for spine in ax.spines.values():
                  spine.set_linewidth(defaultLineSettings["semi-thick"])
                  
def defaultFormatColorbar(*cbars):
  for c in cbars:
    c.outline.set_linewidth(defaultLineSettings["thin"])

                 