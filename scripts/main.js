    
    var markdownBuffer = document.createElement("div")
    
    function renderAll(){
      
      //MathJax.Hub.Startup.signal.Interest(function (message) {console.log("Startup: "+message)});
      //MathJax.Hub.signal.Interest(function (message) {console.log("Hub: "+message)});

      // load wiki Home markup -> render mathjax -> render markdown -> replace videos -> parse/apply toc
      {
        //var fileUrl = "MathJaxTest.md" ; 
        var fileUrl = "https://rawgit.com/wiki/gabyx/GRSFramework/Home.md";
        var client1 = new XMLHttpRequest();
        
        console.log("get: ", fileUrl);
        client1.open('GET', fileUrl);
        
        client1.onreadystatechange = function() {
            if(client1.readyState == 4){
                console.log("loaded: ", fileUrl);
                
                // put loaded markdown text into content
                //document.getElementById('content').innerHTML = client1.responseText;
                markdownBuffer.innerHTML = client1.responseText;
                
                console.log("1. run mathjax");
                MathJax.Hub.Typeset(markdownBuffer, renderMarkDown);
                
            }
        }
        client1.send();
      }
    };
    
    function renderMarkDown(){
        

        
        marked.setOptions({
          renderer: new marked.Renderer(),
          gfm: true,
          tables: true,
          breaks: false,
          pedantic: false,
          sanitize: false, // IMPORTANT, because we do MathJax before markdown,
                           //            however we do escaping in 'CreatePreview'.
          smartLists: true,
          smartypants: false,
          highlight: function(code) {
            return hljs.highlightAuto(code).value;
          }
        });

        // load video subtitles
        jQuery.get('videoSubtitle.md',function(data){
          $("#videoSubtitle").html(  marked(data) );
        });
        
        var markedText = marked(markdownBuffer.innerHTML);
        document.getElementById('content').innerHTML = markedText;
        
        // replace videos with storage
        console.log("2. replace videos");
        var vidS=$("#videoStorage");
        $("#videos-placeholder").replaceWith( vidS );
        
        console.log("3. parse/apply TOC");
        parseTOC('#content','#toc-level1')
        applyTOC('#toc-button','#toc');
        
        // remove loader div
        $(".loaderdiv").fadeOut("slow");
        


    }
    
    function parseTOC(from, to) {
      // parse in the TOC from marked markdown text
      var tocPlaceholder = $(to);
      var staticContent = $(from);
      
      staticContent.find('h1').each(function() {
        tocPlaceholder.append('<li id="'+ $(this).attr('id') + '-menu"><a href="#' + $(this).attr('id') + '">' + $(this).html() + '</li>');
      });
      staticContent.find('h2').each(function() {
        prevTitle = tocPlaceholder.find('#' + $(this).prevAll('h1').first().attr('id') + '-menu');
        prevTitle.not(":has(ul)").append('<ul></ul>');
        prevTitle.find('ul').append('<li id="'+ $(this).attr('id') + '-menu"><a href="#' + $(this).attr('id') + '">' + $(this).html() + '</li>');
      });
      
      staticContent.find('h3').each(function() {
        prevTitle = tocPlaceholder.find('#' + $(this).prevAll('h2').first().attr('id') + '-menu');
        prevTitle.not(":has(ul)").append('<ul></ul>');
        prevTitle.find('ul').append('<li id="'+ $(this).attr('id') + '-menu"><a href="#' + $(this).attr('id') + '">' + $(this).html() + '</li>');
      });
      
    }

    function applyTOC(buttonSel,srcSel) {
      
      var set = false,
      $button = $(buttonSel).sidr({
        name: 'sidr-toc',
        source: srcSel
      });
      
      var startOffset = 360;
      
      $(window).scroll(function() {
        var top = window.scrollY;
        if (top > startOffset && !set) {
          // make TOC button dark when it leaves the header
          $button[0].classList.add('darker');
          set = true;
        } else if (top < startOffset && set) {
          $button[0].classList.remove('darker');
          set = false;
        }
      });
      
      // add class 'nav' to  make scrollspy work
      $(".sidr-inner").addClass("nav");
      
      // add expList class to make expandable
      $("#sidr-id-toc-level1").addClass("expList");
      prepareList("#sidr-id-toc-level1");
      
      // add +/- button to sidr-toc
      $("#sidr-id-table-of-contents").prepend($("#toc-buttons-expcoll"));
      
      $('body').scrollspy({offset:400});
      
    };
    
    
    // register renderAll to 
    $(window).load( renderAll );
    