
    String.prototype.format = function()
    {
      var content = this;
      for (var i=0; i < arguments.length; i++)
      {
          var replacement = '{' + i + '}';
          content = content.replace(replacement, arguments[i]);
      }
      return content;
    };

    var mkConverter = new showdown.Converter(
    { literalMidWordUnderscores : true,
      tables : true,
      ghCodeBlocks : true,
      tasklists : true}
    );

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

    // use marked
    var renderMKtoHTML = function (text){ return marked(text); };

    // use showdown
    //var renderMKtoHTML = function (text){ return mkConverter.makeHtml(text); };

    var markdownBufferText = null;
    var markdownBufferDiv = document.createElement("div");
    var $markdownBufferDiv = null;

    function renderAll(){

      //MathJax.Hub.Startup.signal.Interest(function (message) {console.log("Startup: "+message)});
      //MathJax.Hub.signal.Interest(function (message) {console.log("Hub: "+message)});

      // load wiki Home markup -> render markdown -> render mathjax -> replace videos -> parse/apply toc
      {
        //var fileUrl = "MathJaxTest.md" ;
        var fileUrl = "https://rawgit.com/wiki/gabyx/GRSFramework/Home.md";
        var client1 = new XMLHttpRequest();

        console.log("get: ", fileUrl);
        client1.open('GET', fileUrl);
        //client1.overrideMimeType("text/plain; charset=utf-8");
        client1.onreadystatechange = function() {
            if(client1.readyState == 4){
                console.log("loaded: ", fileUrl);

                // put loaded markdown text into content
                //document.getElementById('content').innerHTML = client1.responseText;

                console.log("render markdown");
                markdownBufferDiv.innerHTML = renderMKtoHTML(client1.responseText);
                $markdownBufferDiv = $(markdownBufferDiv);

                // load video subtitles
                jQuery.get('videoSubtitle.md',function(data){

                  //replace videos with storage
                  console.log("replace videos");
                  $markdownBufferDiv.find("#videos-placeholder").replaceWith( $("#videoStorage") );

                  $markdownBufferDiv.find("#videoSubtitle").html(  renderMKtoHTML(data) );

                  //replace JobWorkflow with CSS slider
                  $markdownBufferDiv.find("#jobWorkflowSlides").html(
                      buildCSSSlider("https://rawgit.com/wiki/gabyx/GRSFramework/files/DefencePresentation-{0}.svg",155,172,"jobWorkflowSlider")
                  );

                  // remove loader div
                  $(".loaderdiv").fadeOut("slow", function (){

                    // make content appear
                    $("#content").html($markdownBufferDiv);

                    // Start ISM slider
                    window.ISM.startISM();

                    console.log("3. parse/apply TOC");
                    parseTOC("#content",'#toc-level1')
                    applyTOC('#toc-button','#toc');

                    var $t=$(window.location.hash);
                    if($t[0]){
                      $t[0].scrollIntoView(true);
                    }

                    console.log("run mathjax");
                    MathJax.Hub.Typeset(document.getElementById('content'));


                  });

                });

            }
        }
        client1.send();
      }
    };


    function buildCSSSlider(image,startIdx,stopIdx,id){
        cssSlider = document.createElement("div");
        cssSlider.setAttribute("id",id);
        cssSlider.setAttribute("data-transition_type","instant");
        cssSlider.classList.add("ism-slider");

        nImgs = stopIdx - startIdx;
        text = '<ol>\n'
        for(i=0;i < nImgs;i++){
             sNr = i+1;
             imageS = image.format(startIdx+i);
             console.log("build cssSlider:", imageS)
             text  +='<li>\n\
                 <img src="{0}" />\n\
                 <!--<div class="ism-caption ism-caption-0">Slide {1}</div>-->\n\
             </li>'.format(imageS,sNr);
        }
        text += "</ol>";
        cssSlider.innerHTML = text;

        return cssSlider;
    }

    function parseTOC(from, to) {
      // parse in the TOC from marked markdown text
      var tocPlaceholder = $(to);
      var staticContent = $(from);

      //console.log(staticContent)
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
