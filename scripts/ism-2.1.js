/*
  Copyright © 2015 Jay Wilson / ImageSliderMaker.com - All Rights Reserved
*/

(function() {

  //////////////
  // ISMQuery //
  //////////////

  // ISMQuery (iq): Simple jQuery-like implementation for IE8+ DOM manipulation under ISM
  function ISMQuery(el_or_selector, node_list) {
    this.selector = null;
    if(node_list)
    {
      this.nodes = node_list;
    }
    else if(typeof el_or_selector == "string")
    {
      this.selector = el_or_selector;
      this.nodes = document.querySelectorAll(el_or_selector);
    }
    else // assume single node; set this.nodes as raw array, which is okay for our needs
    {
      this.nodes = [el_or_selector];
    }
    this.length = this.nodes.length;
    this.el = this.nodes[0];
  };

  function iq(el_or_selector) {
    return new ISMQuery(el_or_selector);
  };

  ISMQuery.prototype.get = function(index) {
    return this.nodes[index];
  };

  ISMQuery.prototype.find = function(selector) {
    var el = this.el;
    var found_nodes = el.querySelectorAll(selector);
    return new ISMQuery(null, found_nodes);
  };

  ISMQuery.prototype.index = function() {
    var child = this.el;
    var i = 0;
    while((child = child.previousSibling) != null)
    {
      i++;
    }
    return i;
  };

  ISMQuery.prototype.each = function(cb) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      var iqObj = new ISMQuery(el);
      cb.call(iqObj, i, iqObj);
    }
  };

  ISMQuery.prototype.remove = function() {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      el.parentNode.removeChild(el);
    }
  };

  ISMQuery.prototype.removeAttr = function(name) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      el.removeAttribute(name);
    }
  };

  ISMQuery.prototype.attr = function(name, value) {
    var el = this.el;
    if(value != undefined)
    {
      el.setAttribute(name, value);
    }
    else
    {
      return el.getAttribute(name);
    }
  };

  ISMQuery.prototype.data = function(name) {
    var el = this.el;
    return el.getAttribute("data-" + name);
  };

  ISMQuery.prototype.hasClass = function(class_name) {
    var el = this.el;
    if(el.classList)
    {
      return el.classList.contains(class_name);
    }
    else
    {
      return (new RegExp('(^| )' + class_name + '( |$)', 'gi')).test(el.className);
    }
  };

  ISMQuery.prototype.addClass = function(class_name) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      if(el.classList)
      {
        el.classList.add(class_name);
      }
      else
      {
        el.className += " " + class_name;
      }
    }
  };

  ISMQuery.prototype.removeClass = function(class_name) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      if(el.classList)
      {
        el.classList.remove(class_name.split(" "));
      }
      else
      {
        el.className = el.className.replace(new RegExp('(^|\\b)' + class_name.split(' ').join('|') + '(\\b|$)', 'gi'), ' ');
      }
    }
  };

  ISMQuery.prototype.show = function() {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      if(el && el.style)
      {
        el.style.display = "";
      }
    }
  };

  ISMQuery.prototype.hide = function() {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.el;
      if(el && el.style)
      {
        el.style.display = "none";
      }
    }
  };

  ISMQuery.prototype.css = function(prop, value) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      el.style[prop] = value;
    }
  };

  ISMQuery.prototype.fadeIn = function(duration, callback) {
    var el = this.el;
    duration = duration || 400;
    var opacity = 0;
    el.style.opacity = 0;
    el.style.filter = '';
    var last = +new Date();
    var tick = function() {
      opacity += (new Date() - last) / duration;
      el.style.opacity = opacity;
      el.style.filter = 'alpha(opacity=' + (100 * opacity)|0 + ')';
      last = +new Date();
      if(opacity < 1)
      {
        (window.requestAnimationFrame && requestAnimationFrame(tick)) || setTimeout(tick, 16);
      }
      else if(callback)
      {
        callback();
      }
    };
    tick();
  };

  ISMQuery.prototype.fadeOut = function(duration, callback) {
    var el = this.el;
    duration = duration || 400;
    var opacity = 1;
    el.style.opacity = 1;
    el.style.filter = '';
    var last = +new Date();
    var tick = function() {
      opacity -= (new Date() - last) / duration;
      el.style.opacity = opacity;
      el.style.filter = 'alpha(opacity=' + (100 * opacity)|0 + ')';
      last = +new Date();
      if(opacity > 0)
      {
        (window.requestAnimationFrame && requestAnimationFrame(tick)) || setTimeout(tick, 16);
      }
      else if(callback)
      {
        callback();
      }
    };
    tick();
  };

  ISMQuery.prototype.wrap = function(html) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      var el_index = (new ISMQuery(el)).index();
      var el_html = el.outerHTML;
      var parent = el.parentNode;
      el.insertAdjacentHTML("beforebegin", html);
      ////el.remove();
      parent.removeChild(el);
      var wrap_elem = parent.childNodes[el_index];
      wrap_elem.innerHTML = el_html;
    }
  };

  ISMQuery.prototype.wrapInner = function(html) {
    var el = this.el;
    var el_inner_html = el.innerHTML;
    el.innerHTML = html;
    el.firstChild.innerHTML = el_inner_html;
  };

  ISMQuery.prototype.unwrap = function() {
    var el = this.el;
    el.parentNode.outerHTML = el.parentNode.innerHTML;
  };

  ISMQuery.prototype.after = function(html) {
    var el = this.el;
    el.insertAdjacentHTML("afterend", html);
  };

  ISMQuery.prototype.append = function(html) {
    var el = this.el;
    el.insertAdjacentHTML("beforeend", html);
  };

  ISMQuery.prototype.insertAfter = function(an_el) {
    var el = this.el;
    an_el.parentNode.insertBefore(el, an_el.nextSibling);
  };

  ISMQuery.prototype.clone = function() {
    var el = this.el;
    return new ISMQuery(el.cloneNode(true)); // deep == true
  };

  ISMQuery.prototype.on = function(event_name, handler) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      if(el.addEventListener)
      {
        el.addEventListener(event_name, handler);
      }
      else
      {
        el.attachEvent("on" + event_name, function() {
          handler.call(el);
        });
      }
    }
  };

  ISMQuery.prototype.off = function(event_name, handler) {
    for(var i = 0; i < this.nodes.length; i++)
    {
      var el = this.nodes[i];
      if(el)
      {
        if(el.removeEventListener)
        {
          el.removeEventListener(event_name, handler);
        }
        else
        {
          el.detachEvent("on" + event_name, handler);
        }
      }
    }
  };

  // static
  iq.ready = function(cb) {
    console.log("document.readyState(" + document.readyState + ") document.addEventListener(" + (!!document.addEventListener) + ")");
    if(document.readyState === "complete")
    {
      setTimeout(cb, 1);
    }
    else if(document.addEventListener)
    {
      document.addEventListener("DOMContentLoaded", cb, false);
    }
    else
    {
      document.attachEvent("onreadystatechange", function() {
        if(document.readyState === "complete")
        {
          cb();
        }
      });
    }
  };

  //////////
  //////////
  //////////

  function ISMSlider(element_id, options) {

    var default_options = {
      transition_type: "slide", // "instant" | "slide" | "fade" | "zoom"
      play_type: "manual", // "manual" | "once" | "once-rewind" | "loop"
      interval: 7000, // time in ms
      image_fx: "none", // "none" | "zoompan" | "zoomrotate"
      buttons: true, // true | false
      radios: true, // true | false
      radio_type: "button", // "button" | "thumbnail"
      pause_button: true, // true | false
      transition_duration: 350, // applies to fade and zoom transition types only
      swipe: true // true | false
    };

    var default_captions = [{enable:false,delay:0}, {enable:false,delay:200}, {enable:false,delay:400}];

    var opts, ol, slide_width_pc, slide_index,
        buttons, prev_button, next_button, radios, pause_button,
        autoplay_run, autoplay_timeout,
        in_transition, dragger, listeners = {};

    init(options, false, options.prevent_stop_loading || false);

    //////////
    // INIT //
    //////////

    function init(options_arg, reset, prevent_stop_loading) {

      options_arg = options_arg || {};

      start_loading();

      if(reset !== false)
      {
        // Reset all HTML, CSS, animation state
        reset_all();
      }

      opts = null;
      ol = null;
      slide_width_pc = null;
      slide_index = (-1);
      buttons = null;
      prev_button = null;
      next_button = null;
      radios = null;
      pause_button = null;
      autoplay_run = false;
      autoplay_timeout = null;
      in_transition = false;
      dragger = null;

      // Merge given config with default config
      init_config(options_arg);

      // Create references to slide elements and count slides
      // Modify markup - add classes, container divs, ...
      analyze_and_modify_markup();

      // Set animation type to "instant", "slide", "fade" or "zoom"
      set_transition_type(opts.transition_type, true);

      // Initialise captions
      init_captions();

      // Render markup for previous and next buttons and bind click events
      init_buttons();

      // Render radios markup and bind click events
      init_radios();

      // Initialise autoplay
      set_play_type(opts.play_type);

      // Listen for swipe events on slider
      bind_swipe();

      setTimeout(function() {
        if(prevent_stop_loading !== true)
        {
          stop_loading();
        }
      }, 1000);

      console.log("ISMSlider Ready");

    };

    /////////////
    // LOADING //
    /////////////

    function start_loading() {

      iq("#" + element_id + "-ism-loading-mask").remove();

      var loading_div = document.createElement("DIV");

      loading_div.id = element_id + "-ism-loading-mask";
      loading_div.style.position = "absolute";
      loading_div.style.zIndex = 10;
      loading_div.style.top = 0;
      loading_div.style.bottom = 0;
      loading_div.style.left = 0;
      loading_div.style.right = 0;
      loading_div.style.backgroundColor = "#eee";

      iq("#" + element_id).el.appendChild(loading_div);

    };

    function stop_loading() {

      iq("#" + element_id).show();

      var loading_mask = iq("#" + element_id + "-ism-loading-mask");
      if(loading_mask.length == 1)
      {
        iq("#" + element_id + "-ism-loading-mask").fadeOut(400, function() {
          iq("#" + element_id + "-ism-loading-mask").remove();

          // Run image effects on initial slide
          run_image_fx(0);

          // This is normally called by the transition functions
          trigger_slide_captions(0);

        });
      }

    };

    ////////////
    // CONFIG //
    ////////////

    function init_config(given_opts) {

      // Config provided using JavaScript has precedence
      // <div class="ism-slider" data-transition-type="" data-play-type="" data-pause="" data-interval="" data-image-fx="" data-buttons="" data-radios="">

      for(var p in default_options)
      {
        var type = typeof default_options[p];
        if(given_opts[p] == undefined && type != "object")
        {
          var data_value = iq("#" + element_id).data(p);
          if(data_value != undefined)
          {
            given_opts[p] = data_value;
          }
        }
      }

      opts = given_opts || {};

      // Apply default options where no value provided
      for(var p in default_options)
      {
        if(opts[p] == undefined)
        {
          opts[p] = default_options[p];
        }
      }

      // Get caption data attributes or apply defaults in their absence
      opts.captions = [];

      var li_els = iq("#" + element_id + " ol > li");
      var slide_count = li_els.length;
      for(var s_i = 0; s_i < slide_count; s_i++)
      {
        opts.captions[s_i] = [];
        for(var c_i = 0; c_i < 3; c_i++)
        {
          var caption_config = {
            enable: default_captions[c_i].enable,
            delay: default_captions[c_i].delay
          };
          var caption_el = iq(li_els.get(s_i)).find(".ism-caption-" + c_i);
          if(caption_el.length == 1)
          {
            caption_config.enable = true;
            var data_delay = caption_el.data("delay");
            if(!isNaN(data_delay))
            {
              caption_config.delay = parseInt(data_delay);
            }
          }
          opts.captions[s_i][c_i] = caption_config;
        }
      }

    };

    ///////////
    // RESET //
    ///////////

    function reset_all() {

      reset_autoplay();
      remove_image_fx();
      unbind_swipe();
      unbind_buttons();
      unbind_radios();
      unbind_pause_button();
      iq("#" + element_id + " .ism-button").remove();
      iq("#" + element_id + " .ism-radios").remove();
      iq("#" + element_id + " .ism-cloned").remove();
      iq("#" + element_id + " .ism-tmp-clone").remove();
      while(iq("#" + element_id + " .ism-frame").length > 0)
      {
        iq("#" + element_id + " .ism-slides").unwrap();
      }
      if(iq("#" + element_id + " .ism-img-frame").length > 0)
      {
        iq("#" + element_id + " .ism-img").unwrap();
      }

      iq("#" + element_id + " .ism-caption").removeAttr("style");
      iq("#" + element_id + " .ism-slide").show();
      iq("#" + element_id + " .ism-slide").removeAttr("style");
      iq("#" + element_id + " .ism-slides").removeAttr("style");
      iq("#" + element_id + " .ism-img").removeClass("ism-img");

      iq("#" + element_id + " .ism-slide").removeClass("ism-slide ism-slide-0 ism-slide-1 ism-slide-2 ism-slide-3 ism-slide-4 ism-slide-5 ism-slide-6 ism-slide-7 ism-slide-8 ism-slide-9");
      iq("#" + element_id + " .ism-slides").removeClass("ism-slides");
      iq("#" + element_id).removeClass("active");

    };

    ////////////////////////
    // ANALYSE AND MODIFY //
    ////////////////////////

    function analyze_and_modify_markup() {

      ol = iq("#" + element_id + " ol");
      var slide_count = iq("#" + element_id + " ol > li").length;
      slide_width_pc = 100.0 / (slide_count);
      slide_index = 0;

      ol.addClass("ism-slides");
      iq("#" + element_id + " .ism-slides > li").addClass("ism-slide");
      iq("#" + element_id + " .ism-slides > .ism-slide > img, #" + element_id + " .ism-slides > .ism-slide > a > img").addClass("ism-img");

      iq("#" + element_id + " .ism-slides > .ism-slide > object, #" + element_id + " .ism-slides > .ism-slide > a > object").addClass("ism-img");


      ol.find("li").each(function(indx) {
        this.addClass("ism-slide-" + indx);
      });

      var slider_frame_html = "<div class='ism-frame'></div>";
      iq("#" + element_id).wrapInner(slider_frame_html); // this breaks things
      ol = iq("#" + element_id + " ol.ism-slides"); // wrapInner method replaces ol node with copy

      var img_frame_html = "<div class='ism-img-frame'></div>";
      iq("#" + element_id + " .ism-img").wrap(img_frame_html);

    };

    function get_slide_count() {

      return iq("#" + element_id + " .ism-slide").length;

    };

    function get_active_slide_index() {

      return slide_index;

    };

    //////////////
    // AUTOPLAY //
    //////////////

    function set_play_type(play_type) {

      opts.play_type = play_type;

      // Render markup for pause button and bind click event
      init_pause_button();

      continue_autoplay()

    };

    function set_interval(play_rate) {

      opts.interval = play_rate;

    };

    function continue_autoplay() {

      autoplay_run = true;
      iq("#" + element_id + " .ism-pause-button").removeClass("ism-play");

      if(opts.play_type != "manual")
      {
        clearTimeout(autoplay_timeout);
        autoplay_timeout = setTimeout(do_autoplay, opts.interval);
      }

    };

    function do_autoplay() {

      var next_slide_index = slide_index + 1;

      if(autoplay_run && opts.play_type != "manual")
      {
        if(opts.play_type == "once" && slide_index == get_slide_count() - 2) // penultimate slide
        {
          transition(next_slide_index);
          pause_autoplay();
        }
        else if(opts.play_type == "once-rewind" && slide_index == get_slide_count() - 1) // last slide
        {
          transition(0);
          pause_autoplay();
        }
        else if(opts.play_type == "loop" && slide_index == get_slide_count() - 1) // last slide
        {
          transition(0);
          continue_autoplay();
        }
        else
        {
          transition(next_slide_index);
          continue_autoplay();
        }
      }

    };

    function pause_autoplay() {

      autoplay_run = false;
      clearTimeout(autoplay_timeout);
      autoplay_timeout = null;

      iq("#" + element_id + " .ism-pause-button").addClass("ism-play");

    };

    function reset_autoplay() {

      pause_autoplay();

    };

    function user_play(new_slide_index, callback) {

      if(new_slide_index != slide_index)
      {
        pause_autoplay();
        transition(new_slide_index, callback);
      }

    };

    /////////////
    // BUTTONS //
    /////////////

    function init_buttons() {

      if(opts.buttons)
      {
        var buttons_html = "<div class='ism-button ism-button-prev'>&nbsp;</div>"
                        + "<div class='ism-button ism-button-next'>&nbsp;</div>";
        ol.after(buttons_html);
        prev_button = iq("#" + element_id + " .ism-button-prev");
        next_button = iq("#" + element_id + " .ism-button-next");

        // Listen for click of prev button
        prev_button.on("click", prev_button_handler);
        prev_button.on("touchstart", prev_button_handler);

        // Listen for click of next button
        next_button.on("click", next_button_handler);
        next_button.on("touchstart", next_button_handler);
      }

    };

    function enable_buttons(enable) {
      if(enable === true && !opts.buttons)
      {
        unbind_buttons();
        iq("#" + element_id + " .ism-button").remove();
        init_buttons();
      }
    };

    function unbind_buttons() {

      iq("#" + element_id + " .ism-button-prev").off("click", prev_button_handler);
      iq("#" + element_id + " .ism-button-prev").off("touchstart", prev_button_handler);
      iq("#" + element_id + " .ism-button-next").off("click", next_button_handler);
      iq("#" + element_id + " .ism-button-next").off("touchstart", next_button_handler);

    };

    function prev_button_handler(e) {
      if(e && e.preventDefault)
      {
        e.preventDefault();
      }
      if(e && e.stopPropagation)
      {
        e.stopPropagation();
      }
      user_play(slide_index - 1);
    };

    function next_button_handler(e) {
      if(e && e.preventDefault)
      {
        e.preventDefault();
      }
      if(e && e.stopPropagation)
      {
        e.stopPropagation();
      }
      user_play(slide_index + 1);
    };

    ////////////
    // RADIOS //
    ////////////

    function set_radio_type(radio_type) {

      if(radio_type == "thumbnail")
      {
        iq("#" + element_id + " .ism-radios").addClass("ism-radios-as-thumbnails");
      }
      else
      {
        iq("#" + element_id + " .ism-radios").removeClass("ism-radios-as-thumbnails");
      }

    };

    function init_radios() {

      // Radios
      if(opts.radios)
      {
        iq("#" + element_id).append("<ul class='ism-radios'></ul>");
        radios = iq("#" + element_id + " .ism-radios");

        if(opts.radio_type == "thumbnail")
        {
          radios.addClass("ism-radios-as-thumbnails");
        }

        for(var i = 0; i < get_slide_count(); i++)
        {
          if(i == 0)
          {
            radios.append("<li class='ism-radio-" + i + " active'>"
                        + "<input type='radio' name='ism-radio' class='ism-radio' id='ism-radio-" + i + "' checked='checked' />"
                        + "<label class='ism-radio-label' for='ism-radio-" + i + "'></label>"
                        + "</li>");
          }
          else
          {
            radios.append("<li class='ism-radio-" + i + "'>"
                        + "<input type='radio' name='ism-radio' class='ism-radio' id='ism-radio-" + i + "' />"
                        + "<label class='ism-radio-label' for='ism-radio-" + i + "'></label>"
                        + "</li>");
          }
        }

        iq("#" + element_id + " .ism-radios input.ism-radio, #" + element_id + " .ism-radios .ism-radio-label").on("click", radio_handler);
        iq("#" + element_id + " .ism-radios input.ism-radio, #" + element_id + " .ism-radios .ism-radio-label").on("touchstart", radio_handler);
      }

    };

    function enable_radios(enable) {
    };

    function unbind_radios() {
      iq("#" + element_id + " .ism-radios input.ism-radio, #" + element_id + " .ism-radios .ism-radio-label").off("click", radio_handler);
      iq("#" + element_id + " .ism-radios input.ism-radio, #" + element_id + " .ism-radios .ism-radio-label").off("touchstart", radio_handler);
    };

    function radio_handler(e) {
      if(e.preventDefault)
      {
        e.preventDefault();
      }
      if(e.stopPropagation)
      {
        e.stopPropagation();
      }

      var radio_index = iq(e.target.parentNode).index();
      user_play(radio_index);
    };

    function refresh_radios(new_slide_index) {

      if(opts.radios)
      {
        iq("#" + element_id + " .ism-radios li").removeClass("active");
        var radio_li = iq("#" + element_id + " .ism-radios li").get(new_slide_index);
        iq(radio_li).addClass("active");
        iq(radio_li).find("input").attr("checked", "checked");
      }

    };

    //////////////////
    // PAUSE BUTTON //
    //////////////////

    function init_pause_button() {

      unbind_pause_button();
      iq("#" + element_id + " .ism-pause-button").remove();

      if(opts.pause_button && opts.play_type != "manual")
      {
        var pause_button_html = "<div class='ism-pause-button'>&nbsp;</div>";
        ol.after(pause_button_html);
        pause_button = iq("#" + element_id + " .ism-pause-button");

        // Listen for click of pause button
        pause_button.on("click", pause_button_handler);
        pause_button.on("touchstart", pause_button_handler);
      }

    };

    function unbind_pause_button() {

      iq("#" + element_id + " .ism-pause-button").off("click", pause_button_handler);
      iq("#" + element_id + " .ism-pause-button").off("touchstart", pause_button_handler);

    };

    function pause_button_handler(e) {
      e.preventDefault();
      e.stopPropagation();

      if(autoplay_run)
      {
        pause_autoplay();
      }
      else
      {
        continue_autoplay();
      }

    };

    ///////////
    // SWIPE //
    ///////////

    function bind_swipe() {

      var slider_el = iq("#" + element_id).get(0);
      var handle_el = ol.get(0);

      dragger = new Dragdealer(slider_el, handle_el, {
        steps: get_slide_count(),
        x: 0,
        speed: 0.2,
        loose: true,
        requestAnimationFrame: true,
        dragStartCallback: function() {
          pause_autoplay();
        },
        dragStopCallback: function(x_val, y_val) {
          var real_index = (dragger.getStep()[0] - 1); // -1: convert from 1-based to 0-based index
          new_slide_index = real_index;
          pause_autoplay();
          after_swipe(new_slide_index);
        },
        onAfterGlide: function() {
          dragger.setStep(slide_index + 1, 1, true); // +1 because dragger uses 1-based index
        }
      });

      // After render and after slider's width has been determined
      window.onload = function() {
        setTimeout(function() { dragger.reflow(); }, 150);
        setTimeout(function() { dragger.reflow(); }, 600);
      };

    };

    function unbind_swipe() {
      if(dragger)
      {
        dragger.unbindEventListeners();
      }
    };

    function reflow() {
      if(dragger)
      {
        dragger.setStep(slide_index + 1, 1, true); // +1 because dragger uses 1-based index
        dragger.reflow();
      }
    };

    ////////////
    // EVENTS //
    ////////////

    function fire(event_name, args) {

      if(listeners[event_name])
      {
        listeners[event_name].apply(this, args);
      }

    };

    function listen(event_name, callback) {

      // Only allows one listener per event
      listeners[event_name] = callback;

    };

    /////////////////////
    // SLIDE ANIMATION //
    /////////////////////

    function set_transition_type(new_transition_type, force) {

      if(force != true && new_transition_type == opts["transition_type"])
      {
        return;
      }

      opts["transition_type"] = new_transition_type;

      iq("#" + element_id + " .ism-slide").removeClass("ism-zoom-in");
      iq("#" + element_id + " .ism-slide").show();

      var ol_unit_length = get_slide_count();
      var unit_fraction = 1.0 / ol_unit_length;
      ol.css("width", (100.0 * ol_unit_length) + "%");

      ol.find(".ism-slide").each(function(indx) {
        var left_percent = (slide_width_pc * indx) + "%";
        var width_percent = (100.0 / ol_unit_length) + "%";
        this.css("width", width_percent);
        this.css("left", left_percent);
      });

    };

    function transition(new_slide_index, callback) {

      if(in_transition !== true && new_slide_index != slide_index)
      {
        in_transition = true;

        var current_slide_index = slide_index;

        new_slide_index = parseInt(new_slide_index);

        if(new_slide_index < 0)
        {
          new_slide_index = get_slide_count() - 1;
        }
        else if(new_slide_index >= get_slide_count())
        {
          new_slide_index = 0;
        }

        before_transition(current_slide_index, new_slide_index);

        if(opts.transition_type == "instant")
        {
          instant_swap(current_slide_index, new_slide_index, callback);
        }
        else if(opts.transition_type == "slide")
        {
          hori_slide(current_slide_index, new_slide_index, callback);
        }
        else if(opts.transition_type == "fade")
        {
          fade(current_slide_index, new_slide_index, false, callback);
        }
        else if(opts.transition_type == "zoom")
        {
          fade(current_slide_index, new_slide_index, true, callback);
        }
      }

    };

    function before_transition(current_slide_index, new_slide_index) {

      refresh_radios(new_slide_index);

      slide_index = new_slide_index;

      fire("beforetransition", [new_slide_index]);

    };

    function instant_swap(current_slide_index, new_slide_index, callback) {

      dragger.setStep(new_slide_index + 1, 1, true);

      after_transition(current_slide_index, new_slide_index, true, callback);

    };

    function hori_slide(current_slide_index, new_slide_index, callback) {

      var target_ratio = (new_slide_index) / (get_slide_count() - 1); // e.g. 0/(4-1)=0, 1/(4-1)=0.33, 2/(4-1)=0.66, 3/(4-1)=1

      dragger.startSlide(target_ratio, function() {

        after_transition(current_slide_index, new_slide_index, true, callback);

      });

    };

    function fade(current_slide_index, new_slide_index, zoom, callback) {

      iq("#" + element_id + " li.ism-slide").removeClass("ism-zoom-in");

      var li_current = iq("#" + element_id + " li.ism-slide-" + current_slide_index);

      // Clone 'next' slide
      var ol_clone = ol.clone();
      ol_clone.addClass("ism-slides-clone");
      var pos_ratio = (new_slide_index) / (get_slide_count() - 1);
      var offset = dragger.getOffsetsByRatios([pos_ratio, 0]);
      ////ol_clone.hide();
      ol_clone.css("transform", "translateX(" + offset[0] + "px)");
      ol_clone.insertAfter(ol.el);

      if(zoom)
      {
        li_current.addClass("ism-zoom-in");
      }

      ol_clone.fadeIn(opts.transition_duration * 2, function() {
        dragger.setStep(new_slide_index + 1, 1, true);
        iq("#" + element_id + " .ism-slides-clone").remove();
        iq("#" + element_id + " .ism-slides").show();
        iq("#" + element_id + " .ism-slides").css("opacity", null);
        after_transition(current_slide_index, new_slide_index, true, callback);
        console.log("fadeIn ok");
      });

      ol.fadeOut(opts.transition_duration * 2 * 0.9, function() {
        console.log("fadeOut ok");
      });

    };

    function after_swipe(new_slide_index) {

      var current_slide_index = slide_index;

      slide_index = new_slide_index;
      fire("afterswipe", [new_slide_index]);
      refresh_radios(new_slide_index);

      after_transition(current_slide_index, new_slide_index, false);

    };

    function after_transition(current_slide_index, new_slide_index, do_reflow, callback) {

      new_slide_index = parseInt(new_slide_index);

      iq("#" + element_id + " .ism-slides-clone").remove();
      iq("#" + element_id + " .ism-slides").show();
      iq("#" + element_id + " li.ism-slide").removeClass("ism-zoom-in");

      if(do_reflow)
      {
        reflow();
      }

      if(callback) { callback(); }

      run_image_fx(new_slide_index);

      trigger_slide_captions(new_slide_index);

      in_transition = false;

      fire("aftertransition", [new_slide_index]);

    };

    //////////////
    // IMAGE FX //
    //////////////

    function set_image_fx(image_effect) {

      if(image_effect != opts["image_fx"])
      {
        opts["image_fx"] = image_effect;

        run_image_fx(slide_index);
      }
      else
      {
        opts["image_fx"] = image_effect;
      }

    };

    function run_image_fx(new_slide_index) {

      remove_image_fx();

      if(opts["image_fx"] == "none")
      {
        return;
      }
      else if(opts["image_fx"] == "zoompan")
      {
        iq("#" + element_id + " .ism-slide-" + new_slide_index + " .ism-img-frame").addClass("ism-zoom-pan");
      }
      else if(opts["image_fx"] == "zoomrotate")
      {
        iq("#" + element_id + " .ism-slide-" + new_slide_index + " .ism-img-frame").addClass("ism-zoom-rotate");
      }

    };

    function remove_image_fx() {

      iq("#" + element_id + " .ism-slide .ism-img-frame").removeClass("ism-zoom-pan");
      iq("#" + element_id + " .ism-slide .ism-img-frame").removeClass("ism-zoom-rotate");

    };

    //////////////
    // CAPTIONS //
    //////////////

    function init_captions() {

      ol.find("#" + element_id + " .ism-caption").css("visibility", "hidden");

    };

    function set_caption_enable(slide_index, caption_index, enable) {

      opts.captions[slide_index][caption_index].enable = enable;

    };

    function set_caption_delay(slide_index, caption_index, delay_ms) {

      opts.captions[slide_index][caption_index].delay = delay_ms;

    };

    function trigger_slide_captions(s_i) {

      ol.find(".ism-caption").css("visibility", "hidden");
      ol.find(".ism-caption").removeClass("ism-caption-anim");

      caption_transition(s_i, 0);
      caption_transition(s_i, 1);
      caption_transition(s_i, 2);

    };

    function caption_transition(s_i, c_i) {

      if(opts.captions[s_i] && opts.captions[s_i][c_i].enable == true)
      {
        setTimeout(function() {
          ol.find(".ism-slide-" + s_i + " .ism-caption-" + c_i).css("visibility", "visible");
          ol.find(".ism-slide-" + s_i + " .ism-caption-" + c_i).addClass("ism-caption-anim");
        }, opts.captions[s_i][c_i].delay);
      }

    };

    function set_element_id(new_element_id) {

      var elem = document.getElementById(element_id);
      elem.id = new_element_id;
      element_id = new_element_id;

    };

    ////////////
    // PUBLIC //
    ////////////

    this.init = init;
    this.deinit = reset_all;
    this.stopLoading = stop_loading;
    this.transition = transition;
    this.listen = listen;
    this.reflow = reflow;
    this.setTransitionType = set_transition_type;
    this.setPlayType = set_play_type;
    this.setInterval = set_interval;
    this.setImageFx = set_image_fx;
    this.setCaptionEnable = set_caption_enable;
    this.setCaptionDelay = set_caption_delay;
    this.enableButtons = enable_buttons;
    this.enableRadios = enable_radios;
    this.setRadioType = set_radio_type;
    this.getSlideCount = get_slide_count;
    this.getActiveSlideIndex = get_active_slide_index;
    this.setElementId = set_element_id;

  };

  window.ISM = window.ISM || {};
  window.ISM.Slider = ISMSlider;
  window.ISM.Config = window.ISM.Config || {};
  window.ISM.instances = [];

  window.ISM.startISM = function() {
    console.log("ISM.startISM");
    if(window.ISM.Config.no_instantiation !== true)
    {
      var slider_divs = iq(".ism-slider");
      console.log(slider_divs + " - " + slider_divs.length);
      for(var i = 0; i < slider_divs.length; i++)
      {
        console.log("[" + i + "]");
        var slider_div = slider_divs.get(i);
        slider_div.id = slider_div.id || "ism-slider-" + i;
        window.ISM.instances.push(new ISMSlider(slider_div.id, {}));
      }
    }
  };

  //iq.ready(window.ISM.startISM);

})();

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Dragdealer.js 0.9.8 - ISM mod

(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    // AMD. Register as an anonymous module.
    define(factory);
  } else {
    // Browser globals
    root.Dragdealer = factory();
  }
}(this, function () {

var Dragdealer = function(wrapper_el, handle_el, options) {

  this.options = this.applyDefaults(options || {});
  this.bindMethods();
  this.wrapper = wrapper_el;
  this.handle = handle_el;
  this.init();
  this.bindEventListeners();
};

Dragdealer.prototype = {
  defaults: {
    disabled: false,
    horizontal: true,
    vertical: false,
    slide: true,
    steps: 0,
    snap: false,
    loose: false,
    speed: 0.1,
    xPrecision: 0,
    yPrecision: 0,
    activeClass: 'active',
    css3: true,
    tapping: true,
    afterSwipeCallback: function(){}
  },
  init: function() {
    if (this.options.css3) {
      triggerWebkitHardwareAcceleration(this.handle);
    }
    this.value = {
      prev: [-1, -1],
      current: [this.options.x || 0, this.options.y || 0],
      target: [this.options.x || 0, this.options.y || 0]
    };
    this.offset = {
      wrapper: [0, 0],
      mouse: [0, 0],
      prev: [-999999, -999999],
      current: [0, 0],
      target: [0, 0]
    };
    this.change = [0, 0];
    this.stepRatios = this.calculateStepRatios();

    this.activity = false;
    this.dragging = false;
    this.tapping = false;
    this.sliding = false;
    this.slide_count = 0;

    this.reflow();
    if (this.options.disabled) {
      this.disable();
    }
  },
  applyDefaults: function(options) {
    for (var k in this.defaults) {
      if (!options.hasOwnProperty(k)) {
        options[k] = this.defaults[k];
      }
    }
    return options;
  },
  calculateStepRatios: function() {
    var stepRatios = [];
    if (this.options.steps >= 1) {
      for (var i = 0; i <= this.options.steps - 1; i++) {
        if (this.options.steps > 1) {
          stepRatios[i] = i / (this.options.steps - 1);
        } else {
          // A single step will always have a 0 value
          stepRatios[i] = 0;
        }
      }
    }
    return stepRatios;
  },
  setWrapperOffset: function() {
    this.offset.wrapper = Position.get(this.wrapper);
  },
  calculateBounds: function() {
    var bounds = {
      top: this.options.top || 0,
      bottom: -(this.options.bottom || 0) + this.wrapper.offsetHeight,
      left: this.options.left || 0,
      right: -(this.options.right || 0) + this.wrapper.offsetWidth
    };
    bounds.availWidth = (bounds.right - bounds.left) - this.handle.offsetWidth;
    bounds.availHeight = (bounds.bottom - bounds.top) - this.handle.offsetHeight;
    return bounds;
  },
  calculateValuePrecision: function() {
    var xPrecision = this.options.xPrecision || Math.abs(this.bounds.availWidth),
        yPrecision = this.options.yPrecision || Math.abs(this.bounds.availHeight);
    return [
      xPrecision ? 1 / xPrecision : 0,
      yPrecision ? 1 / yPrecision : 0
    ];
  },
  bindMethods: function() {
    if (typeof(this.options.customRequestAnimationFrame) === 'function') {
      this.requestAnimationFrame = bind(this.options.customRequestAnimationFrame, window);
    } else {
      this.requestAnimationFrame = bind(requestAnimationFrame, window);
    }
    if (typeof(this.options.customCancelAnimationFrame) === 'function') {
      this.cancelAnimationFrame = bind(this.options.customCancelAnimationFrame, window);
    } else {
      this.cancelAnimationFrame = bind(cancelAnimationFrame, window);
    }
    this.animateWithRequestAnimationFrame = bind(this.animateWithRequestAnimationFrame, this);
    this.animate = bind(this.animate, this);
    this.onHandleMouseDown = bind(this.onHandleMouseDown, this);
    this.onHandleTouchStart = bind(this.onHandleTouchStart, this);
    this.onDocumentMouseMove = bind(this.onDocumentMouseMove, this);
    this.onWrapperTouchMove = bind(this.onWrapperTouchMove, this);
    this.onWrapperMouseDown = bind(this.onWrapperMouseDown, this);
    this.onWrapperTouchStart = bind(this.onWrapperTouchStart, this);
    this.onDocumentMouseUp = bind(this.onDocumentMouseUp, this);
    this.onDocumentTouchEnd = bind(this.onDocumentTouchEnd, this);
    this.onHandleClick = bind(this.onHandleClick, this);
    this.onWindowResize = bind(this.onWindowResize, this);
  },
  bindEventListeners: function() {
    // Start dragging
    addEventListener(this.handle, 'mousedown', this.onHandleMouseDown);
    addEventListener(this.handle, 'touchstart', this.onHandleTouchStart);
    // While dragging
    addEventListener(document, 'mousemove', this.onDocumentMouseMove);
    addEventListener(this.wrapper, 'touchmove', this.onWrapperTouchMove);
    // Start tapping
    addEventListener(this.wrapper, 'mousedown', this.onWrapperMouseDown);
    addEventListener(this.wrapper, 'touchstart', this.onWrapperTouchStart);
    // Stop dragging/tapping
    addEventListener(document, 'mouseup', this.onDocumentMouseUp);
    addEventListener(document, 'touchend', this.onDocumentTouchEnd);

    addEventListener(this.handle, 'click', this.onHandleClick);
    addEventListener(window, 'resize', this.onWindowResize);

    this.animate(false, true);
    this.interval = this.requestAnimationFrame(this.animateWithRequestAnimationFrame);

  },
  unbindEventListeners: function() {
    removeEventListener(this.handle, 'mousedown', this.onHandleMouseDown);
    removeEventListener(this.handle, 'touchstart', this.onHandleTouchStart);
    removeEventListener(document, 'mousemove', this.onDocumentMouseMove);
    removeEventListener(this.wrapper, 'touchmove', this.onWrapperTouchMove);
    removeEventListener(this.wrapper, 'mousedown', this.onWrapperMouseDown);
    removeEventListener(this.wrapper, 'touchstart', this.onWrapperTouchStart);
    removeEventListener(document, 'mouseup', this.onDocumentMouseUp);
    removeEventListener(document, 'touchend', this.onDocumentTouchEnd);
    removeEventListener(this.handle, 'click', this.onHandleClick);
    removeEventListener(window, 'resize', this.onWindowResize);
    this.cancelAnimationFrame(this.interval);
  },
  onHandleMouseDown: function(e) {
    if(e.target && e.target.tagName == "A" && e.target.className.search(/ism-caption/) >= 0)
    {
      document.location = e.target.href;
    }
    Cursor.refresh(e);
    preventEventDefaults(e);
    stopEventPropagation(e);
    this.activity = false;
    this.startDrag();
  },
  onHandleTouchStart: function(e) {
    if(e.target && e.target.tagName == "A" && e.target.className.search(/ism-caption/) >= 0)
    {
      document.location = e.target.href;
    }
    Cursor.refresh(e);
    stopEventPropagation(e);
    this.activity = false;
    this.startDrag();
  },
  onDocumentMouseMove: function(e) {
    Cursor.refresh(e);
    if (this.dragging) {
      this.activity = true;
      preventEventDefaults(e);
    }
  },
  onWrapperTouchMove: function(e) {
    Cursor.refresh(e);
    if (!this.activity && this.draggingOnDisabledAxis()) {
      if (this.dragging) {
        this.stopDrag();
      }
      return;
    }
    preventEventDefaults(e);
    this.activity = true;
  },
  onWrapperMouseDown: function(e) {
    if(e.target && e.target.className.search(/ism-(button|radio|caption)/) >= 0)
    {
      return;
    }
    Cursor.refresh(e);
    preventEventDefaults(e);
    this.startTap();
  },
  onWrapperTouchStart: function(e) {
    Cursor.refresh(e);
    preventEventDefaults(e);
    this.startTap();
  },
  onDocumentMouseUp: function(e) {
    this.stopDrag();
    this.stopTap();
    if(e.target && e.target.className.search(/ism-(button|radio|caption)/) >= 0)
    {
      return;
    }
    this.options.afterSwipeCallback();
  },
  onDocumentTouchEnd: function(e) {
    this.stopDrag();
    this.stopTap();
    this.options.afterSwipeCallback();
  },
  onHandleClick: function(e) {
    if(e.target && e.target.className.search(/ism-(button|radio)/) >= 0)
    {
      return;
    }
    if (this.activity) {
      preventEventDefaults(e);
      stopEventPropagation(e);
    }
  },
  onWindowResize: function(e) {
    this.reflow();
  },
  enable: function() {
    this.disabled = false;
    this.handle.className = this.handle.className.replace(/\s?disabled/g, '');
  },
  disable: function() {
    this.disabled = true;
    this.handle.className += ' disabled';
  },
  reflow: function() {
    this.setWrapperOffset();
    this.bounds = this.calculateBounds();
    this.valuePrecision = this.calculateValuePrecision();
    this.updateOffsetFromValue();
  },
  getStep: function() {
    return [
      this.getStepNumber(this.value.target[0]),
      this.getStepNumber(this.value.target[1])
    ];
  },
  getValue: function() {
    return this.value.target;
  },
  setStep: function(x, y, snap) {
    this.setValue(
      this.options.steps && x > 1 ? (x - 1) / (this.options.steps - 1) : 0,
      this.options.steps && y > 1 ? (y - 1) / (this.options.steps - 1) : 0,
      snap
    );
  },
  setValue: function(x, y, snap) {
    this.setTargetValue([x, y || 0]);
    if (snap) {
      this.groupCopy(this.value.current, this.value.target);
      this.updateOffsetFromValue();
      this.callAnimationCallback();
    }
  },
  startTap: function() {
    if (this.disabled || !this.options.tapping) {
      return;
    }

    this.tapping = true;
    this.setWrapperOffset();

    this.setTargetValueByOffset([
      Cursor.x - this.offset.wrapper[0] - (this.handle.offsetWidth / 2),
      Cursor.y - this.offset.wrapper[1] - (this.handle.offsetHeight / 2)
    ]);
  },
  stopTap: function() {
    if (this.disabled || !this.tapping) {
      return;
    }
    this.tapping = false;

    this.setTargetValue(this.value.current);
  },
  startDrag: function() {
    if (this.disabled) {
      return;
    }
    this.dragging = true;
    this.interval = this.requestAnimationFrame(this.animateWithRequestAnimationFrame);
    this.setWrapperOffset();

    this.offset.mouse = [
      Cursor.x - Position.get(this.handle)[0],
      Cursor.y - Position.get(this.handle)[1]
    ];
    if (!this.wrapper.className.match(this.options.activeClass)) {
      this.wrapper.className += ' ' + this.options.activeClass;
    }
    this.callDragStartCallback();
  },
  stopDrag: function() {
    if (this.disabled || !this.dragging) {
      return;
    }
    this.dragging = false;

    var target = this.groupClone(this.value.current);
    if (this.options.slide) {
      var ratioChange = this.change;
      target[0] += ratioChange[0] * 4;
      target[1] += ratioChange[1] * 4;
    }
    this.setTargetValue(target);
    this.wrapper.className = this.wrapper.className.replace(' ' + this.options.activeClass, '');
    this.callDragStopCallback();
  },
  callAnimationCallback: function() {
    var value = this.value.current;
    if (this.options.snap && this.options.steps > 1) {
      value = this.getClosestSteps(value);
    }
    if (!this.groupCompare(value, this.value.prev)) {
      if (typeof(this.options.animationCallback) == 'function') {
        this.options.animationCallback.call(this, value[0], value[1]);
      }
      this.groupCopy(this.value.prev, value);
    }
  },
  callTargetCallback: function() {
    if (typeof(this.options.callback) == 'function') {
      this.options.callback.call(this, this.value.target[0], this.value.target[1]);
    }
  },
  callDragStartCallback: function() {
    if (typeof(this.options.dragStartCallback) == 'function') {
      this.options.dragStartCallback.call(this, this.value.target[0], this.value.target[1]);
    }
  },
  callDragStopCallback: function() {
    if (typeof(this.options.dragStopCallback) == 'function') {
      this.options.dragStopCallback.call(this, this.value.target[0], this.value.target[1]);
    }
  },
  startSlide: function(slide_target, callback) {
    this.slide_callback = callback;
    this.sliding = true;
    this.value.target[0] = slide_target;
    this.slide_start = this.value.current[0];
    this.step_size = Math.abs(this.value.target[0] - this.value.current[0]);
    this.interval = this.requestAnimationFrame(this.animateWithRequestAnimationFrame); // EDIT
  },
  animateWithRequestAnimationFrame: function (time) {
    if (time) {
      // using requestAnimationFrame
      this.timeOffset = this.timeStamp ? time - this.timeStamp : 0;
      this.timeStamp = time;
    } else {
      // using setTimeout(callback, 25) polyfill
      this.timeOffset = 25;
    }
    if(this.sliding)
    {
      this.animateSlide();
    }
    else
    {
      this.animate();
    }
    // only animate if dragging or gliding
    if(this.sliding || this.dragging || this.value.target[0] != this.value.current[0])
    {
      this.interval = this.requestAnimationFrame(this.animateWithRequestAnimationFrame);
    }
    else
    {
      this.options.onAfterGlide();
    }
  },
  animate: function(direct, first) {
    if (direct && !this.dragging) {
      return;
    }
    if (this.dragging) {
      var prevTarget = this.groupClone(this.value.target);

      var offset = [
        Cursor.x - this.offset.wrapper[0] - this.offset.mouse[0],
        Cursor.y - this.offset.wrapper[1] - this.offset.mouse[1]
      ];
      this.setTargetValueByOffset(offset, this.options.loose);

      this.change = [
        this.value.target[0] - prevTarget[0],
        this.value.target[1] - prevTarget[1]
      ];
    }
    if (this.dragging || first) {
      this.groupCopy(this.value.current, this.value.target);
    }
    if (this.dragging || this.glide() || first) {
      this.updateOffsetFromValue();
      this.callAnimationCallback();
    }
  },
  glide: function() {
    var diff = [
      this.value.target[0] - this.value.current[0],
      this.value.target[1] - this.value.current[1]
    ];
    if (!diff[0] && !diff[1]) {
      return false;
    }
    if (Math.abs(diff[0]) > this.valuePrecision[0] ||
        Math.abs(diff[1]) > this.valuePrecision[1]) {
      this.value.current[0] += diff[0] * Math.min(this.options.speed * this.timeOffset / 25, 1);
      this.value.current[1] += diff[1] * Math.min(this.options.speed * this.timeOffset / 25, 1);
    } else {
      this.groupCopy(this.value.current, this.value.target);
    }
    return true;
  },
  animateSlide: function() {
    var diff = this.value.target[0] - this.value.current[0];
    var sign = diff >= 0 ? 1 : (-1);
    var remain = Math.abs(diff);
    var progress = (this.value.current[0] - this.slide_start) / (this.value.target[0] - this.slide_start); // 0.0 .. 1.0
    var p = progress - 0.5; // -0.5 .. 0.5
    var d = ((-3 * p * p) + 0.8) * this.step_size * 0.08; // quadratic
    while(d > remain)
    {
      d *= 0.5;
    }
    if(progress > 0.995)
    {
      this.groupCopy(this.value.current, this.value.target);
      this.sliding = false;
      this.slide_callback();
    }
    else
    {
      this.value.current[0] += sign * d;
    }
    this.updateOffsetFromValue();
    this.renderHandlePosition();
    if(isNaN(progress))
    {
      //throw "progress NaN";
    }
  },
  updateOffsetFromValue: function() {
    if (!this.options.snap) {
      this.offset.current = this.getOffsetsByRatios(this.value.current);
    } else {
      this.offset.current = this.getOffsetsByRatios(
        this.getClosestSteps(this.value.current)
      );
    }
    if (!this.groupCompare(this.offset.current, this.offset.prev)) {
      this.renderHandlePosition();
      this.groupCopy(this.offset.prev, this.offset.current);
    }
  },
  renderHandlePosition: function() {
    var transform = '';
    if (this.options.css3 && StylePrefix.transform) {
      if (this.options.horizontal) {
        transform += 'translateX(' + this.offset.current[0] + 'px)';
      }
      this.handle.style[StylePrefix.transform] = transform;
      return;
    }
    if (this.options.horizontal) {
      this.handle.style.left = this.offset.current[0] + 'px';
    }
  },
  setTargetValue: function(value, loose) {
    var target = loose ? this.getLooseValue(value) : this.getProperValue(value);

    this.groupCopy(this.value.target, target);
    this.offset.target = this.getOffsetsByRatios(target);

    this.callTargetCallback();
  },
  setTargetValueByOffset: function(offset, loose) {
    var value = this.getRatiosByOffsets(offset);
    var target = loose ? this.getLooseValue(value) : this.getProperValue(value);

    this.groupCopy(this.value.target, target);
    this.offset.target = this.getOffsetsByRatios(target);
  },
  getLooseValue: function(value) {
    var proper = this.getProperValue(value);
    return [
      proper[0] + ((value[0] - proper[0]) / 4),
      proper[1] + ((value[1] - proper[1]) / 4)
    ];
  },
  getProperValue: function(value) {
    var proper = this.groupClone(value);

    proper[0] = Math.max(proper[0], 0);
    proper[1] = Math.max(proper[1], 0);
    proper[0] = Math.min(proper[0], 1);
    proper[1] = Math.min(proper[1], 1);

    if ((!this.dragging && !this.tapping) || this.options.snap) {
      if (this.options.steps > 1) {
        proper = this.getClosestSteps(proper);
      }
    }
    return proper;
  },
  getRatiosByOffsets: function(group) {
    return [
      this.getRatioByOffset(group[0], this.bounds.availWidth, this.bounds.left),
      this.getRatioByOffset(group[1], this.bounds.availHeight, this.bounds.top)
    ];
  },
  getRatioByOffset: function(offset, range, padding) {
    return range ? (offset - padding) / range : 0;
  },
  getOffsetsByRatios: function(group) {
    return [
      this.getOffsetByRatio(group[0], this.bounds.availWidth, this.bounds.left),
      this.getOffsetByRatio(group[1], this.bounds.availHeight, this.bounds.top)
    ];
  },
  getOffsetByRatio: function(ratio, range, padding) {
    return Math.round(ratio * range) + padding;
  },
  getStepNumber: function(value) {
    return this.getClosestStep(value) * (this.options.steps - 1) + 1;
  },
  getClosestSteps: function(group) {
    return [
      this.getClosestStep(group[0]),
      this.getClosestStep(group[1])
    ];
  },
  getClosestStep: function(value) {
    var k = 0;
    var min = 1;
    for (var i = 0; i <= this.options.steps - 1; i++) {
      if (Math.abs(this.stepRatios[i] - value) < min) {
        min = Math.abs(this.stepRatios[i] - value);
        k = i;
      }
    }
    return this.stepRatios[k];
  },
  groupCompare: function(a, b) {
    return a[0] == b[0] && a[1] == b[1];
  },
  groupCopy: function(a, b) {
    a[0] = b[0];
    a[1] = b[1];
  },
  groupClone: function(a) {
    return [a[0], a[1]];
  },
  draggingOnDisabledAxis: function() {
    return (!this.options.horizontal && Cursor.xDiff > Cursor.yDiff) ||
           (!this.options.vertical && Cursor.yDiff > Cursor.xDiff);
  }
};


var bind = function(fn, context) {
  return function() {
    return fn.apply(context, arguments);
  };
};

// Cross-browser vanilla JS event handling

var addEventListener = function(element, type, callback) {
  if (element.addEventListener) {
    element.addEventListener(type, callback, false);
  } else if (element.attachEvent) {
    element.attachEvent('on' + type, callback);
  }
};

var removeEventListener = function(element, type, callback) {
  if (element.removeEventListener) {
    element.removeEventListener(type, callback, false);
  } else if (element.detachEvent) {
    element.detachEvent('on' + type, callback);
  }
};

var preventEventDefaults = function(e) {
  if (!e) {
    e = window.event;
  }
  if (e.preventDefault) {
    e.preventDefault();
  }
  e.returnValue = false;
};

var stopEventPropagation = function(e) {
  if (!e) {
    e = window.event;
  }
  if (e.stopPropagation) {
    e.stopPropagation();
  }
  e.cancelBubble = true;
};


var Cursor = {

  x: 0,
  y: 0,
  xDiff: 0,
  yDiff: 0,
  refresh: function(e) {
    if (!e) {
      e = window.event;
    }
    if (e.type == 'mousemove') {
      this.set(e);
    } else if (e.touches) {
      this.set(e.touches[0]);
    }
  },
  set: function(e) {
    var lastX = this.x,
        lastY = this.y;
    if (e.clientX || e.clientY) {
      this.x = e.clientX;
      this.y = e.clientY;
    } else if (e.pageX || e.pageY) {
      this.x = e.pageX - document.body.scrollLeft - document.documentElement.scrollLeft;
      this.y = e.pageY - document.body.scrollTop - document.documentElement.scrollTop;
    }
    this.xDiff = Math.abs(this.x - lastX);
    this.yDiff = Math.abs(this.y - lastY);
  }
};


var Position = {
  get: function(obj) {
    var rect = {left: 0, top: 0};
    if (obj.getBoundingClientRect !== undefined) {
      rect = obj.getBoundingClientRect();
    }
    return [rect.left, rect.top];
  }
};


var StylePrefix = {
  transform: getPrefixedStylePropName('transform'),
  perspective: getPrefixedStylePropName('perspective'),
  backfaceVisibility: getPrefixedStylePropName('backfaceVisibility')
};

function getPrefixedStylePropName(propName) {
  var domPrefixes = 'Webkit Moz ms O'.split(' '),
      elStyle = document.documentElement.style;
  if (elStyle[propName] !== undefined) return propName; // Is supported unprefixed
  propName = propName.charAt(0).toUpperCase() + propName.substr(1);
  for (var i = 0; i < domPrefixes.length; i++) {
    if (elStyle[domPrefixes[i] + propName] !== undefined) {
      return domPrefixes[i] + propName; // Is supported with prefix
    }
  }
};

function triggerWebkitHardwareAcceleration(element) {
  if (StylePrefix.backfaceVisibility && StylePrefix.perspective) {
    element.style[StylePrefix.perspective] = '1000px';
    element.style[StylePrefix.backfaceVisibility] = 'hidden';
  }
};

var vendors = ['webkit', 'moz'];
var requestAnimationFrame = window.requestAnimationFrame;
var cancelAnimationFrame = window.cancelAnimationFrame;

for (var x = 0; x < vendors.length && !requestAnimationFrame; ++x) {
  requestAnimationFrame = window[vendors[x] + 'RequestAnimationFrame'];
  cancelAnimationFrame = window[vendors[x] + 'CancelAnimationFrame'] ||
                         window[vendors[x] + 'CancelRequestAnimationFrame'];
}

if (!requestAnimationFrame) {
  requestAnimationFrame = function (callback) {
    return setTimeout(callback, 25);
  };
  cancelAnimationFrame = clearTimeout;
}

return Dragdealer;

}));

/* End of file */
