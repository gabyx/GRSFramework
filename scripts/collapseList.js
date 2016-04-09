/**************************************************************/
/* Prepares the cv to be dynamically expandable/collapsible   */
/**************************************************************/
function prepareList(listSel) {
    $(listSel).find('li:has(ul)')
    .click( function(event) {
        console.log("traget:", event.target);
        console.log("t.p:", $(event.target).parent()[0]);
        console.log("this:",$(this)[0]);
       if ( $(event.target).parent()[0] == this){
             $(this).toggleClass('expanded');
             $(this).children('ul').toggle('medium');
       }
        
       return true;
    })
    .addClass('collapsed')
    .children('ul').hide();

    //Create the button funtionality
    $('#toc-button-expand')
    .unbind('click')
    .click( function() {
        $('.collapsed').addClass('expanded');
        $('.collapsed').children().show('medium');
    })
    $('#toc-button-collapse')
    .unbind('click')
    .click( function() {
        $('.collapsed').removeClass('expanded');
        $('.collapsed').children('ul').hide('medium');
    })
    
};