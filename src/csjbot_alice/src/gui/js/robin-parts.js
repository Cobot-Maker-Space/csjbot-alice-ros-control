$( document ).ready(function() {

    $('.parts-db a').click(function (e) {
        e.preventDefault();
        $(this).tab('show');
    })

    $( document ).ready(function() {
        load_parts_from_file();

        $(document).on('click', '.btn-parts-action', function(){
            transfer_part(this);
        });
    });

    $(document).on('click', '.btn-confirm-parts', function(){
        var parts_speech = "I have the following parts on my tray. "
        $('#in-transit').find('li').each(function() {
            parts_speech += $(this).find('.parts-name').html() + ". ";
        });

        console.log(parts_speech)
        // speak(speech).val();
    });

});

function load_parts_from_file() {
    $.getJSON("db/parts.json", function(json) {
        json.sort((a, b) => {
          return a.name.localeCompare(b.name);
        });
        json.forEach(function(obj) {
            for (i=0; i < obj.qty; i++) {
                add_part(obj);
            }
        });
    }).done(function(){
        update_counts();
    });
}

function add_part(part) {
    part_obj = $( ".templates .part" ).clone()
    part_obj.find('.parts-name').html(part.name)
    part_obj.find('[data-btntype="warehouse"]').addClass('disabled')
    part_obj.appendTo( "#warehouse ul" );
}

function update_counts() {
    $('.count').each(function(){
        count = $('#' + $(this).data('type') + ' ul').find('li').length;
        $(this).html(count);
    });
}

function transfer_part(transfer_button) {
    type = $(transfer_button).data('btntype');
    part = $(transfer_button).parents('.part');
    part.detach().appendTo("#" + type + " ul");
    part.data('type', type)
    setup_transfer_buttons(part);
    update_counts();
}

function setup_transfer_buttons(part) {
    type = $(part).parents('.parts-list').data('type');
    $(part).find('.btn-parts-action').removeClass('disabled');
    $(part).find('[data-btntype="' + type + '"]').addClass('disabled');
}
