socket_url = {
    url : 'ws://' + window.location.hostname + ':9090'
};

var ros = new ROSLIB.Ros(socket_url);

ros.on('connection', function() {
    console.log('Connected to websocket server.');
    $('.connection-status').addClass('badge-success')
    $('.connection-status').removeClass('badge-danger')
    $('.connection-status').html('Connected');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    $('.connection-status').addClass('badge-danger')
    $('.connection-status').removeClass('badge-success')
    $('.connection-status').html('Offline');
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    $('.connection-status').addClass('badge-danger')
    $('.connection-status').removeClass('badge-success')
    $('.connection-status').html('Offline');
});

$('.connection-status').on('click', function(){
    ros.connect(socket_url);
});

var speechText = new ROSLIB.Topic({
    ros : ros,
    name : '/speech',
    messageType : 'std_msgs/String'
});

var jointsTest = new ROSLIB.Topic({
    ros : ros,
    name : '/reset_joints',
    messageType : 'std_msgs/Empty'
});

jointsTest.publish(new ROSLIB.Message({}));

var speechVoiceName = new ROSLIB.Topic({
    ros : ros,
    name : '/speech_settings/voice_name',
    messageType : 'std_msgs/String'
});

var movementPublisher = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});

var videoSubscriber = new ROSLIB.Topic({
    ros : ros,
    name : '/video_on',
    messageType : 'std_msgs/Bool'
});

videoSubscriber.subscribe(function(message) {
    console.log('Received message on ' + videoSubscriber.name + ': ' + message.data);
    if (message.data == true) {
        console.log('read as true')
        $('.visuals').removeClass('alert-danger');
        $('.visuals').addClass('alert alert-success');
    } else { 
        $('.visuals').removeClass('alert-success');
        $('.visuals').addClass('alert alert-danger');
    }
});

$('.publish-speech').on('click', function(){
    speak($('.custom-text-to-speak').val());
});

$('.quick-speak, .long-speak').on('click', function(){
    speak($(this).html());
});

$('.movement').on('click', function() {
    move($(this).data('linear'), $(this).data('angular'));
});

$(".voice-option").click(function(){
    change_voice($(this).data('voice'));
});

$(".appendix-movement").on("input change", function() { 
    //Code here to dig into Arduino movement.
});

$(".reset-limbs").on("input change", function() { 
    //Code here to publish to joint reset topic
});

function speak(message) {
    speechText.publish(new ROSLIB.Message({data:message}));
}

function change_voice(voicename) {
    speechVoiceName.publish(new ROSLIB.Message({data:voicename}))
}

function move(linear, angular) {
    if(linear > 0.5) linear = 0.5;
    if(linear < -0.5) linear = -0.5;
    if(angular > 1) angular = 1;
    if(angular < -1) angular = -1;

    var twist = new ROSLIB.Message({
        linear: {
            x: linear,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: angular
        }
    });
    movementPublisher.publish(twist);
}

change_voice($('.voice-default').data('voice'));
