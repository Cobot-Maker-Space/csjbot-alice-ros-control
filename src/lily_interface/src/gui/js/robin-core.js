
var ros = new ROSLIB.Ros({
    url : 'ws://10.25.132.128:9090'
});

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

var speechText = new ROSLIB.Topic({ 
    ros : ros,
    name : '/speech',
    messageType : 'std_msgs/String'
});

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

console.log('Script running');

