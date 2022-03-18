
ws_url = 'ws://' + window.location.hostname + ':9090';

var ros = new ROSLIB.Ros({
    url : ws_url
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

$('.connection-status').on('click', function(){
    ros.connect(ws_url);
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

var limbMovementPublisher = new ROSLIB.Topic({
    ros : ros,
    name : '/move_joints',
    messageType : 'csjbot_alice/JointMovement',
    latch: true
});

var limbMovementSubscriber = new ROSLIB.Topic({
    ros : ros,
    name : '/move_joints',
    messageType : 'csjbot_alice/JointMovement',
});

var limbResetPublisher = new ROSLIB.Topic({
    ros : ros,
    name : '/reset_joints',
    messageType : 'std_msg/Empty'
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

limbMovementSubscriber.subscribe(function(message) {
    console.log('Received message on ' + limbMovementSubscriber.name);
    console.log(message);

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

$('input[type=range]').on("change", $.throttle(1000, function() { 
    moveLimbs($(this).data('limb'));
}));

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
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular}
    });
    movementPublisher.publish(twist);
}


function moveLimbs(limb_to_move) {
    neck_move = false;
    left_arm_move = false;
    right_arm_move = false;

    switch (limb_to_move){
        case 'neck':
            neck_move = true;
            console.log(limb_to_move + ': ' + parseInt(-$('.neck').val()));
            break;

        case 'left_arm':
            console.log(limb_to_move + ': ' + parseInt($('.left-arm').val()));
            left_arm_move = true;
            break;
    
        case 'right_arm':
            console.log(limb_to_move + ': ' + parseInt(-$('.right-arm').val()));
            right_arm_move = true;
            break;

    }

    var joint_movement = new ROSLIB.Message({
        neck: neck_move,
        neck_to: parseInt(-$('.neck').val()),
        neck_speed: 5000,
        left_arm: left_arm_move,
        left_arm_to: parseInt($('.left-arm').val()),
        left_arm_speed: 3000,
        right_arm: right_arm_move,
        right_arm_to: parseInt(-$('.right-arm').val()),
        right_arm_speed: 3000
    });

    limbMovementPublisher.publish(joint_movement);
}
 
$('.reset-limbs').on('click', function() {
    console.log('Resetting all limbs.');
    limbResetPublisher.publish({});
});


change_voice($('.voice-default').data('voice'));
