
var ros = new ROSLIB.Ros({
    url : 'ws://10.25.132.128:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

var speechText = new ROSLIB.Topic({ 
    ros : ros,
    name : '/speech',
    messageType : 'std_msgs/String'
});

$('.publish-speech').on('click', function(){
    speak($('.custom-text-to-speak').val());
});

$('.quick-speak').on('click', function(){
    speak($(this).html());
});

function speak(message) {
    speechText.publish(new ROSLIB.Message({data:message}));
}

console.log('Script running');

