
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
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

var cmdPaddleReset = new ROSLIB.Topic({
    ros : ros,
    name : '/paddle_1/reset_centre',
    messageType : 'std_msgs/Bool'
})

var pdl1_reset_msg = new ROSLIB.Message({data:true})
cmdPaddleReset.publish(pdl1_reset_msg)

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/client_count',
    messageType : 'std_msgs/Int32'
});

listener.subscribe(function(message) {
    $('.client_count').html(message.data);
    console.log('Received message on ' + listener.name + ': ' + message.data);
});

var listener_player1_score = new ROSLIB.Topic({
    ros : ros,
    name : '/player_1/score',
    messageType : 'std_msgs/Int32'
});

listener_player1_score.subscribe(function(message) {
    $('.player_1_score').html(message.data);
});

var listener_player2_score = new ROSLIB.Topic({
    ros : ros,
    name : '/player_2/score',
    messageType : 'std_msgs/Int32'
});

listener_player2_score.subscribe(function(message) {
    $('.player_2_score').html(message.data);
});


var battery_paddle_1 = new ROSLIB.Topic({
    ros : ros,
    name : '/paddle_1/battery_state',
    messageType : 'sensor_msgs/BatteryState'
});

battery_paddle_1.subscribe(function(message) {
    $('.battery-paddle-1').html(message.data.percentage);
});


var battery_paddle_2 = new ROSLIB.Topic({
    ros : ros,
    name : '/paddle_2/battery_state',
    messageType : 'sensor_msgs/BatteryState'
});

battery_paddle_2.subscribe(function(message) {
    $('.battery-paddle-2').html(message.data.percentage);
});


var battery_ball = new ROSLIB.Topic({
    ros : ros,
    name : '/ball/battery_state',
    messageType : 'sensor_msgs/BatteryState'
});

battery_ball.subscribe(function(message) {
    $('.battery-ball').html(message.data.percentage);
});
