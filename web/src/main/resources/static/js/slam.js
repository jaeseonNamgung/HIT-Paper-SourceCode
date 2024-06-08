const ros = new ROSLIB.Ros();

function connect() {
    ros.connect("ws://rosbridge.web.com/");
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        mapload()
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });
}
function mapload() {
    var viewer = new ROS2D.Viewer({
        divID: 'map',
        width: 700,
        height: 700
    });

    let gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        continuous: true
    });

    gridClient.on('change', function () {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x,gridClient.currentGrid.pose.position.y);
    });

    let robotMarker = new ROS2D.ArrowShape({
        size: 0.5,
        strokeSize: 0.01,
        pulse: true,
        strokeColor: '#d70e0e',
        fillColor: createjs.Graphics.getRGB(255, 0, 0, 0.9),
    });

    // Robot odometry topic
    let odomTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });

    odomTopic.subscribe(function(message) {
        console.log("SLAM operation is active");
        let baseLinkPose = message.pose.pose.position;  // Accessing position from the Odometry message
        let rotation = message.pose.pose.orientation;  // Accessing quaternion rotation

        // Update the robot's marker position
        robotMarker.x = baseLinkPose.x;
        robotMarker.y = -baseLinkPose.y;

        // Convert quaternion to Euler angles to get yaw in degrees
        let degreeZ = quaternionToEuler(rotation).z;

        // Update robotMarker rotation
        robotMarker.rotation = -degreeZ;

        function quaternionToEuler(quaternion) {
            // Assuming the quaternion is normalized
            let sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
            let cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
            let roll = Math.atan2(sinr_cosp, cosr_cosp);

            let sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
            let pitch = Math.asin(sinp);

            let siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
            let cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
            let yaw = Math.atan2(siny_cosp, cosy_cosp);

            // Converting yaw angle to degrees
            yaw = yaw * (180 / Math.PI);

            // Return all three in an object
            return {
                x: roll,
                y: pitch,
                z: yaw
            };
        }
        viewer.scene.addChild(robotMarker);
    });

    const cmdVelTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    document.addEventListener('keydown', function(event) {
        switch (event.key) {
            case 'ArrowUp':
                moveRobot(0.5, 0); // Move forward
                break;
            case 'ArrowDown':
                moveRobot(-0.5, 0); // Move backward
                break;
            case 'ArrowLeft':
                moveRobot(0, 0.5); // Turn left
                break;
            case 'ArrowRight':
                moveRobot(0, -0.5); // Turn right
                break;
        }
    });

    function moveRobot(linear, angular) {
        var twist = new ROSLIB.Message({
            linear : {
                x : linear,
                y : 0,
                z : 0
            },
            angular : {
                x : 0,
                y : 0,
                z : angular
            }
        });
        cmdVelTopic.publish(twist);
    }

    document.addEventListener('keyup', function(event) {
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
            moveRobot(0, 0); // Stop the robot
        }
    });
}
window.addEventListener('onload',
    connect()
)