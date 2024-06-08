const ros = new ROSLIB.Ros();

function connect() {
    ros.connect("ws://rosbridge.web.com/");
    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        mapLoad();
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });
}

let pathPlanning = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/NavfnROS/plan',
    messageType: 'nav_msgs/Path'
});

let pathFeedback = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/feedback',
    messageType: 'move_base_msgs/MoveBaseActionFeedback'
});

function mapLoad() {
    let poseTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/amcl_pose',
        messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });

    var viewer = new ROS2D.Viewer({
        divID: 'map',
        width: 700,
        height: 700,
    });

    let gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        continuous: true,
    });

    gridClient.on('change', function () {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });

    let robotMarker = new ROS2D.ArrowShape({
        size: 0.7,
        strokeSize: 0.01,
        pulse: true,
        strokeColor: '#d70e0e',
        fillColor: createjs.Graphics.getRGB(255, 0, 0, 0.9),
    });

    let pathShape = new ROS2D.PathShape({
        strokeSize: 2,
        strokeColor: createjs.Graphics.getRGB(0, 255, 0, 0.1)
    });

    gridClient.rootObject.addChild(pathShape);

     let traceShape = new ROS2D.TraceShape({
        strokeSize: 0.1,
        strokeColor: createjs.Graphics.getRGB(0, 0, 255, 0.5),
        maxPoses: 250
    });
    
    pathFeedback.subscribe((message) => {
        console.log("pathFeedback -> " + message);
        if (message) {
            traceShape.addPose(message.feedback.base_position.pose);;
        }
    });


    gridClient.rootObject.addChild(traceShape);

    pathPlanning.subscribe((message) => {
        console.log("pathPlanning message received:", message);
        pathShape.setPath(message);
    });

    const createInitialPose = (x, y, orientation) => {
        const initialPose = new ROSLIB.Topic({
            ros: ros,
            name: '/initialpose',
            messageType: 'geometry_msgs/PoseWithCovarianceStamped'
        });

        console.log("initial_pose");

        let posestamped_msg = new ROSLIB.Message({
            header: {
                stamp: {
                    secs: 0,
                    nsecs: 100
                },
                frame_id: "map"
            },
            pose: {
                pose: {
                    position: {
                        x: x,
                        y: y,
                        z: 0.0
                    },
                    orientation: orientation
                },
                covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            }
        });
        initialPose.publish(posestamped_msg);
        console.log("initialPose publish");
    }

    const createGoalPose = (x, y, orientation) => {
        const goalPose = new ROSLIB.Topic({
            ros: ros,
            name: '/move_base_simple/goal',
            messageType: 'geometry_msgs/PoseStamped'
        });

        let posestamped_msg = new ROSLIB.Message({
            header: {
                stamp: {
                    secs: 0,
                    nsecs: 100
                },
                frame_id: "map"
            },
            pose: {
                position: {
                    x: x,
                    y: y,
                    z: 0.0
                },
                orientation: orientation
            }
        });

        goalPose.publish(posestamped_msg);
        console.log("goalPose publish");
    }

    let mouseDown = false;
    let mouseDownPose = {};

    const mouseEventHandler = function (event, mouseState, oPerMode) {
        console.log(oPerMode)
        if (mouseState === 'down') {
            mouseDown = true;
            let mouseDownPosition = viewer.scene.globalToRos(event.stageX, event.stageY);
            let mouseDownPositionVec3 = new ROSLIB.Vector3(mouseDownPosition);
            mouseDownPose = new ROSLIB.Pose({
                position: mouseDownPositionVec3
            });
        } else if (mouseState === 'move' && mouseDown) {
            gridClient.rootObject.removeChild(robotMarker);
        } else if (mouseState === 'up' && mouseDown) {
            mouseDown = false;
            let mouseUpPosition = viewer.scene.globalToRos(event.stageX, event.stageY);
            let mouseUpPositionVec3 = new ROSLIB.Vector3(mouseUpPosition);
            const mouseUpPose = new ROSLIB.Pose({
                position: mouseUpPositionVec3
            });

            let xDelta = mouseUpPose.position.x - mouseDownPose.position.x;
            let yDelta = mouseUpPose.position.y - mouseDownPose.position.y;

            let thetaRadians = Math.atan2(xDelta, yDelta);
            let thetaDegrees = thetaRadians * (180.0 / Math.PI);

            if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
                thetaRadians += (3 * Math.PI / 2);
            } else {
                thetaRadians -= (Math.PI / 2);
            }

            let qz = Math.sin(-thetaRadians / 2.0);
            let qw = Math.cos(-thetaRadians / 2.0);
            let orientation = new ROSLIB.Quaternion({ x: 0, y: 0, z: qz, w: qw });

            if (oPerMode === "initial") {
                createInitialPose(mouseDownPose.position.x, mouseDownPose.position.y, orientation);
            } else if (oPerMode === "goal") {
                createGoalPose(mouseDownPose.position.x, mouseDownPose.position.y, orientation);
            }
           
        }
    };

    viewer.scene.addEventListener('stagemousedown', function (event) {
        console.log("stagemousedown");
        let initialPoseChecked = document.querySelector("#initialPoseswitch").checked;
        let goalPoseChecked = document.querySelector("#goalPoseswitch").checked;
        let oPerMode = initialPoseChecked ? "initial" : "goal";

        if (initialPoseChecked || goalPoseChecked) {
            mouseEventHandler(event, 'down', oPerMode);
        }
    });

    viewer.scene.addEventListener('stagemousemove', function (event) {
        console.log("stagemousemove");
        let initialPoseChecked = document.querySelector("#initialPoseswitch").checked;
        let goalPoseChecked = document.querySelector("#goalPoseswitch").checked;
        let oPerMode = initialPoseChecked ? "initial" : "goal";

        if (initialPoseChecked || goalPoseChecked) {
            mouseEventHandler(event, 'move', oPerMode);
        }
    });

    viewer.scene.addEventListener('stagemouseup', function (event) {
        console.log("stagemouseup");
        let initialPoseChecked = document.querySelector("#initialPoseswitch").checked;
        let goalPoseChecked = document.querySelector("#goalPoseswitch").checked;
        let oPerMode = initialPoseChecked ? "initial" : "goal";

        if (initialPoseChecked || goalPoseChecked) {
            mouseEventHandler(event, 'up', oPerMode);
        }
    });

    const createFunc = function (handlerToCall, discriminator, robotMarker) {
        return discriminator.subscribe(function (pose) {
            robotMarker.x = pose.pose.pose.position.x;
            robotMarker.y = -pose.pose.pose.position.y;

            let orientationQuerter = pose.pose.pose.orientation;
            let q0 = orientationQuerter.w;
            let q1 = orientationQuerter.x;
            let q2 = orientationQuerter.y;
            let q3 = orientationQuerter.z;
            let degree = -Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / Math.PI;
            robotMarker.rotation = degree;

            gridClient.rootObject.addChild(robotMarker);
        });
    };

    createFunc('subscribe', poseTopic, robotMarker);
}

window.addEventListener('load', connect);
