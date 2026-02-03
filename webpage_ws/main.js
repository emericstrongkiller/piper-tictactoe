let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'wss://i-00c8408c1d93299a9.robotigniteacademy.com/8eeea66b-7520-4eec-a8c9-df37be35782c/rosbridge/',
        connected: false,
        // page content
        menu_title: 'My menu title',
        main_title: 'Main title, from Vue!!',
        // robot variables
        current_robot_pos: 0,
        //perception variables
        video_server: "/roi_image_perception_server",
        viewer: null,
        viewer2: null,
        // slider value
        slider_val: 0,
        slider2_val: 3,
        slider3_val: 0,
        // board state
        game_board_state: [0, 0, 0, 0, 0, 0, 0, 0, 0],
        // game vals
        human_shape: 1,
        robot_shape: 2,
    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // keep alive
            setInterval(() => {
            if (this.connected) {
                this.ros.callOnConnection(() => {})
            }
            }, 30000)   

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                // Camera
                this.setCameras()
                this.setRawCamera()
                // Subscribe to game board
                this.GetGameBoard()
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
                // delete Camera div
                document.getElementById('divCamera').innerHTML = ''
                document.getElementById('divCamera2').innerHTML = ''
            })
        },
        disconnect: function() {
            this.ros.close()
        },
        // MODIFY BASED ON PIPER TOPICS AND CMDS
        goToPos: function(pos_nbr) {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/piper_move_orders',
                messageType: 'std_msgs/msg/Int32MultiArray'
            })
            let message = new ROSLIB.Message({data: [pos_nbr]})
            topic.publish(message)
            current_robot_pos = pos_nbr
            if (pos_nbr == 11){
                this.game_board_state = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            }
        },
        drawShape: function(shape_id) {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/piper_move_orders',
                messageType: 'std_msgs/msg/Int32MultiArray'
            })
            let message;
            if (shape_id == 2){
                message = new ROSLIB.Message({data: [4, shape_id]})
                current_robot_pos = 4
            } else {
                message = new ROSLIB.Message({data: [current_robot_pos, shape_id]})
            }
            topic.publish(message)
        },
        startGame: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/piper_move_orders',
                messageType: 'std_msgs/msg/Int32MultiArray'
            })
            let message = new ROSLIB.Message({data: [4, 2]})
            topic.publish(message)
            // also publish chosen human/robot shapes for this game
            let topic2 = new ROSLIB.Topic({
                ros: this.ros,
                name: '/game_start',
                messageType: 'std_msgs/msg/Int32MultiArray'
            })
            let message2 = new ROSLIB.Message({data: [this.robot_shape]})
            topic2.publish(message2)
                        console.log("Just Published SHAPE: " + this.robot_shape)
        },
        swapShapes() {
            const tmp = this.human_shape
            this.human_shape = this.robot_shape
            this.robot_shape = tmp
        },
        setCameras: function() {
            // destroy previous viewers
            if (this.viewer) {
              document.getElementById("divCamera").innerHTML = ""
              this.viewer = null
            }
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            this.viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 1400,
                height: 1080,
                topic: this.video_server,
                ssl: true,
            })
        },
        setRawCamera: function() {
            // destroy previous viewers
            if (this.viewer2) {
              document.getElementById("divCamera2").innerHTML = ""
              this.viewer2 = null
            }
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            this.viewer2 = new MJPEGCANVAS.Viewer({
                divID: 'divCamera2',
                host: host,
                width: 1400,
                height: 600,
                topic: '/camera1/image_raw',
                ssl: true,
            })
        },
        sendParameter: function(slider, slider2, slider3) {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/perception_param',
                messageType: 'std_msgs/msg/Int32MultiArray'
            })

            let message = new ROSLIB.Message({data: [Number(slider), Number(slider2), Number(slider3)]})
            topic.publish(message)
        },
        GetGameBoard: function() {
        let topic = new ROSLIB.Topic({
            ros: this.ros,
            name: '/take_pic_order_response',
            messageType: 'std_msgs/msg/Int32MultiArray'
        })
        topic.subscribe((message) => {
            console.log('Received game board:', message.data)
            this.game_board_state = message.data
        })
        }
    },
    watch: {
        slider_val(val) {
            // publish param
            this.sendParameter(val, this.slider2_val, this.slider3_val)
            //console.log("slider_val: " + this.slider_val + "\nslider2_val: " + this.slider2_val)
        },
        slider2_val(val) {
            // publish param
            this.sendParameter(this.slider_val, val, this.slider3_val)
            //console.log("slider_val: " + this.slider_val + "\nslider2_val: " + this.slider2_val)
        },
        slider3_val(val) {
            // publish param
            this.sendParameter(this.slider_val, this.slider2_val, val)
        },
        video_server(val) {
            this.setCameras()
        },
    },
    mounted() {
        // page is ready
        console.log('page is ready!')
        this.connect()
    },
})