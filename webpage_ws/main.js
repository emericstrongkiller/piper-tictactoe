let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'wss://i-02d1dc2e3ad1472a8.robotigniteacademy.com/3ae217f3-b080-459a-af01-f524c269e0b0/rosbridge/',
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
        },
        drawShape: function(shape_id) {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/piper_move_orders',
                messageType: 'std_msgs/msg/Int32MultiArray'
            })
            let message = new ROSLIB.Message({data: [current_robot_pos,shape_id]})
            topic.publish(message)
        },
        setCameras: function() {
            // destroy previous viewers
            if (this.viewer) {
              document.getElementById("divCamera").innerHTML = ""
              this.viewer = null
            }
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
                width: 1920,
                height: 1080,
                topic: '/camera/D435/color/image_raw',
                ssl: true,
            })
            this.viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 1400,
                height: 1080,
                topic: this.video_server,
                //topic: '/camera1/image_raw',
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
    },
})