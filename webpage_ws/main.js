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
    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                // Camera
                this.setCamera()
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
        setCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 320,
                height: 240,
                topic: '/camera/D435/color/image_raw',
                ssl: true,
            })
        },
    },
    mounted() {
        // page is ready
        console.log('page is ready!')
    },
})