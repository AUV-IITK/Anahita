// Connecting to ROS -----------------
  var connection = false;
  console.log("JS File opened");
  var ip, x=0, yaw=0, sway=0, surge=0;

  document.getElementById("ip_input").addEventListener('submit', function(event) {
    event.preventDefault();
    console.log("Function button submitted");
    ip = document.getElementById("ip_submit");
    console.log("Function button submitted, string entered = " + ip.value);
    makeConnection();
  });

  //Establishing Connection 
  
  function makeConnection()
  {

    var ros = new ROSLIB.Ros({
      url : 'ws://' + ip.value
    });

    ros.on('connection', function() {
      console.log('Connected to websocket server.');
      document.getElementById("INFO").textContent="Connected to Socket";
    });

    ros.on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
      document.getElementById("INFO").textContent="Error";
    });

    ros.on('close', function() {
      console.log('Connection to websocket server closed.');
    });

  // Subscribing to a Topic
    // ----------------------
  
    var pwm_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/pwm',
      messageType : 'anahita_msgs/Thrust'
    });

   var forces_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/anahita/thruster_manager/input',
      messageType : 'geometry_msgs/Wrench'
    });

    pwm_listener.subscribe(function(message){
      console.log("Successfully getting new PWM");
      document.getElementById("pwm.fl").textContent=message.forward_left;
      document.getElementById("pwm.fr").textContent=message.forward_right;
      document.getElementById("pwm.sf").textContent=message.sideward_front
      document.getElementById("pwm.sb").textContent=message.sideward_back;
      document.getElementById("pwm.une").textContent=message.upward_north_east;
      document.getElementById("pwm.unw").textContent=message.upward_north_west;
      document.getElementById("pwm.use").textContent=message.upward_south_east;
      document.getElementById("pwm.usw").textContent=message.upward_south_west;
    });

    var Thrust = new ROSLIB.Message({
      sideward_front : 0,
      sideward_back : 0,
      forward_right : 0,
      forward_left : 0,
      upward_north_east : 0,
      upward_north_west : 0,
      upward_south_east : 0,
      upward_south_west : 0   
    });

    var Forces = new ROSLIB.Message({
      "force": {x : 0, y: 0, z: 0},
      "torque":{x : 0, y: 0, z: 0}
  });


    //////////////////////////////////////////////////////////////////////////

    //PARAMETER INPUT FROM TEXT INPUT

    document.getElementById("pwm_ForwardLeft").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_ForwardLeft_input =  document.getElementById("pwm_ForwardLeft_submit").value;
      Thrust.forward_left = Number(pwm_ForwardLeft_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_FL: " + pwm_ForwardLeft_input);  
      document.getElementById("pwm_ForwardLeft_submit").value="";
    });

    document.getElementById("pwm_ForwardRight").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_ForwardRight_input =  document.getElementById("pwm_ForwardRight_submit").value;
      Thrust.forward_right = Number(pwm_ForwardRight_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_FR: " + pwm_ForwardRight_input);  
      document.getElementById("pwm_ForwardRight_submit").value="";
    });

    document.getElementById("pwm_SidewardFront").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_SidewardFront_input =  document.getElementById("pwm_SidewardFront_submit").value;
      Thrust.sideward_front = Number(pwm_SidewardFront_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_SF: " + pwm_SidewardFront_input);  
      document.getElementById("pwm_SidewardFront_submit").value="";
    });
    document.getElementById("pwm_SidewardBack").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_SidewardBack_input =  document.getElementById("pwm_SidewardBack_submit").value;
      Thrust.sideward_back = Number(pwm_SidewardBack_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_SB: " + pwm_SidewardBack_input);  
      document.getElementById("pwm_SidewardBack_submit").value="";
    });
    document.getElementById("pwm_UpwardNE").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_UpwardNE_input =  document.getElementById("pwm_UpwardNE_submit").value;
      Thrust.upward_north_east = Number(pwm_UpwardNE_input);
      pwm_listener.publish(Thrust)
      console.log("Setting the value of /pwm_UNE: " + pwm_UpwardNE_input);  
      document.getElementById("pwm_UpwardNE_submit").value="";
    });
    document.getElementById("pwm_UpwardNW").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_UpwardNW_input =  document.getElementById("pwm_UpwardNW_submit").value;
      Thrust.upward_north_west = Number(pwm_UpwardNW_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_UNW: " + pwm_UpwardNW_input);  
      document.getElementById("pwm_UpwardNW_submit").value="";
    });
    document.getElementById("pwm_UpwardSE").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_UpwardSE_input =  document.getElementById("pwm_UpwardSE_submit").value;
      Thrust.upward_south_east = Number(pwm_UpwardSE_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_USE: " + pwm_UpwardSE_input);  
      document.getElementById("pwm_UpwardSE_submit").value="";
    });
    document.getElementById("pwm_UpwardSW").addEventListener('submit', function(event) {
      event.preventDefault();
      pwm_UpwardSW_input =  document.getElementById("pwm_UpwardSW_submit").value;
      Thrust.upward_south_west = Number(pwm_UpwardSW_input);
      pwm_listener.publish(Thrust);
      console.log("Setting the value of /pwm_USW: " + pwm_UpwardSW_input);  
      document.getElementById("pwm_UpwardSW_submit").value="";
    });
  //}
    document.getElementById("reset_bt").onclick =function(){myfunction()};
    
    function myfunction(){
      Thrust.forward_left =0;
      Thrust.forward_right=0;
      Thrust.sideward_front=0;
      Thrust.sideward_back=0;
      Thrust.upward_north_east=0;
      Thrust.upward_north_west=0;
      Thrust.upward_south_east=0;
      Thrust.upward_south_west=0;
      pwm_listener.publish(Thrust);
      console.log("Reset successful");
    }

console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");
	
			var joystick	= new VirtualJoystick({
				container	: document.getElementById('container'),
				mouseSupport	: true,
			});
			joystick.addEventListener('touchStart', function(){
				console.log('down')
			})
			joystick.addEventListener('touchEnd', function(){
				console.log('up')
			})
			setInterval(function(){
				var outputEl	= document.getElementById('yaw');
				outputEl.innerHTML	= '<b>Result:</b> '
					+ ' dx:'+joystick.deltaX()
					+ ' dy:'+joystick.deltaY()
					+ (joystick.right()	? ' right'	: '')
					+ (joystick.up()	? ' up'		: '')
					+ (joystick.left()	? ' left'	: '')
					+ (joystick.down()	? ' down' 	: '');
			        Forces.force.x = joystick.deltaX();
			        Forces.force.y = joystick.deltaY();
				forces_listener.publish(Forces);
			}, 1/30 * 1000);
  }

	

    ////////////////////////////////////////////////////////////////////////
  
   /*
    document.getElementById("forward").addEventListener('click', function(event){
      console.log("Pressed Forward");
      event.preventDefault();
      surge=surge+20;
      pwm_surge_param.set(surge);
      console.log("Setting the value of /pwm_surge: 50");       
      document.getElementById("INFO").textContent="Moving forward"; 
    });


    document.getElementById("reverse").addEventListener('click', function(event){
      console.log("Pressed Reverse");
      event.preventDefault();
      surge=surge-20;
      pwm_surge_param.set(surge);
      console.log("Setting the value of /pwm_surge: -50");
      document.getElementById("INFO").textContent="Moving back";  
    });


    document.getElementById("left").addEventListener('click', function(event){
      console.log("Pressed Left");
      event.preventDefault();
      sway=sway-20;
      pwm_sway_param.set(sway);
      console.log("Setting the value of /pwm_sway:  -50");
      document.getElementById("INFO").textContent="Moving left";  
    });


    document.getElementById("right").addEventListener('click', function(event){
      console.log("Pressed Right");
      event.preventDefault();
      sway=sway+20;
      pwm_sway_param.set(sway);
      console.log("Setting the value of /pwm_sway: 50");  
      document.getElementById("INFO").textContent="Moving right";  
    });


    document.getElementById("turn_cw").addEventListener('click', function(event){
      console.log("Pressed CW:");
      event.preventDefault();
      yaw=yaw+20;
      pwm_turn_param.set(yaw);
      console.log("Setting the value of /pwm_turn: 50");  
      document.getElementById("INFO").textContent="Turning CW";
    });


    document.getElementById("turn_acw").addEventListener('click', function(event){
      console.log("Pressed:");
      event.preventDefault(); 
      yaw=yaw-20;
      pwm_turn_param.set(yaw);
      console.log("Setting the value of /pwm_turn: -50");
      document.getElementById("INFO").textContent="Turning ACW";  
    });

    document.getElementById("kill").addEventListener('click', function(event){
      console.log("Pressed:");
      x=0; 
      sway=0;
      surge=0;
      yaw=0;
      event.preventDefault();
      kill_param.set(true);
      console.log("Setting the kill to true");
      document.getElementById("INFO").textContent="KILLED";  
    });

    document.getElementById("heave_up").addEventListener('click', function(event){
      x=x-10;      
      console.log("Pressed:");
      event.preventDefault();
      pwm_heave_param.set(x);
      console.log("Setting the kill to true");
      document.getElementById("INFO").textContent="Heaving Up";  
    });

   document.getElementById("heave_down").addEventListener('click', function(event){
      x=x+10;      
      console.log("Pressed:");
      event.preventDefault();
      pwm_heave_param.set(x);
      console.log("Setting the kill to true");
      document.getElementById("INFO").textContent="Heaving Down";  
    });
  */
      /////////////////////////////////////////////////////////////////////////////////////////////

    /*var pwm_sway_input, pwm_heave_input, pwm_surge_input, pwm_roll_input, pwm_pitch_input, pwm_turn_input;
   
    var x_coordinate_listener =  new ROSLIB.Topic({
      ros : ros,
      name : '/anahita/x_coordinate',
      messageType : 'std_msgs/Float32'
    });
    var y_coordinate_listener =  new ROSLIB.Topic({
      ros : ros,
      name : '/anahita/y_coordinate',
      messageType : 'std_msgs/Float32'
    });
    var z_coordinate_listener =  new ROSLIB.Topic({
      ros : ros,
      name : '/anahita/z_coordinate',
      messageType : 'std_msgs/Float32'
    });

    var yaw_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/mavros/imu/yaw',
      messageType : 'std_msgs/Float32'
    });

    var roll_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/mavros/imu/roll',
      messageType : 'std_msgs/Float32'
    });

    var pitch_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/mavros/imu/pitch',
      messageType : 'std_msgs/Float32'
    });

    var pwm_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/pwm',
      messageType : 'anahita_msgs/Thrust'
    });

   var pressure_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/depth',
      messageType : 'std_msgs/Float32'
    });

    var image_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/front_camera/image_raw',
      messageType : 'sensor_msgs/Image'
    });
    
    pressure_listener.subscribe(function(message){document.getElementById("depth").textContent=Number.parseFloat(message.data).toPrecision(7);});
    roll_listener.subscribe(function(message){document.getElementById("roll").textContent=Number.parseFloat(message.data).toPrecision(7);});
    yaw_listener.subscribe(function(message){document.getElementById("yaw").textContent=Number.parseFloat(message.data).toPrecision(7);});
    pitch_listener.subscribe(function(message){document.getElementById("pitch").textContent=Number.parseFloat(message.data).toPrecision(7);});
    
    x_coordinate_listener.subscribe(function(message){document.getElementById("x_coordinate").textContent=Number.parseFloat(message.data).toPrecision(7);});
    y_coordinate_listener.subscribe(function(message){document.getElementById("y_coordinate").textContent=Number.parseFloat(message.data).toPrecision(7);});
    z_coordinate_listener.subscribe(function(message){document.getElementById("z_coordinate").textContent=Number.parseFloat(message.data).toPrecision(7);});

    pwm_listener.subscribe(function(message){
      console.log("Successfully getting new PWM");
      document.getElementById("pwm.fl").textContent=message.forward_left;
      document.getElementById("pwm.fr").textContent=message.forward_right;
      document.getElementById("pwm.ns").textContent=message.sideward_front
      document.getElementById("pwm.ss").textContent=message.sideward_back;
      document.getElementById("pwm.une").textContent=message.upward_north_east;
      document.getElementById("pwm.unw").textContent=message.upward_north_west;
      document.getElementById("pwm.use").textContent=message.upward_south_east;
      document.getElementById("pwm.usw").textContent=message.upward_south_west;
    });
  */


     ///////////////////////////////////////////////////////////////////////////////

   /* image_listener.subscribe(function(message){
	console.log("Getting a new image");
	var c = document.getElementById("myCanvas"); 
         var ctx = c.getContext("2d");
         var img = message;
         ctx.drawImage(img, 10, 10);
    });


    pwm_sway_param = new ROSLIB.Param({
      ros : ros,
      name : '/pwm_sway',
      messageType : 'int'
    });
    pwm_heave_param = new ROSLIB.Param({
      ros : ros,
      name : '/pwm_heave',
      messageType : 'int'
    });
    pwm_surge_param = new ROSLIB.Param({
      ros : ros,
      name : '/pwm_surge',
      messageType : 'int'
    });
    pwm_roll_param = new ROSLIB.Param({
      ros : ros,
      name : '/pwm_roll',
      messageType : 'int'
    });
    pwm_turn_param = new ROSLIB.Param({
      ros : ros,
      name : '/pwm_yaw',
      messageType : 'int'
    });
    pwm_pitch_param = new ROSLIB.Param({
      ros : ros,
      name : '/pwm_pitch',
      messageType : 'int'
    });
    kill_param = new ROSLIB.Param({
      ros: ros,
      name: '/kill_signal',
      messageType : 'bool'
    });
  */
      /////////////////////////////////////////////////////////////////////////  
