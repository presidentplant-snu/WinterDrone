// Controller state
const droneState = {
	armed: false,
	throttle: 0,   // 0-100%
	yaw: 0,        // -180 to 180 degrees
	pitch: 0,      // -90 to 90 degrees
	roll: 0,       // -90 to 90 degrees
	telemetryInterval: 40, // TODO add separate logic to receive pitch/roll/battery from drone, rename telemetry to control or whatever
	telemetryIntervalId: null
};

// DOM Elements
const controllerValues = document.getElementById('controller-values');
const telemetryLog = document.getElementById('telemetry-log');
const telemetryIntervalInput = document.getElementById('telemetry-interval');
const chevron = document.querySelector('.chevron');

const socket = new WebSocket("ws://192.168.4.1/ws");
// Joystick elements
const stick1 = document.getElementById('stick1');
const joystick1 = document.getElementById('joystick1');
const stick2 = document.getElementById('stick2');
const joystick2 = document.getElementById('joystick2');

/* ARMING SWITCH */
// Handle arming switch, always disarm at the beginning
const armingSwitch = document.getElementById('arming-switch');
const armingStatus = document.getElementById('arming-status');

armingSwitch.checked = false;
armingStatus.textContent = 'DISARMED';
armingStatus.classList.remove('armed');
armingSwitch.addEventListener('change', function() {
	droneState.armed = this.checked;

	if (droneState.armed) {
		armingStatus.textContent = 'ARMED';
		armingStatus.classList.add('armed');

		const packet = new Uint8Array(1);
  		packet[0] = 0x00; // ARM command
		socket.send(packet);

		// Start telemetry when armed
		startTelemetry();

		// Log arming event
		addLogEntry('Drone ARMED');
	} else {
		armingStatus.textContent = 'DISARMED';
		armingStatus.classList.remove('armed');

		const packet = new Uint8Array(1);
  		packet[0] = 0x01; // ARM command
		socket.send(packet);
		// Stop telemetry when disarmed
		stopTelemetry();

		// Log disarming event
		addLogEntry('Drone DISARMED');
	}

	// Update controller values display
	updateControllerValues();
});



// Toggle telemetry section
const telemetryToggle = document.getElementById('telemetry-toggle');
const telemetryContent = document.getElementById('telemetry-content');
telemetryToggle.addEventListener('click', function() {
	telemetryContent.classList.toggle('expanded');
	chevron.classList.toggle('up');
});

/* TELEMETRY */
// Function to send telemetry data
function sendTelemetry() {
	// Create telemetry data object
	const telemetryData = {
		timestamp: new Date().toISOString(),
		armed: droneState.armed,
		throttle: droneState.throttle,
		yaw: droneState.yaw,
		pitch: droneState.pitch,
		roll: droneState.roll
	};

  const packet = new Int8Array(5); // 1 byte command + 4 bytes of joystick data
  packet[0] = 0x02; // JOYSTICK command
  packet[1] = droneState.throttle;
  packet[2] = droneState.yaw;
  packet[3] = droneState.pitch;
  packet[4] = droneState.roll;
  socket.send(packet);
}

// Function to start telemetry
function startTelemetry() {
	if (droneState.telemetryIntervalId) {
		stopTelemetry();
	}

	// Send initial telemetry immediately
	sendTelemetry();

	// Set up interval for regular telemetry
	droneState.telemetryIntervalId = setInterval(sendTelemetry, droneState.telemetryInterval);

	addLogEntry(`Telemetry started (${droneState.telemetryInterval}ms interval)`);
}

// Function to stop telemetry
function stopTelemetry() {
	if (droneState.telemetryIntervalId) {
		clearInterval(droneState.telemetryIntervalId);
		droneState.telemetryIntervalId = null;

		addLogEntry('Telemetry stopped');
	}
}

// Handle telemetry interval change
telemetryIntervalInput.addEventListener('change', function() {
	const value = parseInt(this.value);
	if (value >= 10) {
		droneState.telemetryInterval = value;
		addLogEntry(`Telemetry interval set to ${value}ms`);
	}
});


// Joystick handling functions
function setupJoystick(stickElement, joystickElement, updateFunction) {
	let active = false;
	let joystickRect, joystickRadius, stickRadius, maxDistance;

	function updateJoystickDimensions() {
		joystickRect = joystickElement.getBoundingClientRect();
		joystickRadius = joystickRect.width / 2;
		stickRadius = stickElement.offsetWidth / 2;
		maxDistance = joystickRadius - stickRadius;
	}

	// Update dimensions initially and on window resize
	updateJoystickDimensions();
	window.addEventListener('resize', updateJoystickDimensions);

	// Mouse events
	stickElement.addEventListener('mousedown', startDrag);
	document.addEventListener('mousemove', drag);
	document.addEventListener('mouseup', endDrag);

	// Touch events
	stickElement.addEventListener('touchstart', startDrag);
	document.addEventListener('touchmove', drag);
	document.addEventListener('touchend', endDrag);

	function startDrag(e) {

		updateJoystickDimensions(); // Update dimensions on each start
		active = true;
		stickElement.style.transition = '0s';
		e.preventDefault();
	}

	function drag(e) {
		if (!active) return;

		e.preventDefault();

		const joystickRect = joystickElement.getBoundingClientRect();
		const centerX = joystickRect.left + joystickRect.width / 2;
		const centerY = joystickRect.top + joystickRect.height / 2;

		// Get position either from mouse or touch
		for(touch of e.touches){
		let clientX, clientY;
		if (e.type === 'touchmove') {
			clientX = touch.clientX;
			clientY = touch.clientY;
		} else {
			clientX = e.clientX;
			clientY = e.clientY;
		}

		// Calculate distance from center
		let deltaX = clientX - centerX;
		let deltaY = clientY - centerY;

		// Calculate distance from center (Pythagorean theorem)
		const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

		// If the distance is greater than the max distance, scale it down
	 if (distance > 3*maxDistance) {
		 	console.log("contiuning");
			continue;
		}
		else if (distance > maxDistance) {
			const factor = maxDistance / distance;
			deltaX *= factor;
			deltaY *= factor;
		}
		// Move the stick (accounting for the initial -50%, -50% transform)
		stickElement.style.transform = `translate(calc(-50% + ${deltaX}px), calc(-50% + ${deltaY}px))`;

		// Calculate values based on position
		// Map X from -maxDistance to maxDistance => -100 to 100
		const xPercent = (deltaX / maxDistance) * 100;
		// Map Y from -maxDistance to maxDistance => -100 to 100
		// Note: Y is inverted in the DOM, so we negate it
		const yPercent = -(deltaY / maxDistance) * 100;

		// Update the state based on which joystick is being used
		updateFunction(xPercent, yPercent);

		// Update display
		updateControllerValues();
		}
	}

	function endDrag(e) {
		if (!active) return;
	
		if (e.type === 'touchend') {
			console.log(e.changedTouches[0]);
			if(e.changedTouches[0].target != stickElement){
				return;
			}
		}

		stickElement.style.transition = '0.2s';
		// Don't reset throttle for stick1
		if(e.changedTouches[0].target.id == "stick1"){
		console.log(`translate(-50%,calc(-50%+${-(droneState.throttle*2 - 100)/100 * maxDistance}px))`);
		stickElement.style.transform = `translate(-50%, calc(-50% + ${-(droneState.throttle*2 - 100)/100 * maxDistance}px))`;
		updateFunction(0, NaN);
		}
		else{
		stickElement.style.transform = 'translate(-50%, -50%)';
		updateFunction(0, 0);
		}

		// Update display
		updateControllerValues();
		active = false;
	}
}

// Function to update left joystick values (throttle/yaw)
function updateLeftJoystick(x, y) {
	// X controls yaw rate (-100 to 100 percent)
	droneState.yaw = Math.round(x); 
	
	if(!isNaN(y)){ droneState.throttle = Math.round((y + 100) / 2);}
	// Y controls throttle (0 to 100%)
	// Map Y from -100 to 100 => 0 to 100
	// When stick is at bottom (y = -100), throttle = 0
	// When stick is at top (y = 100), throttle = 100
}

// Function to update right joystick values (pitch/roll)
function updateRightJoystick(x, y) {
	// X controls roll (-90 to 90 degrees)
	droneState.roll = Math.round(x * 0.3); // Scale -100 to 100 to -90 to 90

	// Y controls pitch (-90 to 90 degrees)
	droneState.pitch = Math.round(y * 0.3); // Scale -100 to 100 to -90 to 90
}

// Function to reset joysticks to center position
function resetJoysticks() {
	stick1.style.transform = 'translate(-50%, -50%)';
	stick2.style.transform = 'translate(-50%, -50%)';

	// Reset values
	droneState.throttle = 0;
	droneState.yaw = 0;
	droneState.pitch = 0;
	droneState.roll = 0;

	updateControllerValues();
}

// Function to update the controller values display
function updateControllerValues() {
	controllerValues.textContent = `Throttle: ${droneState.throttle}%, Yaw: ${droneState.yaw}%, Pitch: ${droneState.pitch}°, Roll: ${droneState.roll}°`;
}

// Function to add entry to telemetry log
function addLogEntry(message) {
	const entry = document.createElement('div');
	entry.className = 'telemetry-entry';

	const timestamp = new Date().toISOString().substr(11, 12);
	entry.textContent = `[${timestamp}] ${message}`;

	telemetryLog.appendChild(entry);
	telemetryLog.scrollTop = telemetryLog.scrollHeight;

	// Keep only the last 50 entries
	while (telemetryLog.children.length > 50) {
		telemetryLog.removeChild(telemetryLog.firstChild);
	}
}


// Initialize joysticks
setupJoystick(stick1, joystick1, updateLeftJoystick);
setupJoystick(stick2, joystick2, updateRightJoystick);

// Prevent scrolling when touching the controller area
document.querySelector('.joystick-container').addEventListener('touchmove', function(e) {
	e.preventDefault();
}, { passive: false });

// Initial update of controller values
updateControllerValues();

// Initial log entry
addLogEntry('Drone controller initialized');

