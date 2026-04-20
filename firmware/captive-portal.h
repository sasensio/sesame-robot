#pragma once

#include <Arduino.h>

// ======================================================================
// --- WEB INTERFACE HTML ---
// ======================================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Sesame Access Point Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root {
      --content-color: #ff8c42;
      --content-color-dark: #e67a30;
      --content-color-darker: #cc6b29;
      --content-color-glow: rgba(255, 140, 66, 0.3);
    }
    
    * {
      user-select: none;
      -webkit-user-select: none;
      -webkit-touch-callout: none;
    }
    
    body { 
      font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; 
      text-align: center; 
      background: linear-gradient(135deg, #0a0a0a, #1a1a2e);
      color: #e0e0e0; 
      touch-action: manipulation; 
      margin: 0;
      padding: 10px;
      overflow-x: hidden;
      box-sizing: border-box;
    }
    
    h2 { 
      margin: 10px 0 20px 0;
      color: #fff;
      font-size: 32px;
      font-weight: 600;
      text-shadow: 0 2px 4px rgba(0,0,0,0.5);
    }
    
    /* Command Queue Status */
    .command-queue {
      font-size: 12px;
      color: #888;
      margin-bottom: 20px;
    }
    .command-queue.full {
      color: #ff6b6b;
      font-weight: bold;
    }
    
    /* Section Containers */
    .sections-container {
      display: flex;
      flex-direction: column;
      gap: 15px;
      max-width: 1400px;
      margin: 0 auto;
    }
    
    .section {
      background: rgba(30, 30, 30, 0.8);
      border: 1px solid #333;
      border-radius: 16px;
      padding: 15px;
      margin: 0 auto;
      width: calc(100% - 20px);
      max-width: 450px;
      box-shadow: 0 4px 20px rgba(0,0,0,0.3);
      box-sizing: border-box;
    }
    
    .section-title {
      font-size: 16px;
      font-weight: 600;
      color: var(--content-color);
      margin: 0 0 15px 0;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    
    /* Button Base Styles */
    button { 
      background: linear-gradient(145deg, #3a3a3a, #2a2a2a);
      border: none; 
      color: #e0e0e0; 
      padding: 15px; 
      font-size: 18px; 
      border-radius: 12px; 
      cursor: pointer; 
      box-shadow: 0 4px 8px rgba(0,0,0,0.3);
      transition: all 0.1s;
      font-weight: 500;
    }
    button:active { 
      box-shadow: 0 2px 4px rgba(0,0,0,0.3);
      transform: translateY(2px); 
    }
    
    /* D-Pad Controls */
    .dpad-container { 
      display: flex; 
      flex-direction: column;
      align-items: center;
      gap: 15px;
      width: 100%;
    }
    .dpad { 
      display: grid; 
      grid-template-columns: repeat(3, 1fr); 
      grid-template-rows: repeat(2, 1fr); 
      gap: 12px;
      width: 100%;
      max-width: 294px;
      aspect-ratio: 3 / 2;
    }
    .dpad button { 
      font-size: 35px; 
      border: 2px solid #555; 
      color: #fff;
      width: 100%;
      height: 100%;
      min-height: 70px;
    }
    .spacer { 
      visibility: hidden; 
    }
    
    /* Pose Grid */
    .grid { 
      display: grid; 
      grid-template-columns: repeat(3, 1fr); 
      gap: 10px; 
    }
    .btn-pose { 
      background: linear-gradient(145deg, var(--content-color), var(--content-color-dark));
      padding: 12px 8px;
      font-size: 15px;
    }
    .btn-pose:active { 
      background: linear-gradient(145deg, var(--content-color-dark), var(--content-color-darker));
    }
    
    /* Special Buttons */
    .btn-stop-all { 
      background: linear-gradient(145deg, #e63946, #c92a35);
      width: 100%; 
      font-size: 20px; 
      padding: 18px; 
      box-shadow: 0 6px 12px rgba(230, 57, 70, 0.4);
      border: 2px solid #ff6b6b;
      color: #fff;
      text-transform: uppercase;
      letter-spacing: 2px;
    }
    .btn-stop-all:active { 
      background: linear-gradient(145deg, #c92a35, #a8222c);
      transform: translateY(3px); 
    }
    
    .btn-settings { 
      background: linear-gradient(145deg, #555, #444);
      padding: 12px 25px;
      font-size: 16px;
    }
    
    /* Motor Controls */
    .lock-indicator {
      font-size: 11px;
      color: #ff6b6b;
      text-align: center;
      margin-top: 5px;
      display: none;
    }
    .lock-indicator.active {
      display: block;
    }
    
    .motor-controls {
      margin-top: 10px;
    }
    .motor-slider {
      margin: 15px 0;
    }
    .motor-slider label {
      font-size: 13px;
      color: #aaa;
      margin-bottom: 4px;
      display: block;
    }
    .slider-wrap {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .slider-wrap input[type="range"] {
      flex: 1;
    }
    .slider-wrap .angle-val {
      min-width: 42px;
      text-align: right;
      color: #fb0;
      font-weight: bold;
      font-size: 14px;
      white-space: nowrap;
    }
    .motor-slider input[type="range"] {
      width: 100%;
      height: 6px;
      background: #333;
      border-radius: 5px;
      outline: none;
      -webkit-appearance: none;
    }
    .motor-slider input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 18px;
      height: 18px;
      background: var(--content-color);
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 2px 6px var(--content-color-glow);
    }
    .motor-slider input[type="range"]::-moz-range-thumb {
      width: 18px;
      height: 18px;
      background: var(--content-color);
      border-radius: 50%;
      cursor: pointer;
      border: none;
      box-shadow: 0 2px 6px var(--content-color-glow);
    }
    .motor-slider input[type="range"]:disabled {
      opacity: 0.5;
      cursor: not-allowed;
    }
    .motor-slider input[type="range"]:disabled::-webkit-slider-thumb,
    .motor-slider input[type="range"]:disabled::-moz-range-thumb {
      background: #666;
      cursor: not-allowed;
    }
    .cal-row {
      display: flex;
      align-items: center;
      gap: 4px;
      margin-top: 4px;
      font-size: 12px;
      color: #aaa;
    }
    .cal-row input[type="number"] {
      width: 54px;
      background: #222;
      border: 1px solid #444;
      color: #eee;
      border-radius: 4px;
      padding: 3px 4px;
      font-size: 12px;
      text-align: center;
    }
    .cal-row .cal-btn, .angle-row .cal-btn {
      background: #333;
      border: 1px solid #555;
      color: #f90;
      border-radius: 4px;
      padding: 2px 6px;
      font-size: 10px;
      cursor: pointer;
    }
    .cal-row .cal-btn:active, .angle-row .cal-btn:active { background: #555; }
    .angle-row {
      display: flex;
      align-items: center;
      gap: 4px;
      margin-top: 2px;
      font-size: 11px;
      color: #8cf;
    }
    .angle-row input[type="number"] {
      width: 64px;
      background: #222;
      border: 1px solid #446;
      color: #8cf;
      border-radius: 4px;
      padding: 4px 6px;
      font-size: 12px;
      text-align: center;
    }
    
    /* Gamepad Status */
    .gamepad-status {
      font-size: 13px;
      padding: 8px 14px;
      border-radius: 10px;
      border: 2px solid #666;
      color: #ccc;
      background: rgba(26, 26, 26, 0.8);
      display: inline-block;
    }
    .gamepad-status.connected {
      border-color: #2ecc71;
      color: #2ecc71;
      background: rgba(46, 204, 113, 0.1);
    }
    
    /* Settings Panel */
    .settings-panel { 
      display: none; 
      position: fixed; 
      top: 0; 
      left: 0; 
      width: 100%; 
      height: 100%; 
      background: rgba(0,0,0,0.9); 
      z-index: 100; 
      backdrop-filter: blur(8px);
      overflow-y: auto;
    }
    .settings-content { 
      background: linear-gradient(145deg, #1e1e1e, #2a2a2a);
      border: 1px solid #444; 
      max-width: 400px; 
      margin: 30px auto; 
      padding: 25px; 
      border-radius: 20px; 
      text-align: left; 
      box-shadow: 0 10px 40px rgba(0,0,0,0.6); 
    }
    .settings-content h3 { 
      color: var(--content-color); 
      margin-top: 0; 
      text-align: center;
      font-size: 24px;
    }
    .settings-section {
      margin: 20px 0;
      padding: 15px;
      background: rgba(0,0,0,0.3);
      border-radius: 10px;
    }
    .settings-section h4 {
      color: var(--content-color);
      margin: 0 0 10px 0;
      font-size: 14px;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    .settings-content label { 
      display: block; 
      margin-top: 12px; 
      font-weight: 500; 
      color: #ccc;
      font-size: 13px;
    }
    .settings-content input, 
    .settings-content select { 
      width: 100%; 
      padding: 10px; 
      margin-top: 5px; 
      background: #333; 
      color: #fff; 
      border: 1px solid #555; 
      border-radius: 8px; 
      box-sizing: border-box;
      font-size: 14px;
    }
    .btn-save { 
      background: linear-gradient(145deg, #2ecc71, #27ae60);
      box-shadow: 0 4px 8px rgba(46, 204, 113, 0.3);
      width: 100%; 
      margin-top: 25px; 
      color: #fff; 
    }
    .btn-close { 
      background: linear-gradient(145deg, #e74c3c, #c0392b);
      box-shadow: 0 4px 8px rgba(231, 76, 60, 0.3);
      width: 100%; 
      margin-top: 12px; 
      color: #fff; 
    }
    
    /* Desktop Layout */
    @media (min-width: 1024px) {
      body {
        padding: 15px;
      }
      .section {
        padding: 20px;
        width: 100%;
      }
      h2 {
        margin-bottom: 30px;
      }
      .command-queue {
        margin-bottom: 30px;
      }
      .sections-container {
        flex-direction: row;
        justify-content: center;
        align-items: flex-start;
        gap: 50px;
        padding: 0 20px;
      }
      .section-column {
        flex: 0 1 450px;
        display: flex;
        flex-direction: column;
        gap: 20px;
      }
      .section {
        width: 100%;
        max-width: 450px;
        margin: 0;
      }
    }
  </style>
</head>
<body>
  <h2>Sesame Controller</h2>
  <div id="connBar" style="display:flex;justify-content:center;align-items:center;gap:12px;font-size:11px;color:#aaa;margin:-4px 0 8px 0;">
    <span id="connDot" style="width:8px;height:8px;border-radius:50%;background:#555;display:inline-block;"></span>
    <span id="connText">Connecting...</span>
    <span id="connMode" style="color:#666;"></span>
    <span id="connRSSI" style="color:#666;"></span>
  </div>
  <div class="command-queue" id="queueStatus">Command Queue: 0/3</div>
  
  <div class="sections-container">
    <div class="section-column">
      <!-- Camera Feed Section -->
      <div class="section" id="cameraSection">
        <div class="section-title" style="display:flex;justify-content:space-between;align-items:center;">
          <span>Camera Feed <span id="camFps" style="font-size:11px;color:#888;font-weight:normal;"></span></span>
          <div style="display:flex;align-items:center;gap:10px;">
            <label style="display:flex;align-items:center;gap:6px;cursor:pointer;font-size:12px;font-weight:normal;color:#aaa;">
              <input type="checkbox" id="cameraToggle" checked style="width:16px;height:16px;accent-color:var(--content-color);" onchange="toggleCamera(this.checked)">
              On
            </label>
            <label id="camLedWrap" style="display:none;align-items:center;gap:6px;cursor:pointer;font-size:12px;font-weight:normal;color:#aaa;">
              <input type="checkbox" id="cameraLedToggle" style="width:16px;height:16px;accent-color:var(--content-color);" onchange="toggleCamLed(this.checked)">
              LED
            </label>
          </div>
        </div>
        <img id="cameraStream" style="width:100%;border-radius:8px;background:#111;display:none;" alt="Camera">
        <div id="camOff" style="display:none;padding:20px;color:#666;font-size:13px;">Camera off — saves ~40mA</div>
      </div>

      <!-- Movement Control Section -->
      <div class="section">
    <div class="section-title">Movement Control</div>
    <div class="dpad-container">
      <div class="dpad">
        <div class="spacer"></div>
        <button onmousedown="move('forward')" onmouseup="stop()" ontouchstart="move('forward')" ontouchend="stop()">&#9650;</button>
        <div class="spacer"></div>
        
        <button onmousedown="move('left')" onmouseup="stop()" ontouchstart="move('left')" ontouchend="stop()">&#9664;</button>
        <button onmousedown="move('backward')" onmouseup="stop()" ontouchstart="move('backward')" ontouchend="stop()">&#9660;</button>
        <button onmousedown="move('right')" onmouseup="stop()" ontouchstart="move('right')" ontouchend="stop()">&#9654;</button>
      </div>
      <button class="btn-stop-all" onclick="stop()">STOP ALL</button>
    </div>
  </div>

      <!-- Poses & Animations Section -->
      <div class="section">
        <div class="section-title">Poses & Animations</div>
        <div class="grid">
          <button class="btn-pose" onclick="pose('rest')">Rest</button>
          <button class="btn-pose" onclick="pose('stand')">Stand</button>
          <button class="btn-pose" onclick="pose('wave')">Wave</button>
          <button class="btn-pose" onclick="pose('dance')">Dance</button>
          <button class="btn-pose" onclick="pose('swim')">Swim</button>
          <button class="btn-pose" onclick="pose('point')">Point</button>
          <button class="btn-pose" onclick="pose('pushup')">Pushup</button>
          <button class="btn-pose" onclick="pose('bow')">Bow</button>
          <button class="btn-pose" onclick="pose('cute')">Cute</button>
          <button class="btn-pose" onclick="pose('freaky')">Freaky</button>
          <button class="btn-pose" onclick="pose('worm')">Worm</button>
          <button class="btn-pose" onclick="pose('shake')">Shake</button>
          <button class="btn-pose" onclick="pose('shrug')">Shrug</button>
          <button class="btn-pose" onclick="pose('dead')">Dead</button>
          <button class="btn-pose" onclick="pose('crab')">Crab</button>
        </div>
      </div>
    </div>

    <div class="section-column">
      <!-- Settings & Status Section -->
      <div class="section">
        <div class="section-title">System</div>
        <button class="btn-settings" onclick="openSettings()">Settings</button>
        <div style="margin-top: 15px;">
          <div id="gamepadStatus" class="gamepad-status">Gamepad disconnected</div>
        </div>
      </div>
    </div>
  </div>

  <div id="settingsPanel" class="settings-panel">
    <div class="settings-content">
      <h3>Settings</h3>
      
      <div class="settings-section">
        <h4>Animation Parameters</h4>
        <label>Frame Delay (ms):</label>
        <input type="number" id="frameDelay" min="1" max="1000" step="1">
        <label>Walk Cycles:</label>
        <input type="number" id="walkCycles" min="1" max="50" step="1">
      </div>

      <div class="settings-section">
        <h4>Motor Settings</h4>
        <label>Motor Current Delay (ms):</label>
        <input type="number" id="motorCurrentDelay" min="0" max="500" step="1">
        <label>Motor Speed:</label>
        <select id="motorSpeed">
          <option value="slow">Slow</option>
          <option value="medium" selected>Medium</option>
          <option value="fast">Fast</option>
        </select>
      </div>

      <div class="settings-section">
        <h4>Theme</h4>
        <label>Accent Color:</label>
        <select id="themeColor">
          <option value="#ff8c42">Orange (Default)</option>
          <option value="#66d9ef">Cyan</option>
          <option value="#a8dadc">Light Blue</option>
          <option value="#2ecc71">Green</option>
          <option value="#e74c3c">Red</option>
          <option value="#9b59b6">Purple</option>
          <option value="#f39c12">Yellow</option>
          <option value="#e91e63">Pink</option>
          <option value="custom">Custom</option>
        </select>
        <input type="color" id="customColor" value="#ff8c42" style="margin-top: 10px; display: none;">
      </div>

      <div class="settings-section">
        <h4>WiFi Network</h4>
        <div id="wifiStatus" style="font-size:11px;color:#8cf;margin-bottom:8px;"></div>
        <label>SSID:</label>
        <input type="text" id="staSSID" maxlength="32" placeholder="Your WiFi name" style="width:100%;background:#222;border:1px solid #444;color:#eee;border-radius:4px;padding:6px;font-size:13px;">
        <label>Password:</label>
        <input type="password" id="staPass" maxlength="63" placeholder="WiFi password" style="width:100%;background:#222;border:1px solid #444;color:#eee;border-radius:4px;padding:6px;font-size:13px;">
        <button class="btn-save" style="width:100%;margin-top:10px;" onclick="saveWifi()">Save WiFi &amp; Reboot</button>
      </div>

      <button class="btn-settings" style="width: 100%; margin-top: 20px;" onclick="openMotorControl()">Manual Motor Control</button>

      <button class="btn-save" onclick="saveSettings()">Save Settings</button>
      <button class="btn-close" onclick="closeSettings()">Close</button>
    </div>
  </div>

  <div id="motorControlPanel" class="settings-panel">
    <div class="settings-content">
      <h3>Manual Motor Control</h3>
      <div class="lock-indicator" id="lockIndicator">Locked during animations</div>
      
      <div class="settings-section">
        <div class="motor-controls">
          <div class="motor-slider">
            <label>R1 (ch0)</label>
            <div class="slider-wrap"><input type="range" id="motor1" min="0" max="180" value="90" oninput="updateMotor('R1', this.value, 1)"><span class="angle-val" id="m1val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(0,'min')">Set Min</button><input type="number" id="cal0min" min="500" max="2499" value="500" onchange="setCal(0)">&micro;s &ndash; <input type="number" id="cal0max" min="501" max="2500" value="2500" onchange="setCal(0)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(0,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(0,'min')">Set Min</button><input type="number" id="ang0min" min="0" max="179" value="0" onchange="setCal(0)">&deg; &ndash; <input type="number" id="ang0max" min="1" max="180" value="180" onchange="setCal(0)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(0,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>R2 (ch1)</label>
            <div class="slider-wrap"><input type="range" id="motor2" min="0" max="180" value="90" oninput="updateMotor('R2', this.value, 2)"><span class="angle-val" id="m2val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(1,'min')">Set Min</button><input type="number" id="cal1min" min="500" max="2499" value="500" onchange="setCal(1)">&micro;s &ndash; <input type="number" id="cal1max" min="501" max="2500" value="2500" onchange="setCal(1)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(1,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(1,'min')">Set Min</button><input type="number" id="ang1min" min="0" max="179" value="0" onchange="setCal(1)">&deg; &ndash; <input type="number" id="ang1max" min="1" max="180" value="180" onchange="setCal(1)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(1,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>L1 (ch2)</label>
            <div class="slider-wrap"><input type="range" id="motor3" min="0" max="180" value="90" oninput="updateMotor('L1', this.value, 3)"><span class="angle-val" id="m3val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(2,'min')">Set Min</button><input type="number" id="cal2min" min="500" max="2499" value="500" onchange="setCal(2)">&micro;s &ndash; <input type="number" id="cal2max" min="501" max="2500" value="2500" onchange="setCal(2)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(2,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(2,'min')">Set Min</button><input type="number" id="ang2min" min="0" max="179" value="0" onchange="setCal(2)">&deg; &ndash; <input type="number" id="ang2max" min="1" max="180" value="180" onchange="setCal(2)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(2,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>L2 (ch3)</label>
            <div class="slider-wrap"><input type="range" id="motor4" min="0" max="180" value="90" oninput="updateMotor('L2', this.value, 4)"><span class="angle-val" id="m4val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(3,'min')">Set Min</button><input type="number" id="cal3min" min="500" max="2499" value="500" onchange="setCal(3)">&micro;s &ndash; <input type="number" id="cal3max" min="501" max="2500" value="2500" onchange="setCal(3)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(3,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(3,'min')">Set Min</button><input type="number" id="ang3min" min="0" max="179" value="0" onchange="setCal(3)">&deg; &ndash; <input type="number" id="ang3max" min="1" max="180" value="180" onchange="setCal(3)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(3,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>R4 (ch4)</label>
            <div class="slider-wrap"><input type="range" id="motor5" min="0" max="180" value="90" oninput="updateMotor('R4', this.value, 5)"><span class="angle-val" id="m5val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(4,'min')">Set Min</button><input type="number" id="cal4min" min="500" max="2499" value="500" onchange="setCal(4)">&micro;s &ndash; <input type="number" id="cal4max" min="501" max="2500" value="2500" onchange="setCal(4)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(4,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(4,'min')">Set Min</button><input type="number" id="ang4min" min="0" max="179" value="0" onchange="setCal(4)">&deg; &ndash; <input type="number" id="ang4max" min="1" max="180" value="180" onchange="setCal(4)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(4,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>R3 (ch5)</label>
            <div class="slider-wrap"><input type="range" id="motor6" min="0" max="180" value="90" oninput="updateMotor('R3', this.value, 6)"><span class="angle-val" id="m6val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(5,'min')">Set Min</button><input type="number" id="cal5min" min="500" max="2499" value="500" onchange="setCal(5)">&micro;s &ndash; <input type="number" id="cal5max" min="501" max="2500" value="2500" onchange="setCal(5)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(5,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(5,'min')">Set Min</button><input type="number" id="ang5min" min="0" max="179" value="0" onchange="setCal(5)">&deg; &ndash; <input type="number" id="ang5max" min="1" max="180" value="180" onchange="setCal(5)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(5,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>L3 (ch6)</label>
            <div class="slider-wrap"><input type="range" id="motor7" min="0" max="180" value="90" oninput="updateMotor('L3', this.value, 7)"><span class="angle-val" id="m7val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(6,'min')">Set Min</button><input type="number" id="cal6min" min="500" max="2499" value="500" onchange="setCal(6)">&micro;s &ndash; <input type="number" id="cal6max" min="501" max="2500" value="2500" onchange="setCal(6)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(6,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(6,'min')">Set Min</button><input type="number" id="ang6min" min="0" max="179" value="0" onchange="setCal(6)">&deg; &ndash; <input type="number" id="ang6max" min="1" max="180" value="180" onchange="setCal(6)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(6,'max')">Set Max</button></div>
          </div>
          <div class="motor-slider">
            <label>L4 (ch7)</label>
            <div class="slider-wrap"><input type="range" id="motor8" min="0" max="180" value="90" oninput="updateMotor('L4', this.value, 8)"><span class="angle-val" id="m8val">90&deg;</span></div>
            <div class="cal-row"><button class="cal-btn" onclick="setCalFromSlider(7,'min')">Set Min</button><input type="number" id="cal7min" min="500" max="2499" value="500" onchange="setCal(7)">&micro;s &ndash; <input type="number" id="cal7max" min="501" max="2500" value="2500" onchange="setCal(7)">&micro;s<button class="cal-btn" onclick="setCalFromSlider(7,'max')">Set Max</button></div>
            <div class="angle-row"><button class="cal-btn" onclick="setAngleLimitFromSlider(7,'min')">Set Min</button><input type="number" id="ang7min" min="0" max="179" value="0" onchange="setCal(7)">&deg; &ndash; <input type="number" id="ang7max" min="1" max="180" value="180" onchange="setCal(7)">&deg;<button class="cal-btn" onclick="setAngleLimitFromSlider(7,'max')">Set Max</button></div>
          </div>
        </div>
      </div>

      <button class="btn-save" onclick="centerAllMotors()">Center All (90&deg;)</button>
      <button class="btn-save" onclick="saveCalibration()">Save Calibration</button>
      <button class="btn-close" onclick="closeMotorControl()">Close</button>
    </div>
  </div>

<script>
// Command queue management - max 3 commands
let commandQueue = 0;
const MAX_COMMANDS = 3;
let motorsLocked = false;

// Load theme on page load
document.addEventListener('DOMContentLoaded', () => {
  loadTheme();
});

function loadTheme() {
  const savedColor = localStorage.getItem('themeColor');
  if (savedColor) {
    applyTheme(savedColor);
  }
}

function applyTheme(color) {
  const root = document.documentElement;
  root.style.setProperty('--content-color', color);
  
  // Calculate darker shades
  const rgb = hexToRgb(color);
  if (rgb) {
    const dark = `rgb(${Math.max(0, rgb.r - 20)}, ${Math.max(0, rgb.g - 20)}, ${Math.max(0, rgb.b - 20)})`;
    const darker = `rgb(${Math.max(0, rgb.r - 40)}, ${Math.max(0, rgb.g - 40)}, ${Math.max(0, rgb.b - 40)})`;
    const glow = `rgba(${rgb.r}, ${rgb.g}, ${rgb.b}, 0.3)`;
    
    root.style.setProperty('--content-color-dark', dark);
    root.style.setProperty('--content-color-darker', darker);
    root.style.setProperty('--content-color-glow', glow);
  }
}

function hexToRgb(hex) {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : null;
}

function updateQueueStatus() {
  const queueEl = document.getElementById('queueStatus');
  queueEl.textContent = `Command Queue: ${commandQueue}/${MAX_COMMANDS}`;
  if (commandQueue >= MAX_COMMANDS) {
    queueEl.classList.add('full');
  } else {
    queueEl.classList.remove('full');
  }
}

function canSendCommand() {
  return commandQueue < MAX_COMMANDS;
}

function incrementQueue() {
  commandQueue++;
  updateQueueStatus();
  setTimeout(() => {
    if (commandQueue > 0) {
      commandQueue--;
    }
    updateQueueStatus();
  }, 1000);
}

function lockMotors(duration = 3000) {
  motorsLocked = true;
  document.getElementById('lockIndicator').classList.add('active');
  for (let i = 1; i <= 8; i++) {
    const slider = document.getElementById('motor' + i);
    if (slider) slider.disabled = true;
  }
  setTimeout(() => {
    motorsLocked = false;
    document.getElementById('lockIndicator').classList.remove('active');
    for (let i = 1; i <= 8; i++) {
      const slider = document.getElementById('motor' + i);
      if (slider) slider.disabled = false;
    }
  }, duration);
}

function move(dir) { 
  if (!canSendCommand()) return;
  incrementQueue();
  fetch('/cmd?go=' + dir).catch(console.log); 
}

function stop() { 
  commandQueue = 0;
  updateQueueStatus();
  fetch('/cmd?stop=1').catch(console.log); 
}

function pose(name) { 
  if (!canSendCommand()) return;
  incrementQueue();
  lockMotors(3000);
  fetch('/cmd?pose=' + name).catch(console.log); 
}

// Arrow key support for movement
(function() {
  const keyMap = { ArrowUp: 'forward', ArrowDown: 'backward', ArrowLeft: 'left', ArrowRight: 'right' };
  const held = {};
  document.addEventListener('keydown', function(e) {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const dir = keyMap[e.key];
    if (dir && !held[e.key]) { held[e.key] = true; move(dir); e.preventDefault(); }
  });
  document.addEventListener('keyup', function(e) {
    const dir = keyMap[e.key];
    if (dir && held[e.key]) { delete held[e.key]; if (!Object.keys(held).length) stop(); e.preventDefault(); }
  });
})();

let motorThrottleTimers = {};

function updateMotor(name, value, sliderIdx) {
  if (motorsLocked) return;
  document.getElementById('m' + sliderIdx + 'val').textContent = value + ' \u00B0';
  // Throttle: send at most every 50ms per servo to avoid flooding
  if (motorThrottleTimers[sliderIdx]) return;
  motorThrottleTimers[sliderIdx] = setTimeout(() => {
    delete motorThrottleTimers[sliderIdx];
  }, 50);
  fetch('/cmd?motor=' + name + '&value=' + value).catch(console.log);
}

function centerAllMotors() {
  if (motorsLocked) return;
  const names = ['R1','R2','L1','L2','R4','R3','L3','L4'];
  for (let i = 0; i < 8; i++) {
    document.getElementById('motor' + (i+1)).value = 90;
    document.getElementById('m' + (i+1) + 'val').textContent = '90 \u00B0';
  }
  // Stagger in pairs of 2 with 150ms gaps to limit inrush current
  for (let g = 0; g < 4; g++) {
    setTimeout(() => {
      fetch('/cmd?motor=' + names[g*2] + '&value=90').catch(console.log);
      fetch('/cmd?motor=' + names[g*2+1] + '&value=90').catch(console.log);
    }, g * 150);
  }
}

function openSettings() {
  fetch('/getSettings').then(r => r.json()).then(data => {
    document.getElementById('frameDelay').value = data.frameDelay || 100;
    document.getElementById('walkCycles').value = data.walkCycles || 10;
    document.getElementById('motorCurrentDelay').value = data.motorCurrentDelay || 20;
    document.getElementById('motorSpeed').value = data.motorSpeed || 'medium';
    
    // WiFi
    document.getElementById('staSSID').value = data.staSSID || '';
    const ws = document.getElementById('wifiStatus');
    if (data.networkConnected) {
      ws.textContent = 'Connected to ' + data.staSSID + ' (' + data.networkIP + ')';
      ws.style.color = '#2ecc71';
    } else if (data.staEnabled) {
      ws.textContent = 'Not connected (configured: ' + data.staSSID + ')';
      ws.style.color = '#e74c3c';
    } else {
      ws.textContent = 'AP-only mode';
      ws.style.color = '#888';
    }
    
    // Load theme settings
    const savedColor = localStorage.getItem('themeColor') || '#ff8c42';
    const colorSelect = document.getElementById('themeColor');
    const customColorInput = document.getElementById('customColor');
    
    // Check if saved color matches a preset
    let matchFound = false;
    for (let option of colorSelect.options) {
      if (option.value === savedColor) {
        colorSelect.value = savedColor;
        matchFound = true;
        break;
      }
    }
    
    if (!matchFound) {
      colorSelect.value = 'custom';
      customColorInput.value = savedColor;
      customColorInput.style.display = 'block';
    }
    
    document.getElementById('settingsPanel').style.display = 'block';
  }).catch(() => {
    // Fallback if settings endpoint doesn't exist yet
    document.getElementById('frameDelay').value = 100;
    document.getElementById('walkCycles').value = 10;
    document.getElementById('motorCurrentDelay').value = 20;
    
    const savedColor = localStorage.getItem('themeColor') || '#ff8c42';
    document.getElementById('themeColor').value = savedColor;
    
    document.getElementById('settingsPanel').style.display = 'block';
  });
  
  // Add event listener for theme color change
  document.getElementById('themeColor').addEventListener('change', function() {
    const customColorInput = document.getElementById('customColor');
    if (this.value === 'custom') {
      customColorInput.style.display = 'block';
    } else {
      customColorInput.style.display = 'none';
      applyTheme(this.value);
    }
  });
  
  document.getElementById('customColor').addEventListener('input', function() {
    applyTheme(this.value);
  });
}

function closeSettings() { 
  document.getElementById('settingsPanel').style.display = 'none'; 
}

function openMotorControl() {
  document.getElementById('motorControlPanel').style.display = 'block';
  fetch('/getCal').then(r => r.json()).then(data => {
    for (let i = 0; i < 8; i++) {
      document.getElementById('cal' + i + 'min').value = data.min[i];
      document.getElementById('cal' + i + 'max').value = data.max[i];
      document.getElementById('ang' + i + 'min').value = data.angleMin[i];
      document.getElementById('ang' + i + 'max').value = data.angleMax[i];
      if (data.pos) {
        const angle = data.pos[i];
        document.getElementById('motor' + (i + 1)).value = angle;
        document.getElementById('m' + (i + 1) + 'val').textContent = angle + ' \u00B0';
      }
    }
  }).catch(() => {});
}

function setCal(ch) {
  const mn = document.getElementById('cal' + ch + 'min').value;
  const mx = document.getElementById('cal' + ch + 'max').value;
  const amn = document.getElementById('ang' + ch + 'min').value;
  const amx = document.getElementById('ang' + ch + 'max').value;
  fetch('/setCal?min' + ch + '=' + mn + '&max' + ch + '=' + mx + '&amin' + ch + '=' + amn + '&amax' + ch + '=' + amx).catch(console.log);
}

function setAngleLimitFromSlider(ch, target) {
  const angle = parseInt(document.getElementById('motor' + (ch + 1)).value);
  document.getElementById('ang' + ch + target).value = angle;
  setCal(ch);
}

function setCalFromSlider(ch, target) {
  const sliderIdx = ch + 1;
  const angle = parseInt(document.getElementById('motor' + sliderIdx).value);
  const mnCur = parseInt(document.getElementById('cal' + ch + 'min').value);
  const mxCur = parseInt(document.getElementById('cal' + ch + 'max').value);
  const us = Math.round(mnCur + (angle / 180.0) * (mxCur - mnCur));
  document.getElementById('cal' + ch + target).value = us;
  setCal(ch);
}

function saveCalibration() {
  fetch('/saveCal').then(() => {
    alert('Calibration saved to flash!');
  }).catch(() => {
    alert('Save failed');
  });
}

function closeMotorControl() {
  document.getElementById('motorControlPanel').style.display = 'none';
}

function saveSettings() {
  const fd = document.getElementById('frameDelay').value;
  const wc = document.getElementById('walkCycles').value;
  const mcd = document.getElementById('motorCurrentDelay').value;
  const ms = document.getElementById('motorSpeed').value;
  
  // Save theme color
  const colorSelect = document.getElementById('themeColor');
  const customColorInput = document.getElementById('customColor');
  const themeColor = colorSelect.value === 'custom' ? customColorInput.value : colorSelect.value;
  localStorage.setItem('themeColor', themeColor);
  applyTheme(themeColor);
  
  fetch(`/setSettings?frameDelay=${fd}&walkCycles=${wc}&motorCurrentDelay=${mcd}&motorSpeed=${ms}`)
    .then(() => closeSettings())
    .catch(() => closeSettings());
}

function saveWifi() {
  const ssid = document.getElementById('staSSID').value.trim();
  const pass = document.getElementById('staPass').value;
  if (ssid.length === 0) {
    if (!confirm('SSID is empty — this will disable WiFi station mode. Continue?')) return;
  }
  fetch('/setWifi?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass))
    .then(r => r.text()).then(t => {
      alert(t + '\\nThe robot will reboot now to apply WiFi settings.');
      fetch('/cmd?reboot=1').catch(() => {});
    }).catch(() => alert('Failed to save WiFi settings'));
}

let activeGamepadIndex = null;
let gamepadPollId = null;
let lastButtonStates = [];
let lastAxisDir = { x: 0, y: 0 };
const axisThreshold = 0.5;
const pollIntervalMs = 80;

const buttonBindings = {
  0: () => pose('stand'),   // A / Cross
  1: () => pose('wave'),    // B / Circle
  2: () => pose('dance'),   // X / Square
  3: () => pose('swim'),    // Y / Triangle
  4: () => pose('point'),   // LB / L1
  5: () => pose('pushup'),  // RB / R1
  6: () => pose('bow'),     // LT / L2
  7: () => pose('shake'),   // RT / R2
  8: () => stop(),          // Back / Share
  9: () => pose('rest'),    // Start / Options
  10: () => pose('cute'),   // L3
  11: () => pose('freaky'), // R3
  12: () => move('forward'),// D-pad up
  13: () => move('backward'),// D-pad down
  14: () => move('left'),   // D-pad left
  15: () => move('right'),  // D-pad right
  16: () => stop(),         // Home / PS
  17: () => pose('worm')    // Touchpad / extra
};

const buttonReleaseStop = new Set([12, 13, 14, 15]);

function updateGamepadStatus(connected) {
  const status = document.getElementById('gamepadStatus');
  if (!status) return;
  if (connected) {
    status.textContent = 'Gamepad connected';
    status.classList.add('connected');
  } else {
    status.textContent = 'Gamepad disconnected';
    status.classList.remove('connected');
  }
}

function handleButtonChange(index, pressed) {
  if (pressed) {
    const action = buttonBindings[index];
    if (action) action();
  } else if (buttonReleaseStop.has(index)) {
    stop();
  }
}

function getAxisDirection(x, y) {
  if (Math.abs(x) < axisThreshold && Math.abs(y) < axisThreshold) return { x: 0, y: 0 };
  if (Math.abs(x) > Math.abs(y)) {
    return { x: x > 0 ? 1 : -1, y: 0 };
  }
  return { x: 0, y: y > 0 ? 1 : -1 };
}

function applyAxisDirection(dir) {
  if (dir.x === 1) move('right');
  else if (dir.x === -1) move('left');
  else if (dir.y === 1) move('backward');
  else if (dir.y === -1) move('forward');
  else stop();
}

function pollGamepad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  const pad = pads && activeGamepadIndex !== null ? pads[activeGamepadIndex] : null;
  if (!pad) {
    updateGamepadStatus(false);
    return;
  }
  updateGamepadStatus(true);

  if (!lastButtonStates.length) {
    lastButtonStates = pad.buttons.map(b => !!b.pressed);
  }
  pad.buttons.forEach((btn, i) => {
    const pressed = !!btn.pressed;
    if (pressed !== lastButtonStates[i]) {
      handleButtonChange(i, pressed);
      lastButtonStates[i] = pressed;
    }
  });

  const x = pad.axes[0] || 0;
  const y = pad.axes[1] || 0;
  const dir = getAxisDirection(x, y);
  if (dir.x !== lastAxisDir.x || dir.y !== lastAxisDir.y) {
    applyAxisDirection(dir);
    lastAxisDir = dir;
  }
}

window.addEventListener('gamepadconnected', (e) => {
  activeGamepadIndex = e.gamepad.index;
  lastButtonStates = [];
  lastAxisDir = { x: 0, y: 0 };
  updateGamepadStatus(true);
  if (!gamepadPollId) {
    gamepadPollId = setInterval(pollGamepad, pollIntervalMs);
  }
});

window.addEventListener('gamepaddisconnected', (e) => {
  if (activeGamepadIndex === e.gamepad.index) {
    activeGamepadIndex = null;
    lastButtonStates = [];
    lastAxisDir = { x: 0, y: 0 };
    updateGamepadStatus(false);
  }
});

if (navigator.getGamepads) {
  setInterval(() => {
    if (activeGamepadIndex !== null) return;
    const pads = navigator.getGamepads();
    if (!pads) return;
    for (let i = 0; i < pads.length; i++) {
      if (pads[i]) {
        activeGamepadIndex = pads[i].index;
        updateGamepadStatus(true);
        if (!gamepadPollId) {
          gamepadPollId = setInterval(pollGamepad, pollIntervalMs);
        }
        break;
      }
    }
  }, 1000);
}
// Camera stream — connect to port 81 MJPEG endpoint with FPS counter
var camStreamActive = false;
(function() {
  const img = document.getElementById('cameraStream');
  const offMsg = document.getElementById('camOff');
  const fpsEl = document.getElementById('camFps');
  const ledWrap = document.getElementById('camLedWrap');
  const ledToggle = document.getElementById('cameraLedToggle');
  const streamUrl = 'http://' + location.hostname + ':81';
  let retryTimer = null;
  let frameCount = 0;
  let lastFpsTime = Date.now();

  function freshStreamUrl() {
    return streamUrl + '/?_=' + Date.now();
  }

  // FPS counter — MJPEG fires onload for each frame
  img.onload = function() {
    camStreamActive = true;
    frameCount++;
    const now = Date.now();
    if (now - lastFpsTime >= 2000) {
      const fps = (frameCount / ((now - lastFpsTime) / 1000)).toFixed(1);
      fpsEl.textContent = fps + ' fps';
      frameCount = 0;
      lastFpsTime = now;
    }
  };

  // Load initial camera state then connect
  fetch('/getSettings').then(r => r.json()).then(data => {
    const enabled = (data.cameraEnabled !== false);
    document.getElementById('cameraToggle').checked = enabled;
    const ledAvailable = (data.cameraLedAvailable === true);
    ledWrap.style.display = ledAvailable ? 'flex' : 'none';
    ledToggle.checked = (data.cameraLed === true);
    if (enabled) {
      startStream();
    } else {
      img.style.display = 'none';
      offMsg.style.display = 'block';
      fpsEl.textContent = '';
    }
  }).catch(() => startStream());

  function startStream() {
    if (retryTimer) {
      clearTimeout(retryTimer);
      retryTimer = null;
    }
    let retries = 3;
    camStreamActive = false;
    img.style.display = 'block';
    offMsg.style.display = 'none';
    img.src = '';
    img.onerror = function() {
      if (--retries > 0) {
        retryTimer = setTimeout(() => {
          img.src = freshStreamUrl();
        }, 2000);
      }
      else { img.style.display = 'none'; offMsg.style.display = 'block'; offMsg.textContent = 'Stream unavailable'; }
    };
    img.src = freshStreamUrl();
  }

  window.startCamStream = startStream;
})();

function toggleCamera(on) {
  const img = document.getElementById('cameraStream');
  const offMsg = document.getElementById('camOff');
  const fpsEl = document.getElementById('camFps');
  fetch('/setSettings?camera=' + (on ? '1' : '0')).then(() => {
    if (!on) {
      camStreamActive = false;
      img.onerror = null;
      img.src = '';
      img.style.display = 'none';
      offMsg.style.display = 'block';
      offMsg.textContent = 'Camera off \u2014 saves ~40mA';
      fpsEl.textContent = '';
    } else {
      offMsg.style.display = 'none';
      // Wait for firmware wake+reinit path before reconnecting MJPEG.
      setTimeout(() => {
        fetch('/getSettings').then(r => r.json()).then(data => {
          if (data.cameraEnabled !== false) {
            window.startCamStream();
          }
        }).catch(() => window.startCamStream());
      }, 1300);
    }
  });
}

function toggleCamLed(on) {
  fetch('/setSettings?camLed=' + (on ? '1' : '0')).catch(() => {
    // Re-sync from firmware state on error.
    fetch('/getSettings').then(r => r.json()).then(data => {
      const ledToggle = document.getElementById('cameraLedToggle');
      if (ledToggle) ledToggle.checked = (data.cameraLed === true);
    }).catch(() => {});
  });
}

// --- Connection heartbeat ---
(function() {
  const dot = document.getElementById('connDot');
  const txt = document.getElementById('connText');
  const mode = document.getElementById('connMode');
  const rssi = document.getElementById('connRSSI');
  let fails = 0;

  function rssiToBar(r) {
    if (r >= -50) return '\u2582\u2584\u2586\u2588';
    if (r >= -60) return '\u2582\u2584\u2586\u2007';
    if (r >= -70) return '\u2582\u2584\u2007\u2007';
    return '\u2582\u2007\u2007\u2007';
  }

  function poll() {
    fetch('/api/status').then(r => r.json()).then(d => {
      fails = 0;
      dot.style.background = '#2ecc71';
      let info = 'Connected';
      if (d.networkConnected) {
        mode.textContent = 'STA+AP';
        rssi.textContent = rssiToBar(d.rssi) + ' ' + d.rssi + 'dBm';
        info += ' \u2022 ' + d.ssid + ' (' + d.networkIP + ')';
      } else {
        mode.textContent = 'AP only';
        rssi.textContent = '';
      }
      if (d.apClients !== undefined) info += ' \u2022 ' + d.apClients + ' client' + (d.apClients !== 1 ? 's' : '');
      txt.textContent = info;
    }).catch(() => {
      fails++;
      if (fails >= 2) {
        dot.style.background = '#e74c3c';
        txt.textContent = 'Disconnected';
        mode.textContent = '';
        rssi.textContent = '';
      }
    });
  }
  poll();
  setInterval(poll, 3000);
})();
</script>
</body>
</html>
)rawliteral";
