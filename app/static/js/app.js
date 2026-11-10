/**
 * AERO-TRACK GCS - Modern Client Controller
 * Handles Canvas ROI selection, tracked target rendering, deadband box,
 * WebSocket telemetry, MAVLink (UDP / Serial cross-platform), and settings.
 */

class AeroTrackGCS {
  constructor() {
    this.currentMode = 'STANDBY';
    this.isDrawingROI = false;
    this.isSelectingROI = false;
    this.roiStart = null;
    this.roiCurrent = null;
    this.trackedBbox = null; // [x, y, w, h] from tracker
    this.deadbandX = 0.08;   // Normalized ratio of width (default 8%)
    this.deadbandY = 0.08;   // Normalized ratio of height (default 8%)
    const defaultAlt = document.getElementById('setting-takeoff-alt')?.value || '1.0';
    this.takeoffAltitude = parseFloat(localStorage.getItem('aerotrack_takeoff_alt') || defaultAlt);

    // UAV, Gimbal & Vision connection status
    this.uavConnected = false;
    this.lastUavConnectedState = null;
    this.uavArmed = false;
    this.uavAltitude = 0.0;
    this.uavMode = '---';
    this._isTogglingMavlink = false;
    this.gimbalConnected = false;
    this._isTogglingGimbal = false;
    this.rtspConnected = false;
    this.lastRtspConnectedState = null;
    this._toastTimer = null;
    this.connType = 'serial';
    this._processedStatusTexts = new Set(); // Track processed STATUSTEXT messages

    this.isCentered = false;

    // DOM Elements
    this.canvas = document.getElementById('hud-canvas');
    this.ctx = this.canvas.getContext('2d');
    this.videoImg = document.getElementById('camera-stream');
    this.roiBanner = document.getElementById('roi-banner');
    this.settingsModal = document.getElementById('settings-modal');

    this.nativeWidth = 640;
    this.nativeHeight = 480;
    this.ws = null;

    this.init();
  }

  init() {
    this.setupCanvas();
    this.setupKeyboardShortcuts();
    this.setupModalBackdrop();
    this.connectWebSocket();
    this.loadSerialPorts();
    this.loadGimbalSerialPorts();

    // Auto-reconnect broken video stream if image error occurs
    if (this.videoImg) {
      this.videoImg.onerror = () => {
        if (!this._streamReconnectTimer) {
          this._streamReconnectTimer = setTimeout(() => {
            this._streamReconnectTimer = null;
            this.reloadVideoStream();
          }, 2000);
        }
      };
    }

    // Populate saved takeoff altitude in UI
    const takeoffInput = document.getElementById('setting-takeoff-alt');
    if (takeoffInput) {
      takeoffInput.value = this.takeoffAltitude;
    }
  }

  reloadVideoStream() {
    if (this.videoImg) {
      const ts = Date.now();
      this.videoImg.src = `/api/video_feed?t=${ts}`;
    }
  }

  setupCanvas() {
    const resizeCanvas = () => {
      const rect = this.canvas.getBoundingClientRect();
      if (rect.width > 0 && rect.height > 0) {
        const newW = Math.round(rect.width);
        const newH = Math.round(rect.height);
        if (this.canvas.width !== newW || this.canvas.height !== newH) {
          this.canvas.width = newW;
          this.canvas.height = newH;
          this.renderCanvas();
        }
      }
      if (this.videoImg.naturalWidth && this.videoImg.naturalHeight) {
        this.nativeWidth = this.videoImg.naturalWidth;
        this.nativeHeight = this.videoImg.naturalHeight;
      }
    };

    window.addEventListener('resize', resizeCanvas);
    this.videoImg.addEventListener('load', resizeCanvas);
    setTimeout(resizeCanvas, 300);
    setInterval(resizeCanvas, 2000);

    // Mouse events for ROI drawing
    this.canvas.addEventListener('mousedown', (e) => this.onMouseDown(e));
    this.canvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
    this.canvas.addEventListener('mouseup', (e) => this.onMouseUp(e));

    // Right-click to cancel ROI / tracking
    this.canvas.addEventListener('contextmenu', (e) => {
      e.preventDefault();
      if (this.currentMode === 'TRACKING') {
        this.cancelManualTracking();
      }
    });

    // Touch events for mobile/tablet
    this.canvas.addEventListener('touchstart', (e) => {
      const t = e.touches[0];
      this.canvas.dispatchEvent(new MouseEvent('mousedown', { clientX: t.clientX, clientY: t.clientY }));
    });
    this.canvas.addEventListener('touchmove', (e) => {
      const t = e.touches[0];
      this.canvas.dispatchEvent(new MouseEvent('mousemove', { clientX: t.clientX, clientY: t.clientY }));
    });
    this.canvas.addEventListener('touchend', () => {
      this.canvas.dispatchEvent(new MouseEvent('mouseup', {}));
    });
  }

  setupKeyboardShortcuts() {
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

      const key = e.key.toLowerCase();
      if (key === 'escape') {
        e.preventDefault();
        if (this.settingsModal && this.settingsModal.style.display === 'flex') {
          this.closeSettingsModal();
        } else if (this.currentMode === 'TRACKING') {
          this.cancelManualTracking();
        } else {
          this.emergencyStop();
        }
      } else if (key === ' ' || e.code === 'Space') {
        e.preventDefault();
        this.emergencyStop();
      } else if (key === 't') {
        this.selectMode('TRACKING');
      } else if (key === 'r') {
        if (this.currentMode === 'TRACKING') {
          this.enableROIDraw();
        }
      } else if (key === 'f11') {
        // Intercept F11 to use the unified fullscreen handler
        e.preventDefault();
        this.toggleFullscreen();
      }
    });

    // Listen to native fullscreen changes (e.g. Esc or browser shortcut)
    document.addEventListener('fullscreenchange', () => this.onFullscreenChange());
    document.addEventListener('webkitfullscreenchange', () => this.onFullscreenChange());
    document.addEventListener('mozfullscreenchange', () => this.onFullscreenChange());
    document.addEventListener('MSFullscreenChange', () => this.onFullscreenChange());
  }

  // --- FULLSCREEN TOGGLE ---
  toggleFullscreen() {
    if (!document.fullscreenElement && !document.webkitFullscreenElement && !document.mozFullScreenElement && !document.msFullscreenElement) {
      const docEl = document.documentElement;
      if (docEl.requestFullscreen) {
        docEl.requestFullscreen().catch(() => { });
      } else if (docEl.webkitRequestFullscreen) {
        docEl.webkitRequestFullscreen();
      } else if (docEl.mozRequestFullScreen) {
        docEl.mozRequestFullScreen();
      } else if (docEl.msRequestFullscreen) {
        docEl.msRequestFullscreen();
      }
    } else {
      if (document.exitFullscreen) {
        document.exitFullscreen().catch(() => { });
      } else if (document.webkitExitFullscreen) {
        document.webkitExitFullscreen();
      } else if (document.mozCancelFullScreen) {
        document.mozCancelFullScreen();
      } else if (document.msExitFullscreen) {
        document.msExitFullscreen();
      }
    }
  }

  onFullscreenChange() {
    const isFull = !!(document.fullscreenElement || document.webkitFullscreenElement || document.mozFullScreenElement || document.msFullscreenElement);
    const iconSpan = document.getElementById('fullscreen-icon');
    const btn = document.getElementById('btn-toggle-fullscreen');
    if (!iconSpan) return;

    if (isFull) {
      if (btn) btn.title = 'Thu nhỏ (Thoát toàn màn hình)';
      iconSpan.innerHTML = `
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M8 3v3a2 2 0 0 1-2 2H3m18 0h-3a2 2 0 0 1-2-2V3m0 18v-3a2 2 0 0 1 2-2h3M3 16h3a2 2 0 0 1 2 2v3"/>
        </svg>
      `;
    } else {
      if (btn) btn.title = 'Toàn màn hình / Thu nhỏ (F11)';
      iconSpan.innerHTML = `
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M8 3H5a2 2 0 0 0-2 2v3m18 0V5a2 2 0 0 0-2-2h-3m0 18h3a2 2 0 0 0 2-2v-3M3 16v3a2 2 0 0 0 2 2h3"/>
        </svg>
      `;
    }
  }

  setupModalBackdrop() {
    if (!this.settingsModal) return;
    this.settingsModal.addEventListener('click', (e) => {
      if (e.target === this.settingsModal) {
        this.closeSettingsModal();
      }
    });
  }

  // --- SETTINGS MODAL & TABS ---
  switchSettingsTab(tabName) {
    const tabs = ['connection', 'gimbal', 'video', 'parameters'];
    tabs.forEach(t => {
      const btnId = `tab-btn-${t === 'parameters' ? 'params' : (t === 'connection' ? 'conn' : t)}`;
      const btn = document.getElementById(btnId);
      const pane = document.getElementById(`pane-${t}`);
      if (btn) btn.classList.toggle('active', t === tabName);
      if (pane) pane.classList.toggle('active', t === tabName);
    });

    if (tabName === 'gimbal' && !this.gimbalConnected) {
      this.loadGimbalSerialPorts();
    } else if (tabName === 'connection' && !this.uavConnected) {
      this.loadSerialPorts();
    }
  }

  openSettingsModal() {
    if (this.settingsModal) {
      this.settingsModal.style.display = 'flex';
      if (!this.uavConnected) {
        this.loadSerialPorts();
      }
      if (!this.gimbalConnected) {
        this.loadGimbalSerialPorts();
      }
      this.updateMavlinkUIState(this.uavConnected);
      this.syncParametersUI();
    }
  }

  closeSettingsModal() {
    if (this.settingsModal) {
      this.settingsModal.style.display = 'none';
    }
  }

  syncParametersUI() {
    const takeoffInput = document.getElementById('setting-takeoff-alt');
    if (takeoffInput) takeoffInput.value = this.takeoffAltitude;

    const dbXInput = document.getElementById('setting-deadband-x');
    if (dbXInput) dbXInput.value = Math.round(this.deadbandX * 100);

    const dbYInput = document.getElementById('setting-deadband-y');
    if (dbYInput) dbYInput.value = Math.round(this.deadbandY * 100);

    const statusBadge = document.getElementById('setting-conn-status-badge');
    if (statusBadge) {
      statusBadge.textContent = this.uavConnected ? 'Đã kết nối MAVLink' : 'Mất kết nối';
      statusBadge.className = this.uavConnected ? 'telem-val text-success' : 'telem-val text-warning';
    }

    const videoBadge = document.getElementById('setting-video-status-badge');
    if (videoBadge) {
      videoBadge.textContent = this.rtspConnected ? 'Đã kết nối Camera' : 'Chưa có tín hiệu';
      videoBadge.className = this.rtspConnected ? 'telem-val text-success' : 'telem-val text-warning';
    }
  }

  updateMavlinkUIState(connected) {
    const btnToggle = document.getElementById('btn-mav-toggle-conn');
    const btnText = document.getElementById('btn-mav-toggle-text');
    const serialPortSelect = document.getElementById('setting-mav-serial-port');
    const baudSelect = document.getElementById('setting-mav-baud');
    const btnScanPorts = document.getElementById('btn-scan-ports');
    const connSerialRadio = document.getElementById('conn-type-serial');
    const connUdpRadio = document.getElementById('conn-type-udp');
    const ipInput = document.getElementById('setting-mav-ip');
    const portInput = document.getElementById('setting-mav-port');
    const lockHint = document.getElementById('mav-locked-warning');

    // 1. Update button styling & label without replacing DOM innerHTML
    if (btnToggle) {
      btnToggle.className = connected ? 'btn btn-danger btn-block' : 'btn btn-primary btn-block';
    }
    if (btnText) {
      btnText.textContent = connected ? 'Ngắt Kết Nối' : 'Kết Nối Máy Bay';
    }

    // 2. Lock or unlock settings controls
    if (serialPortSelect) serialPortSelect.disabled = connected;
    if (baudSelect) baudSelect.disabled = connected;
    if (btnScanPorts) {
      btnScanPorts.disabled = connected;
      btnScanPorts.style.pointerEvents = connected ? 'none' : 'auto';
      btnScanPorts.style.opacity = connected ? '0.35' : '1';
    }
    if (connSerialRadio) connSerialRadio.disabled = connected;
    if (connUdpRadio) connUdpRadio.disabled = connected;
    if (ipInput) ipInput.disabled = connected;
    if (portInput) portInput.disabled = connected;

    // 3. Radio option label disabled appearance
    const lblSerial = document.getElementById('lbl-conn-serial');
    const lblUdp = document.getElementById('lbl-conn-udp');
    if (lblSerial) lblSerial.classList.toggle('disabled', connected);
    if (lblUdp) lblUdp.classList.toggle('disabled', connected);

    // 4. Warning lock hint banner
    if (lockHint) {
      lockHint.style.display = connected ? 'flex' : 'none';
    }
  }

  onConnTypeChange(type) {
    if (this.uavConnected) return; // Prevent changing while connected
    this.connType = type;
    const boxUdp = document.getElementById('box-conn-udp');
    const boxSerial = document.getElementById('box-conn-serial');

    if (type === 'udp') {
      if (boxUdp) boxUdp.style.display = 'block';
      if (boxSerial) boxSerial.style.display = 'none';
    } else {
      if (boxUdp) boxUdp.style.display = 'none';
      if (boxSerial) boxSerial.style.display = 'block';
      this.loadSerialPorts();
    }
  }

  async toggleMAVLinkConnection(e) {
    if (e) {
      e.preventDefault();
      e.stopPropagation();
    }
    if (this._isTogglingMavlink) return;
    this._isTogglingMavlink = true;

    try {
      if (this.uavConnected) {
        await this.disconnectMAVLink();
      } else {
        await this.saveMAVLinkSetting();
      }
    } finally {
      this._isTogglingMavlink = false;
    }
  }

  async loadSerialPorts() {
    if (this.uavConnected) return; // Don't wipe port list while connected
    try {
      const res = await fetch('/api/mavlink/serial_ports');
      const data = await res.json();
      const select = document.getElementById('setting-mav-serial-port');
      if (!select) return;

      select.innerHTML = '';
      let preferredPort = '';

      if (data.ports && data.ports.length > 0) {
        data.ports.forEach(p => {
          const opt = document.createElement('option');
          opt.value = p.device;
          opt.textContent = `${p.device} (${p.description || p.device})`;
          const descLower = (p.description || '').toLowerCase();
          if (descLower.includes('ardupilot') || descLower.includes('pixhawk') || descLower.includes('cube')) {
            preferredPort = p.device;
          }
          select.appendChild(opt);
        });

        if (preferredPort) {
          select.value = preferredPort;
        } else {
          select.value = data.ports[0].device;
        }
      } else {
        const opt = document.createElement('option');
        opt.value = '';
        opt.textContent = 'Không tìm thấy cổng COM/TTY (Hãy cắm thiết bị & bấm Quét lại)';
        select.appendChild(opt);
      }
    } catch (err) {
      console.warn('Could not load serial ports:', err);
    }
  }

  async loadGimbalSerialPorts() {
    if (this.gimbalConnected) return; // Don't wipe port list while connected
    try {
      let res = await fetch('/api/gimbal/serial_ports');
      if (!res.ok) {
        res = await fetch('/api/mavlink/serial_ports');
      }
      const data = await res.json();
      const select = document.getElementById('setting-gimbal-port');
      if (!select) return;

      select.innerHTML = '';
      let preferredPort = '';

      if (data.ports && data.ports.length > 0) {
        data.ports.forEach(p => {
          const opt = document.createElement('option');
          opt.value = p.device;
          opt.textContent = `${p.device} (${p.description || p.device})`;
          const descLower = (p.description || '').toLowerCase();
          const devLower = (p.device || '').toLowerCase();
          if (devLower.includes('ttyusb') || devLower.includes('ttyacm') || descLower.includes('usb') || descLower.includes('ch340') || descLower.includes('cp210')) {
            if (!preferredPort) preferredPort = p.device;
          }
          select.appendChild(opt);
        });

        if (preferredPort) {
          select.value = preferredPort;
        } else {
          select.value = data.ports[0].device;
        }
      } else {
        const opt = document.createElement('option');
        opt.value = '';
        opt.textContent = 'Không tìm thấy cổng COM/TTY (Hãy cắm thiết bị & bấm Quét lại)';
        select.appendChild(opt);
      }
    } catch (err) {
      console.warn('Could not load gimbal serial ports:', err);
    }
  }

  async toggleGimbalConnection(e) {
    if (e) {
      e.preventDefault();
      e.stopPropagation();
    }
    if (this._isTogglingGimbal) return;
    this._isTogglingGimbal = true;

    try {
      if (this.gimbalConnected) {
        const res = await fetch('/api/gimbal/disconnect', { method: 'POST' });
        const data = await res.json();
        if (data.status === 'ok') {
          this.addLog('Đã ngắt kết nối Gimbal.', 'info');
          this.gimbalConnected = false;
        } else {
          this.addLog(`Lỗi ngắt kết nối Gimbal: ${data.detail || 'Không rõ'}`, 'warn');
        }
      } else {
        const portSelect = document.getElementById('setting-gimbal-port');
        const baudSelect = document.getElementById('setting-gimbal-baud');
        const port = portSelect ? portSelect.value : '';
        const baud = baudSelect ? parseInt(baudSelect.value, 10) : 115200;

        if (!port) {
          alert('Vui lòng chọn cổng Serial cho Gimbal');
          return;
        }

        const res = await fetch('/api/gimbal/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ port, baud })
        });
        const data = await res.json();
        if (data.status === 'ok') {
          this.addLog(`Đã kết nối Gimbal qua cổng ${port} (${baud} bps).`, 'success');
          this.gimbalConnected = true;
        } else {
          this.addLog(`Lỗi kết nối Gimbal: ${data.detail || 'Không kết nối được'}`, 'warn');
          alert(`Không thể kết nối Gimbal: ${data.detail || 'Lỗi cổng serial'}`);
        }
      }
    } catch (err) {
      this.addLog(`Lỗi kết nối Gimbal: ${err.message}`, 'error');
    } finally {
      this._isTogglingGimbal = false;
    }
  }

  async centerGimbal() {
    try {
      const res = await fetch('/api/gimbal/center', { method: 'POST' });
      const data = await res.json();
      if (data.status === 'ok') {
        this.addLog('Đã gửi lệnh đưa Gimbal về vị trí chính giữa.', 'info');
      } else {
        this.addLog(`Lỗi đưa Gimbal về giữa: ${data.detail || ''}`, 'warn');
      }
    } catch (err) {
      this.addLog(`Lỗi điều khiển Gimbal: ${err.message}`, 'error');
    }
  }

  // --- FLOATING VIDEO TOAST NOTIFICATION ---
  showVideoToast(msg, type = 'info', duration = 3000) {
    const toast = document.getElementById('video-toast-banner');
    const toastText = document.getElementById('video-toast-text');
    if (!toast || !toastText) return;

    if (this._toastTimer) {
      clearTimeout(this._toastTimer);
      this._toastTimer = null;
    }

    toastText.textContent = msg;
    toast.className = `video-toast-banner ${type}`;
    toast.style.display = 'flex';
    toast.style.opacity = '1';

    this._toastTimer = setTimeout(() => {
      toast.style.opacity = '0';
      setTimeout(() => {
        toast.style.display = 'none';
      }, 300);
    }, duration);
  }

  async saveRTSPSetting() {
    const url = document.getElementById('setting-rtsp-url').value.trim();
    if (!url) {
      this.addLog('Vui lòng nhập đường dẫn RTSP hợp lệ!', 'warn');
      return;
    }
    const transportSelect = document.getElementById('setting-rtsp-transport');
    const transport = transportSelect ? transportSelect.value : 'tcp';

    this.showVideoToast('Đang kết nối luồng RTSP...', 'info', 3000);
    this.addLog(`Đang chuyển luồng RTSP sang: ${url} (${transport.toUpperCase()})...`, 'info');

    try {
      const res = await fetch('/api/settings/rtsp', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ rtsp_url: url, transport: transport })
      });
      if (!res.ok) {
        throw new Error(`Mã lỗi HTTP ${res.status}`);
      }
      const data = await res.json();
      this.closeSettingsModal();
      this.addLog(`Đã gửi yêu cầu kết nối RTSP: ${url}`, 'success');
      setTimeout(() => this.reloadVideoStream(), 600);
    } catch (err) {
      this.showVideoToast(`Lỗi kết nối RTSP: ${err.message}`, 'warning', 3500);
      this.addLog(`Lỗi kết nối RTSP: ${err.message}`, 'error');
    }
  }

  async saveMAVLinkSetting() {
    const btnText = document.getElementById('btn-mav-toggle-text');
    if (btnText) btnText.textContent = 'Đang kết nối...';

    let payload = {};

    if (this.connType === 'udp') {
      const ip = document.getElementById('setting-mav-ip').value.trim() || '127.0.0.1';
      const port = parseInt(document.getElementById('setting-mav-port').value, 10) || 14550;
      payload = {
        connection_type: 'udp',
        ip: ip,
        port: port,
        baud: 57600
      };
    } else {
      const portSelect = document.getElementById('setting-mav-serial-port');
      const serialPort = portSelect ? portSelect.value.trim() : '';
      if (!serialPort) {
        if (btnText) btnText.textContent = 'Kết Nối Máy Bay';
        return;
      }
      const baud = parseInt(document.getElementById('setting-mav-baud').value, 10) || 115200;
      payload = {
        connection_type: 'serial',
        serial_port: serialPort,
        baud: baud
      };
    }

    try {
      const res = await fetch('/api/mavlink/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      if (data.connected) {
        this.uavConnected = true;
        this.lastUavConnectedState = true;
        this.updateMavlinkUIState(true);
        this.closeSettingsModal();
      } else {
        if (btnText) btnText.textContent = 'Đang chờ Heartbeat...';
        this.closeSettingsModal();
      }
    } catch (err) {
      this.addLog(`Lỗi kết nối với máy bay: ${err.message}`, 'error');
      if (btnText) btnText.textContent = 'Kết Nối Máy Bay';
    }
  }

  async saveAllParameters() {
    // 1. Takeoff Altitude
    const altInput = document.getElementById('setting-takeoff-alt');
    const valAlt = parseFloat(altInput ? altInput.value : 1.0);
    if (!isNaN(valAlt) && valAlt > 0.2) {
      this.takeoffAltitude = valAlt;
      localStorage.setItem('aerotrack_takeoff_alt', String(valAlt));
    }

    // 2. Deadband Box
    const rawX = parseFloat(document.getElementById('setting-deadband-x').value) || 8;
    const rawY = parseFloat(document.getElementById('setting-deadband-y').value) || 8;
    const dbX = Math.max(0.01, Math.min(0.50, rawX / 100.0));
    const dbY = Math.max(0.01, Math.min(0.50, rawY / 100.0));

    // 3. Gains
    const kpSway = parseFloat(document.getElementById('setting-kp-sway').value) || 0.8;
    const kpClimb = parseFloat(document.getElementById('setting-kp-climb').value) || 0.7;
    const kpYaw = parseFloat(document.getElementById('setting-kp-yaw').value) || 0.6;

    try {
      // Save Deadband
      const resDb = await fetch('/api/settings/deadband', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ deadband_x: dbX, deadband_y: dbY })
      });
      const dataDb = await resDb.json();
      this.deadbandX = dataDb.deadband_x;
      this.deadbandY = dataDb.deadband_y;

      // Save Gains
      await fetch('/api/settings/gains', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          kp_sway: kpSway,
          kp_climb: kpClimb,
          kp_yaw: kpYaw,
          deadband_x: dbX,
          deadband_y: dbY
        })
      });

      this.renderCanvas();
      this.closeSettingsModal();
    } catch (err) {
      this.addLog(`Lỗi khi lưu tham số: ${err.message}`, 'error');
    }
  }

  saveTakeoffSetting() {
    const altInput = document.getElementById('setting-takeoff-alt');
    const val = parseFloat(altInput ? altInput.value : 2.5);
    if (isNaN(val) || val <= 0.2) {
      return;
    }
    this.takeoffAltitude = val;
    localStorage.setItem('aerotrack_takeoff_alt', String(val));
    this.closeSettingsModal();
  }

  // --- DYNAMIC FLIGHT ACTION (COMBINED TAKEOFF / LAND) ---
  isUAVInFlight() {
    // UAV is considered in flight if armed and altitude > 0.3m, or if altitude is significant (> 0.5m)
    return (this.uavArmed && this.uavAltitude > 0.3) || (this.uavAltitude > 0.6);
  }

  updateFlightActionButton() {
    const btn = document.getElementById('btn-uav-flight-action');
    const iconSpan = document.getElementById('flight-action-icon');
    const textSpan = document.getElementById('flight-action-text');
    if (!btn || !textSpan) return;

    const inFlight = this.isUAVInFlight();

    if (inFlight) {
      // Show "Hạ Cánh (Land)"
      btn.className = 'btn btn-flight-action btn-land';
      btn.title = 'UAV đang bay. Nhấn để ra lệnh hạ cánh an toàn (LAND)';
      textSpan.textContent = 'Hạ Cánh (Land)';
      if (iconSpan) {
        iconSpan.innerHTML = `
          <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 5v14M5 12l7 7 7-7" />
          </svg>
        `;
      }
    } else {
      // Show "Cất Cánh (Takeoff)"
      btn.className = 'btn btn-flight-action btn-takeoff';
      btn.title = `UAV đang ở mặt đất. Nhấn để cất cánh tự động lên ${this.takeoffAltitude}m`;
      textSpan.textContent = 'Cất Cánh (Takeoff)';
      if (iconSpan) {
        iconSpan.innerHTML = `
          <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 19V5M5 12l7-7 7 7" />
          </svg>
        `;
      }
    }
  }

  async handleFlightAction() {
    if (!this.uavConnected) {
      this.addLog('Chưa kết nối UAV! Hãy kết nối máy bay trước.', 'warn');
      return;
    }

    if (this.isUAVInFlight()) {
      await this.landUAV();
    } else {
      await this.takeoffUAV();
    }
  }

  async takeoffUAV() {
    if (!this.uavConnected) {
      this.addLog('Chưa kết nối UAV! Không thể thực hiện lệnh cất cánh.', 'warn');
      return;
    }

    if (!confirm(`Xác nhận cất cánh tự động lên độ cao ${this.takeoffAltitude}m?`)) {
      return;
    }

    try {
      const res = await fetch('/api/mavlink/takeoff', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ altitude: this.takeoffAltitude })
      });
      const data = await res.json();
      if (res.ok && data.status === 'ok') {
        this.addLog(`Lệnh cất cánh thành công! Mục tiêu độ cao: ${data.altitude}m`, 'success');
      } else {
        const errMsg = data.detail || 'Lệnh cất cánh thất bại!';
        this.addLog(errMsg, 'error');
      }
    } catch (err) {
      this.addLog(`Lỗi thực hiện cất cánh: ${err.message}`, 'error');
    }
  }

  async landUAV() {
    if (!this.uavConnected) {
      this.addLog('Chưa kết nối UAV! Không thể thực hiện lệnh hạ cánh.', 'warn');
      return;
    }

    if (!confirm('Xác nhận hạ cánh UAV ngay bây giờ?')) {
      return;
    }

    try {
      const res = await fetch('/api/mavlink/land', { method: 'POST' });
      const data = await res.json();
      if (res.ok && data.status === 'ok') {
        this.addLog('Lệnh hạ cánh đã được kích hoạt thành công!', 'warn');
      } else {
        const errMsg = data.detail || 'Không thể gửi lệnh hạ cánh!';
        this.addLog(errMsg, 'error');
      }
    } catch (err) {
      this.addLog(`Lỗi hạ cánh: ${err.message}`, 'error');
    }
  }

  async saveDeadbandSetting() {
    const rawX = parseFloat(document.getElementById('setting-deadband-x').value) || 8;
    const rawY = parseFloat(document.getElementById('setting-deadband-y').value) || 8;

    const dbX = Math.max(0.01, Math.min(0.50, rawX / 100.0));
    const dbY = Math.max(0.01, Math.min(0.50, rawY / 100.0));

    try {
      const res = await fetch('/api/settings/deadband', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ deadband_x: dbX, deadband_y: dbY })
      });
      const data = await res.json();
      this.deadbandX = data.deadband_x;
      this.deadbandY = data.deadband_y;
      this.renderCanvas();
      this.closeSettingsModal();
    } catch (err) {
      this.addLog(`Lỗi cập nhật Deadband: ${err.message}`, 'error');
    }
  }

  async disconnectMAVLink() {
    const btnText = document.getElementById('btn-mav-toggle-text');
    if (btnText) btnText.textContent = 'Đang ngắt kết nối...';

    try {
      const res = await fetch('/api/mavlink/disconnect', { method: 'POST' });
      await res.json();
      this.uavConnected = false;
      this.lastUavConnectedState = false;
      this.updateMavlinkUIState(false);
      this.addLog('Đã ngắt kết nối với máy bay thành công.', 'info');
    } catch (err) {
      this.addLog(`Lỗi ngắt kết nối với máy bay: ${err.message}`, 'error');
      this.updateMavlinkUIState(this.uavConnected);
    }
  }

  async setFlightMode(mode) {
    if (!this.uavConnected) {
      this.addLog('Chưa kết nối UAV! Không thể đổi Flight Mode.', 'warn');
      return;
    }
    try {
      const res = await fetch('/api/mavlink/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode })
      });
      const data = await res.json();
    } catch (err) {
      this.addLog(`Lỗi chuyển flight mode: ${err.message}`, 'error');
    }
  }

  async toggleArm() {
    if (!this.uavConnected) {
      this.addLog('Chưa kết nối UAV! Không thể ARM động cơ.', 'warn');
      return;
    }
    const isArmed = document.getElementById('val-arm-status').textContent.includes('ARMED');
    const targetArm = !isArmed;

    try {
      const res = await fetch('/api/mavlink/arm', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ arm: targetArm })
      });
      const data = await res.json();
    } catch (err) {
      this.addLog(`Lỗi thao tác ARM: ${err.message}`, 'error');
    }
  }

  // --- EMERGENCY STOP / HOVER ---
  async emergencyStop() {
    try {
      await fetch('/api/mavlink/stop', { method: 'POST' });
      await fetch('/api/tracking/cancel', { method: 'POST' }).catch(() => { });
      this.currentMode = 'STANDBY';
      this.isSelectingROI = false;
      this.trackedBbox = null;
      if (this.roiBanner) this.roiBanner.style.display = 'none';
      this.updateModeUI('STANDBY');
      this.renderCanvas();
      this.addLog('Dừng quá trình bám mục tiêu.', 'warn');
    } catch (err) {
      console.error(err);
    }
  }

  // --- MODE SELECTION ---
  async selectMode(mode) {
    if (!this.uavConnected) {
      // this.addLog('Cảnh báo: Chưa kết nối UAV! Hệ thống chỉ xử lý hình ảnh, UAV chưa thể nhận lệnh bay.', 'warn');
    }

    this.currentMode = mode;
    if (mode === 'TRACKING') {
      this.enableROIDraw();
    } else {
      this.isSelectingROI = false;
      this.trackedBbox = null;
      if (this.roiBanner) this.roiBanner.style.display = 'none';
    }
    this.updateModeUI(mode);
    this.renderCanvas();

    try {
      const res = await fetch('/api/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode })
      });
      const data = await res.json();
      this.addLog(`Kích hoạt chế độ: ${data.mode}`, 'info');
    } catch (err) {
      this.addLog(`Lỗi chuyển chế độ: ${err.message}`, 'error');
    }
  }

  updateModeUI(mode) {
    const btnEmergency = document.getElementById('btn-emergency-stop');
    const btnTracking = document.getElementById('btn-mode-tracking');

    if (btnEmergency) btnEmergency.classList.toggle('active-mode', mode === 'STANDBY');
    if (btnTracking) btnTracking.classList.toggle('active-mode', mode === 'TRACKING');

    // Banner only visible in TRACKING mode when drawing ROI
    if (this.roiBanner && mode !== 'TRACKING') {
      this.roiBanner.style.display = 'none';
    }
  }

  // --- ROI DRAWING ON VIDEO ---
  enableROIDraw(e) {
    if (e) e.stopPropagation();
    if (this.currentMode !== 'TRACKING') return;
    this.isSelectingROI = true;
    this.trackedBbox = null;
    if (this.roiBanner) this.roiBanner.style.display = 'flex';
    this.renderCanvas();

    // Notify backend to cancel active tracker immediately
    fetch('/api/tracking/cancel', { method: 'POST' }).catch(() => { });
  }

  onMouseDown(e) {
    if (this.currentMode !== 'TRACKING') return;
    if (e.button !== undefined && e.button !== 0) return;

    const rect = this.canvas.getBoundingClientRect();
    this.isDrawingROI = true;
    this.isSelectingROI = true;
    this.trackedBbox = null;
    this.roiStart = {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top
    };
    this.roiCurrent = { ...this.roiStart };
    if (this.roiBanner) this.roiBanner.style.display = 'flex';
    this.renderCanvas();
  }

  onMouseMove(e) {
    if (!this.isDrawingROI) return;

    const rect = this.canvas.getBoundingClientRect();
    this.roiCurrent = {
      x: Math.max(0, Math.min(rect.width, e.clientX - rect.left)),
      y: Math.max(0, Math.min(rect.height, e.clientY - rect.top))
    };

    this.renderCanvas();
  }

  onMouseUp() {
    if (!this.isDrawingROI || !this.roiStart || !this.roiCurrent) return;
    this.isDrawingROI = false;
    this.roiBanner.style.display = 'none';

    const rect = this.canvas.getBoundingClientRect();
    const x1 = Math.min(this.roiStart.x, this.roiCurrent.x);
    const y1 = Math.min(this.roiStart.y, this.roiCurrent.y);
    const x2 = Math.max(this.roiStart.x, this.roiCurrent.x);
    const y2 = Math.max(this.roiStart.y, this.roiCurrent.y);

    const w = x2 - x1;
    const h = y2 - y1;

    if (w < 15 || h < 15) {
      this.addLog('Vùng khoanh quá nhỏ, vui lòng thử lại.', 'warn');
      this.renderCanvas();
      return;
    }

    const scaleX = this.nativeWidth / rect.width;
    const scaleY = this.nativeHeight / rect.height;

    const nativeBbox = [
      Math.round(x1 * scaleX),
      Math.round(y1 * scaleY),
      Math.round(w * scaleX),
      Math.round(h * scaleY)
    ];

    this.sendROI(nativeBbox);
  }

  // --- RENDERING CANVAS (ROI, Tracked Box, Deadband Box) ---
  renderCanvas() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // If in STANDBY mode: nothing drawn
    if (this.currentMode === 'STANDBY') {
      return;
    }

    const scaleX = this.canvas.width / this.nativeWidth;
    const scaleY = this.canvas.height / this.nativeHeight;

    // 1. Draw Deadband Box at center (TRACKING mode only)
    if (this.currentMode === 'TRACKING') {
      const cx = this.canvas.width / 2;
      const cy = this.canvas.height / 2;
      const dbW = this.deadbandX * this.canvas.width;
      const dbH = this.deadbandY * this.canvas.height;
      const dbX = cx - dbW / 2;
      const dbY = cy - dbH / 2;

      this.ctx.save();
      this.ctx.strokeStyle = 'rgba(56, 189, 248, 0.75)';
      this.ctx.lineWidth = 1.5;
      this.ctx.setLineDash([5, 4]);
      this.ctx.strokeRect(dbX, dbY, dbW, dbH);

      // Center crosshair tick
      this.ctx.setLineDash([]);
      this.ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)';
      this.ctx.lineWidth = 1;
      this.ctx.beginPath();
      this.ctx.moveTo(cx - 7, cy);
      this.ctx.lineTo(cx + 7, cy);
      this.ctx.moveTo(cx, cy - 7);
      this.ctx.lineTo(cx, cy + 7);
      this.ctx.stroke();
      this.ctx.restore();
    }

    // 2. In TRACKING mode: Draw active drag ROI or tracked target box
    if (this.currentMode === 'TRACKING') {
      if (this.isDrawingROI && this.roiStart && this.roiCurrent) {
        // User actively dragging
        const x = Math.min(this.roiStart.x, this.roiCurrent.x);
        const y = Math.min(this.roiStart.y, this.roiCurrent.y);
        const w = Math.abs(this.roiCurrent.x - this.roiStart.x);
        const h = Math.abs(this.roiCurrent.y - this.roiStart.y);

        this.ctx.save();
        this.ctx.fillStyle = 'rgba(56, 189, 248, 0.18)';
        this.ctx.fillRect(x, y, w, h);
        this.ctx.strokeStyle = '#38bdf8';
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(x, y, w, h);
        this.ctx.restore();

      } else if (!this.isSelectingROI && this.trackedBbox) {
        // Backend tracked bounding box
        const [bx, by, bw, bh] = this.trackedBbox;
        const x = bx * scaleX;
        const y = by * scaleY;
        const w = bw * scaleX;
        const h = bh * scaleY;

        this.ctx.save();
        this.ctx.strokeStyle = '#10b981'; // Emerald Green
        this.ctx.lineWidth = 2;
        this.ctx.strokeRect(x, y, w, h);

        // Target corner brackets
        const cl = Math.min(12, w / 4, h / 4);
        this.ctx.strokeStyle = '#34d399';
        this.ctx.lineWidth = 3;
        this.ctx.beginPath();
        // Top-left
        this.ctx.moveTo(x, y + cl); this.ctx.lineTo(x, y); this.ctx.lineTo(x + cl, y);
        // Top-right
        this.ctx.moveTo(x + w - cl, y); this.ctx.lineTo(x + w, y); this.ctx.lineTo(x + w, y + cl);
        // Bottom-left
        this.ctx.moveTo(x, y + h - cl); this.ctx.lineTo(x, y + h); this.ctx.lineTo(x + cl, y + h);
        // Bottom-right
        this.ctx.moveTo(x + w - cl, y + h); this.ctx.lineTo(x + w, y + h); this.ctx.lineTo(x + w, y + h - cl);
        this.ctx.stroke();

        // Target center dot
        this.ctx.fillStyle = '#10b981';
        this.ctx.beginPath();
        this.ctx.arc(x + w / 2, y + h / 2, 3.5, 0, Math.PI * 2);
        this.ctx.fill();

        // Label
        this.ctx.font = 'bold 11px sans-serif';
        this.ctx.fillText('TARGET TRACKING', x, y > 15 ? y - 6 : y + h + 14);
        this.ctx.restore();
      }
    }
  }

  async sendROI(bbox) {
    try {
      // this.addLog(`Khởi tạo bám điểm với vùng: [${bbox.join(', ')}]`, 'info');
      const res = await fetch('/api/tracking/roi', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ bbox })
      });
      const data = await res.json();
      this.isSelectingROI = false;
      if (data.status === 'ok') {
        this.trackedBbox = bbox;
        this.renderCanvas();
        this.addLog('Đã khóa mục tiêu! Bắt đầu tự động bám mục tiêu.', 'success');
      } else {
        this.addLog('Không thể khởi tạo theo dõi mục tiêu.', 'warn');
      }
    } catch (err) {
      this.isSelectingROI = false;
      this.addLog(`Lỗi gửi ROI: ${err.message}`, 'error');
    }
  }

  async cancelManualTracking(e) {
    if (e) e.stopPropagation();
    try {
      await fetch('/api/tracking/cancel', { method: 'POST' });
      this.isSelectingROI = false;
      this.trackedBbox = null;
      if (this.roiBanner) this.roiBanner.style.display = 'none';
      this.renderCanvas();
      this.addLog('Đã hủy bám mục tiêu. Đã dừng điều khiển Gimbal.', 'info');
    } catch (err) {
      console.error(err);
    }
  }

  // --- MANUAL FLIGHT NUDGE (Only active when connected) ---
  async startNudge(direction) {
    if (!this.uavConnected) {
      return;
    }
    try {
      await fetch('/api/mavlink/nudge', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction: direction, speed: 0.5 })
      });
    } catch (err) {
      console.error(err);
    }
  }

  async stopNudge() {
    if (!this.uavConnected) return;
    try {
      await fetch('/api/mavlink/nudge', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction: 'stop', speed: 0.0 })
      });
    } catch (err) {
      console.error(err);
    }
  }

  // --- WEBSOCKET TELEMETRY & HUD ---
  connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

    try {
      this.ws = new WebSocket(wsUrl);

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.updateHUD(data);
        } catch (e) {
          console.error('WS parse error', e);
        }
      };

      this.ws.onclose = () => {
        setTimeout(() => this.connectWebSocket(), 2000);
      };

      this.ws.onerror = () => {
        // Fallback: poll state via HTTP if WebSocket is blocked
        this.pollStateHTTP();
      };
    } catch (err) {
      this.pollStateHTTP();
      setTimeout(() => this.connectWebSocket(), 2000);
    }

    // Standing backup poll every 2s
    if (!this._httpPollTimer) {
      this._httpPollTimer = setInterval(() => {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
          this.pollStateHTTP();
        }
      }, 1500);
    }
  }

  async pollStateHTTP() {
    try {
      const res = await fetch('/api/state');
      if (res.ok) {
        const data = await res.json();
        this.updateHUD(data);
      }
    } catch (e) { }
  }

  updateHUD(data) {
    const vision = data.vision || {};
    const telem = data.telemetry || {};

    // 1. Connection states
    this.uavConnected = !!telem.connected;
    this.rtspConnected = !!vision.rtsp_connected;

    // RTSP stream toast notification (shown briefly then auto-hides)
    const isRtspConnected = this.rtspConnected;
    if (this.lastRtspConnectedState === null) {
      this.lastRtspConnectedState = isRtspConnected;
      if (isRtspConnected) {
        this.showVideoToast('Đã kết nối luồng RTSP', 'success', 3000);
      } else {
        this.showVideoToast('Đang kết nối luồng RTSP...', 'info', 3000);
      }
    } else if (this.lastRtspConnectedState !== isRtspConnected) {
      this.lastRtspConnectedState = isRtspConnected;
      if (isRtspConnected) {
        this.showVideoToast('Đã kết nối luồng RTSP thành công', 'success', 3000);
        this.addLog('Đã kết nối luồng RTSP camera thành công.', 'success');
        this.reloadVideoStream();
      } else {
        this.showVideoToast('Mất kết nối luồng RTSP', 'warning', 3000);
        this.addLog('Mất kết nối luồng RTSP camera.', 'warn');
      }
    }

    // 2. Right Sidebar UAV Telemetry Card
    const valMavConn = document.getElementById('val-mav-conn-status');
    if (valMavConn) {
      if (this.uavConnected) {
        valMavConn.textContent = 'Connected';
        valMavConn.className = 'telem-val text-success';
      } else {
        valMavConn.textContent = 'Disconnect';
        valMavConn.className = 'telem-val text-danger';
      }
    }

    const valMode = document.getElementById('val-mav-mode');
    if (valMode) valMode.textContent = telem.mode || '--';

    const valAlt = document.getElementById('val-altitude');
    if (valAlt) valAlt.textContent = `${(telem.altitude || 0).toFixed(1)} m`;

    const armLabel = document.getElementById('val-arm-status');
    const armBtnText = document.getElementById('btn-arm-text');
    if (armLabel) {
      armLabel.textContent = telem.armed ? 'ARMED' : 'DISARMED';
      armLabel.className = telem.armed ? 'telem-val text-success' : 'telem-val text-warning';
    }
    if (armBtnText) {
      armBtnText.textContent = telem.armed ? 'DISARM ĐỘNG CƠ' : 'ARM ĐỘNG CƠ';
    }

    // Battery bar & voltage
    const batPct = telem.battery_pct || 0;
    const batVolt = telem.battery_volt || 0;
    const valBat = document.getElementById('val-battery');
    if (valBat) valBat.textContent = `${batPct}% (${batVolt.toFixed(1)}V)`;
    const batBar = document.getElementById('battery-bar');
    if (batBar) batBar.style.width = `${batPct}%`;

    // 2.5 Gimbal Telemetry & Status
    const gimbal = data.gimbal || {};
    this.gimbalConnected = !!gimbal.connected;

    const valGimbalStatus = document.getElementById('val-gimbal-status');
    if (valGimbalStatus) {
      if (this.gimbalConnected) {
        if (gimbal.is_tracking) {
          valGimbalStatus.textContent = 'Tracking';
          valGimbalStatus.className = 'telem-val text-primary';
        } else {
          valGimbalStatus.textContent = 'Connected';
          valGimbalStatus.className = 'telem-val text-success';
        }
      } else {
        valGimbalStatus.textContent = 'Disconnected';
        valGimbalStatus.className = 'telem-val text-danger';
      }
    }

    const valGimbalAngles = document.getElementById('val-gimbal-angles');
    if (valGimbalAngles) {
      const p = (gimbal.pitch !== undefined) ? Number(gimbal.pitch).toFixed(1) : '0.0';
      const y = (gimbal.yaw !== undefined) ? Number(gimbal.yaw).toFixed(1) : '0.0';
      valGimbalAngles.textContent = `${p}° / ${y}°`;
    }

    // Modal Gimbal tab status & controls
    const gimbalConnText = document.getElementById('gimbal-conn-text');
    const btnGimbalToggle = document.getElementById('btn-gimbal-toggle-conn');
    const btnGimbalText = document.getElementById('btn-gimbal-toggle-text');
    const gimbalPortSelect = document.getElementById('setting-gimbal-port');
    const gimbalBaudSelect = document.getElementById('setting-gimbal-baud');
    const btnScanGimbalPorts = document.getElementById('btn-scan-gimbal-ports');
    if (gimbalConnText) {
      if (this.gimbalConnected) {
        const baudVal = gimbal.baudrate || gimbal.baud || 115200;
        gimbalConnText.innerHTML = `<span>🟢 Đang kết nối: <strong>${gimbal.port || ''}</strong> (${baudVal} baud)</span>`;
        gimbalConnText.style.borderColor = 'rgba(16, 185, 129, 0.4)';
      } else {
        gimbalConnText.innerHTML = `<span>Trạng thái: Chưa kết nối Gimbal</span>`;
        gimbalConnText.style.borderColor = '';
      }
    }
    if (btnGimbalToggle) {
      btnGimbalToggle.className = this.gimbalConnected ? 'btn btn-danger btn-block' : 'btn btn-primary btn-block';
    }
    if (btnGimbalText) {
      btnGimbalText.textContent = this.gimbalConnected ? 'Ngắt Kết Nối Gimbal' : 'Kết Nối Gimbal';
    }
    if (gimbalPortSelect) gimbalPortSelect.disabled = this.gimbalConnected;
    if (gimbalBaudSelect) gimbalBaudSelect.disabled = this.gimbalConnected;
    if (btnScanGimbalPorts) {
      btnScanGimbalPorts.disabled = this.gimbalConnected;
      btnScanGimbalPorts.style.pointerEvents = this.gimbalConnected ? 'none' : 'auto';
      btnScanGimbalPorts.style.opacity = this.gimbalConnected ? '0.35' : '1';
    }

    // 3. Nudge Pad Lock State
    const nudgeWrapper = document.getElementById('nudge-pad-wrapper');
    const lockIndicator = document.getElementById('manual-lock-indicator');
    if (nudgeWrapper && lockIndicator) {
      nudgeWrapper.classList.toggle('locked', !this.uavConnected);
      lockIndicator.textContent = this.uavConnected ? '🟢 Sẵn sàng điều khiển' : '🔒 Cần kết nối UAV';
      lockIndicator.style.color = this.uavConnected ? '#10b981' : '#f59e0b';
    }

    // 3. MAVLink Settings State & Input Lock (Only update on state change)
    if (this.lastUavConnectedState !== this.uavConnected) {
      this.lastUavConnectedState = this.uavConnected;
      this.updateMavlinkUIState(this.uavConnected);
    }

    // Keep port and baud dropdowns synced to active telemetry
    if (this.uavConnected && telem.connection_str) {
      const portSelect = document.getElementById('setting-mav-serial-port');
      if (portSelect && !telem.connection_str.startsWith('udp')) {
        let optFound = false;
        for (let i = 0; i < portSelect.options.length; i++) {
          if (portSelect.options[i].value === telem.connection_str) {
            optFound = true;
            break;
          }
        }
        if (!optFound) {
          const opt = document.createElement('option');
          opt.value = telem.connection_str;
          opt.textContent = `${telem.connection_str} (Đang kết nối)`;
          portSelect.appendChild(opt);
        }
        portSelect.value = telem.connection_str;
      }
      if (telem.baud) {
        const baudSelect = document.getElementById('setting-mav-baud');
        if (baudSelect) baudSelect.value = String(telem.baud);
      }
    }

    // 4. Update Deadband & Tracking Bounding Box from Vision
    if (vision.deadband_x !== undefined) this.deadbandX = vision.deadband_x;
    if (vision.deadband_y !== undefined) this.deadbandY = vision.deadband_y;
    this.isCentered = !!vision.is_centered;
    this.dwellProgress = vision.dwell_progress || 0.0;

    if (this.currentMode === 'TRACKING') {
      if (this.isSelectingROI || this.isDrawingROI) {
        this.trackedBbox = null;
      } else if (vision.tracking_bbox) {
        this.trackedBbox = vision.tracking_bbox;
      } else {
        this.trackedBbox = null;
      }
    } else {
      this.isSelectingROI = false;
      this.trackedBbox = null;
    }

    // 6. Update Dynamic Flight Action Button (Takeoff vs Land)
    this.uavArmed = !!telem.armed;
    this.uavAltitude = parseFloat(telem.altitude || telem.relative_alt || 0.0);
    this.uavMode = telem.mode || '---';
    this.updateFlightActionButton();

    // 7. Process Incoming STATUSTEXT Messages from UAV
    if (Array.isArray(telem.statustexts) && telem.statustexts.length > 0) {
      telem.statustexts.forEach(st => {
        // Unique key: text + timestamp to prevent duplicate logs
        const key = `${st.time}_${st.text}`;
        if (!this._processedStatusTexts.has(key)) {
          this._processedStatusTexts.add(key);
          // Limit cache size
          if (this._processedStatusTexts.size > 200) {
            const arr = Array.from(this._processedStatusTexts);
            this._processedStatusTexts = new Set(arr.slice(-100));
          }
          const badge = st.severity_name ? `[UAV ${st.severity_name}]` : '[UAV]';
          this.addLog(`${badge} ${st.text}`, st.type || 'info');
        }
      });
    }

    // Re-render canvas
    if (!this.isDrawingROI) {
      this.renderCanvas();
    }
  }

  addLog(msg, type = 'info') {
    const consoleBox = document.getElementById('log-console');
    if (!consoleBox) return;

    const timeStr = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-row log-${type}`;
    entry.textContent = `[${timeStr}] ${msg}`;
    consoleBox.appendChild(entry);
    consoleBox.scrollTop = consoleBox.scrollHeight;
  }

  clearLogs() {
    const consoleBox = document.getElementById('log-console');
    if (consoleBox) consoleBox.innerHTML = '';
  }
}

// Start application when DOM is ready
window.addEventListener('DOMContentLoaded', () => {
  window.app = new AeroTrackGCS();
});