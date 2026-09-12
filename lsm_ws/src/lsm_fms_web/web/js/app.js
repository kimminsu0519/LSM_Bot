/**
 * LSM FMS Command Center & Topological Graph Calibrator
 * Author: Antigravity AI
 * Version: 1.0.8
 */

class FMSApp {
  constructor() {
    // Data States
    this.yamlData = null;
    this.waypoints = {};
    this.metadata = {};
    this.mapImage = new Image();
    this.mapLoaded = false;
    this.selectedWpId = null;
    this.hoveredWpId = null;
    this.currentTheme = localStorage.getItem('lsm_fms_theme') || 'dark';
    this.sidebarCollapsed = true;

    // View Transforms (Pan & Zoom)
    this.view = {
      panX: 0,
      panY: 0,
      zoom: 1.0,
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0
    };

    // Calibration Parameters
    // World Origin (0,0) = Drawio Canvas (160pt, 880pt)
    this.calibration = {
      offsetX: 22.8,
      offsetY: 12.65,
      scale: 1.0,
      yaw: 0.0,
      pxPerMeter: 100.0 // 1m = 100pt Uniform Scale
    };

    // Display Toggles (Default showUnconnected = false, showLabels = false by user request)
    this.showEdges = true;
    this.showArrows = true;
    this.showLabels = false; // 기본 설정: 웨이포인트 이형 명칭 숨김
    this.showUnconnected = false; // 기본: 고립 노드 숨김
    this.activeFilters = new Set(['workstation', 'charger', 'node']); // 다중 선택 필터 (기본: 모두 선택)

    this.incomingEdgesCount = {};

    // Robot Telemetry State (lsm_bot_1)
    this.robotState = {
      id: 'lsm_bot_1',
      x: 0.0,
      y: 0.0,
      yaw_deg: 0.0,
      v: 0.0,
      w: 0.0,
      battery: 100,
      status: 'IDLE',
      covX: 0.002,
      covY: 0.002,
      covYaw: 0.01
    };
    this.ws = null;
    this.wsConnected = false;
    this.hasReceivedTelemetry = false;

    // Elements
    this.canvas = document.getElementById('map-canvas');
    this.ctx = this.canvas.getContext('2d');
    this.container = document.getElementById('canvas-container');

    this.init();
  }

  async init() {
    this.applyTheme(this.currentTheme);
    this.loadSavedCalibration();
    this.loadSavedViewOptions();
    this.setupEventListeners();
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());

    await Promise.all([this.loadMapImage(), this.loadYAMLData()]);

    this.fitGraphToViewport();
    this.initWebSocket();
    this.startRenderLoop();
    this.render();
  }

  loadSavedViewOptions() {
    const saved = localStorage.getItem('lsm_fms_view_options');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        if (typeof parsed.showEdges === 'boolean') this.showEdges = parsed.showEdges;
        if (typeof parsed.showArrows === 'boolean') this.showArrows = parsed.showArrows;
        if (typeof parsed.showLabels === 'boolean') this.showLabels = parsed.showLabels;
        if (typeof parsed.showUnconnected === 'boolean') this.showUnconnected = parsed.showUnconnected;
      } catch (e) {}
    }
    this.updateToggleUI();
  }

  saveViewOptions() {
    const options = {
      showEdges: this.showEdges,
      showArrows: this.showArrows,
      showLabels: this.showLabels,
      showUnconnected: this.showUnconnected
    };
    localStorage.setItem('lsm_fms_view_options', JSON.stringify(options));
  }

  updateToggleUI() {
    const chkEdges = document.getElementById('chk-show-edges');
    const chkArrows = document.getElementById('chk-show-arrows');
    const chkLabels = document.getElementById('chk-show-labels');
    const chkUnconnected = document.getElementById('chk-show-unconnected');

    if (chkEdges) chkEdges.checked = this.showEdges;
    if (chkArrows) chkArrows.checked = this.showArrows;
    if (chkLabels) chkLabels.checked = this.showLabels;
    if (chkUnconnected) chkUnconnected.checked = this.showUnconnected;
  }

  applyTheme(theme) {
    this.currentTheme = 'light';
    document.documentElement.setAttribute('data-theme', 'light');
    localStorage.setItem('lsm_fms_theme', 'light');
  }

  toggleSidebar() {
    this.sidebarCollapsed = !this.sidebarCollapsed;
    this.updateSidebarUI();
  }

  openSidebar() {
    if (this.sidebarCollapsed) {
      this.sidebarCollapsed = false;
      this.updateSidebarUI();
    }
  }

  updateSidebarUI() {
    const sidebar = document.getElementById('main-sidebar');
    const icon = document.getElementById('icon-sidebar');
    const label = document.getElementById('label-sidebar');

    if (this.sidebarCollapsed) {
      sidebar.classList.add('collapsed');
      if (icon) icon.className = 'fa-solid fa-angles-left';
      if (label) label.textContent = 'Open Panel';
    } else {
      sidebar.classList.remove('collapsed');
      if (icon) icon.className = 'fa-solid fa-angles-right';
      if (label) label.textContent = 'Close Panel';
    }
  }

  switchToTab(tabId) {
    document.querySelectorAll('.tab-btn').forEach(btn => {
      btn.classList.toggle('active', btn.getAttribute('data-tab') === tabId);
    });
    document.querySelectorAll('.tab-content').forEach(content => {
      content.classList.toggle('active', content.id === tabId);
    });
  }

  loadSavedCalibration() {
    const saved = localStorage.getItem('lsm_fms_calibration');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        this.calibration = { ...this.calibration, ...parsed };
        this.updateCalibrationUI();
      } catch (e) {
        console.error('Failed to parse saved calibration:', e);
      }
    }
  }

  saveCalibration() {
    localStorage.setItem('lsm_fms_calibration', JSON.stringify(this.calibration));
    alert('Calibration parameters successfully saved to LocalStorage!');
  }

  updateCalibrationUI() {
    const setVal = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.value = val;
    };
    setVal('num-offset-x', this.calibration.offsetX);
    setVal('rng-offset-x', this.calibration.offsetX);
    setVal('num-offset-y', this.calibration.offsetY);
    setVal('rng-offset-y', this.calibration.offsetY);
    setVal('num-scale', this.calibration.scale);
    setVal('rng-scale', this.calibration.scale);
    setVal('num-yaw', this.calibration.yaw);
    setVal('rng-yaw', this.calibration.yaw);
  }

  async loadMapImage() {
    return new Promise((resolve) => {
      let isDone = false;
      const done = () => {
        if (!isDone) {
          isDone = true;
          resolve();
        }
      };

      this.mapImage.onload = () => {
        this.mapLoaded = true;
        console.log(`Map Image Loaded: ${this.mapImage.width}x${this.mapImage.height} pt`);
        done();
      };
      this.mapImage.onerror = () => {
        console.warn('Map Image load failed from assets/창고맵_이미지용_점제거.png');
        this.mapLoaded = false;
        done();
      };
      setTimeout(() => {
        if (!isDone) {
          console.warn('Map image load timeout fallback triggered');
          done();
        }
      }, 1500);

      this.mapImage.src = 'assets/창고맵_이미지용_점제거.png';
    });
  }

  async loadYAMLData() {
    try {
      let data = null;
      try {
        const res = await fetch('config/waypoints_graph2.json');
        if (res.ok) {
          data = await res.json();
        }
      } catch (e) {
        console.warn('JSON fetch failed, attempting YAML fallback...', e);
      }

      if (!data) {
        const res = await fetch('config/waypoints_graph2.yaml');
        const text = await res.text();
        if (window.jsyaml) {
          data = jsyaml.load(text);
        }
      }

      this.yamlData = data || {};
      this.metadata = this.yamlData.metadata || {};
      this.waypoints = this.yamlData.waypoints || {};

      this.calculateEdgeDegree();
      this.updateStats();
      this.populateWaypointsList();
      this.fitGraphToViewport();
      this.render();
      console.log(`Successfully loaded ${Object.keys(this.waypoints).length} waypoints.`);
    } catch (err) {
      console.error('Error loading waypoints graph data:', err);
    }
  }

  calculateEdgeDegree() {
    this.incomingEdgesCount = {};
    Object.entries(this.waypoints).forEach(([id, wp]) => {
      if (wp.connected_to) {
        wp.connected_to.forEach(edge => {
          this.incomingEdgesCount[edge.target] = (this.incomingEdgesCount[edge.target] || 0) + 1;
        });
      }
    });
  }

  isUnconnected(id, wp) {
    const outgoing = (wp.connected_to && wp.connected_to.length > 0) ? wp.connected_to.length : 0;
    const incoming = this.incomingEdgesCount[id] || 0;
    return outgoing === 0 && incoming === 0;
  }

  isCharger(id, wp) {
    const typeStr = (wp.type || '').toLowerCase();
    const idStr = id.toLowerCase();
    return typeStr.includes('charger') || typeStr.includes('charging') || idStr.includes('wp-ch') || idStr.includes('charger');
  }

  matchesFilter(id, wp) {
    const isChargerNode = this.isCharger(id, wp);
    const isWS = wp.type === 'workstation';

    if (isWS) return this.activeFilters.has('workstation');
    if (isChargerNode) return this.activeFilters.has('charger');
    return this.activeFilters.has('node');
  }

  autoFitGraph(shouldRender = true) {
    this.calibration.offsetX = 22.8;
    this.calibration.offsetY = 12.65;
    this.calibration.scale = 1.0;
    this.calibration.yaw = 0.0;

    this.updateCalibrationUI();
    if (shouldRender) {
      this.centerView();
      this.render();
    }
  }

  updateStats() {
    const total = Object.keys(this.waypoints).length;
    let wsCount = 0;
    let chargerCount = 0;
    let nodeCount = 0;

    Object.entries(this.waypoints).forEach(([id, wp]) => {
      if (this.isCharger(id, wp)) chargerCount++;
      else if (wp.type === 'workstation') wsCount++;
      else nodeCount++;
    });

    const setTxt = (id, txt) => {
      const el = document.getElementById(id);
      if (el) el.textContent = txt;
    };

    setTxt('stat-total-wp', total);
    setTxt('stat-workstations', wsCount);
    setTxt('stat-chargers', chargerCount);
    setTxt('stat-nodes', nodeCount);
    setTxt('cnt-all', total);
    setTxt('cnt-ws', wsCount);
    setTxt('cnt-charger', chargerCount);
    setTxt('cnt-node', nodeCount);
  }

  resizeCanvas() {
    this.canvas.width = this.container.clientWidth;
    this.canvas.height = this.container.clientHeight;
    this.render();
  }

  centerView() {
    this.fitGraphToViewport(true);
  }

  saveViewState() {
    try {
      localStorage.setItem('lsm_fms_view_state', JSON.stringify({
        zoom: this.view.zoom,
        panX: this.view.panX,
        panY: this.view.panY
      }));
    } catch (e) {}
  }

  loadSavedViewState() {
    try {
      const saved = localStorage.getItem('lsm_fms_view_state');
      if (saved) {
        const parsed = JSON.parse(saved);
        if (parsed && typeof parsed.zoom === 'number') {
          this.view.zoom = parsed.zoom;
          this.view.panX = parsed.panX;
          this.view.panY = parsed.panY;
          this.updateZoomBadge();
          return true;
        }
      }
    } catch (e) {}
    return false;
  }

  fitGraphToViewport(forceReset = false) {
    if (!forceReset && this.loadSavedViewState()) {
      return;
    }

    // Default Zoomed-In View (~87% zoom, centered around active warehouse aisles & charging pad)
    const targetZoom = 0.87;
    this.view.zoom = targetZoom;

    let minPxX = Infinity, maxPxX = -Infinity;
    let minPxY = Infinity, maxPxY = -Infinity;

    if (this.waypoints && Object.keys(this.waypoints).length > 0) {
      Object.values(this.waypoints).forEach(wp => {
        const pos = this.getWaypointUserPose(wp);
        const imgPos = this.worldToImagePixel(pos.x, pos.y);
        if (imgPos.x < minPxX) minPxX = imgPos.x;
        if (imgPos.x > maxPxX) maxPxX = imgPos.x;
        if (imgPos.y < minPxY) minPxY = imgPos.y;
        if (imgPos.y > maxPxY) maxPxY = imgPos.y;
      });
    }

    const centerGraphPxX = (minPxX !== Infinity) ? (minPxX + maxPxX) / 2 : 550.0;
    const centerGraphPxY = (minPxY !== Infinity) ? (minPxY + maxPxY) / 2 : 550.0;

    this.view.panX = (this.canvas.width / 2) - (centerGraphPxX * targetZoom);
    this.view.panY = (this.canvas.height / 2) - (centerGraphPxY * targetZoom);

    this.updateZoomBadge();
    this.saveViewState();
  }

  updateZoomBadge() {
    const badge = document.getElementById('zoom-level');
    if (badge) badge.textContent = `${Math.round(this.view.zoom * 100)}%`;
  }

  getWaypointUserPose(wp) {
    if (wp.user_pose && typeof wp.user_pose.x === 'number') {
      return { x: wp.user_pose.x, y: wp.user_pose.y, yaw: wp.user_pose.yaw_deg || 0.0 };
    }
    if (wp.pose && wp.pose.position) {
      return {
        x: wp.pose.position.x,
        y: wp.pose.position.y,
        yaw: 0.0
      };
    }
    return { x: 0, y: 0, yaw: 0.0 };
  }

  worldToImagePixel(wx, wy) {
    let calX = wx + parseFloat(this.calibration.offsetX);
    let calY = wy + parseFloat(this.calibration.offsetY);

    if (this.calibration.yaw !== 0) {
      const rad = (this.calibration.yaw * Math.PI) / 180;
      const rx = calX * Math.cos(rad) - calY * Math.sin(rad);
      const ry = calX * Math.sin(rad) + calY * Math.cos(rad);
      calX = rx;
      calY = ry;
    }

    const scale = parseFloat(this.calibration.scale);
    const pkm = this.calibration.pxPerMeter * scale;

    const imgHeight = this.mapLoaded ? this.mapImage.height : 760;

    const px = calX * pkm;
    const py = imgHeight - calY * pkm;

    return { x: px, y: py };
  }

  imagePixelToScreen(px, py) {
    return {
      x: px * this.view.zoom + this.view.panX,
      y: py * this.view.zoom + this.view.panY
    };
  }

  screenToWorld(sx, sy) {
    const px = (sx - this.view.panX) / this.view.zoom;
    const py = (sy - this.view.panY) / this.view.zoom;

    const imgHeight = this.mapLoaded ? this.mapImage.height : 760;
    const scale = parseFloat(this.calibration.scale);
    const pkm = this.calibration.pxPerMeter * scale;

    let calX = px / pkm;
    let calY = (imgHeight - py) / pkm;

    if (this.calibration.yaw !== 0) {
      const rad = (-this.calibration.yaw * Math.PI) / 180;
      const rx = calX * Math.cos(rad) - calY * Math.sin(rad);
      const ry = calX * Math.sin(rad) + calY * Math.cos(rad);
      calX = rx;
      calY = ry;
    }

    const wx = calX - parseFloat(this.calibration.offsetX);
    const wy = calY - parseFloat(this.calibration.offsetY);

    return { wx, wy };
  }

  render() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    if (this.mapLoaded) {
      this.ctx.drawImage(
        this.mapImage,
        this.view.panX,
        this.view.panY,
        this.mapImage.width * this.view.zoom,
        this.mapImage.height * this.view.zoom
      );
    }

    if (this.showEdges) {
      this.renderEdges();
    }

    this.renderNodes();
    this.drawRobotState();
  }

  initWebSocket() {
    const wsUrl = `ws://${window.location.hostname || 'localhost'}:9090`;
    try {
      this.ws = new WebSocket(wsUrl);
      this.ws.onopen = () => {
        this.wsConnected = true;
        console.log('Operator WebSocket connected:', wsUrl);
      };
      this.ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === 'telemetry') {
            const data = msg.data || msg;
            if (data.has_pose !== false) {
              this.hasReceivedTelemetry = true;
            }
            this.robotState.has_pose = data.has_pose ?? this.robotState.has_pose;
            this.robotState.x = data.x ?? this.robotState.x;
            this.robotState.y = data.y ?? this.robotState.y;
            this.robotState.ros_x = data.ros_x ?? this.robotState.ros_x;
            this.robotState.ros_y = data.ros_y ?? this.robotState.ros_y;
            this.robotState.yaw_deg = data.yaw_deg ?? this.robotState.yaw_deg;
            this.robotState.v = data.v ?? this.robotState.v;
            this.robotState.w = data.w ?? this.robotState.w;
            this.robotState.battery = data.battery ?? this.robotState.battery;
            this.robotState.status = data.status || this.robotState.status;
            if (data.covX !== undefined) this.robotState.covX = data.covX;
            if (data.covY !== undefined) this.robotState.covY = data.covY;
            if (data.covYaw !== undefined) this.robotState.covYaw = data.covYaw;
            if (data.covariance) {
              this.robotState.covX = data.covariance.x ?? this.robotState.covX;
              this.robotState.covY = data.covariance.y ?? this.robotState.covY;
              this.robotState.covYaw = data.covariance.yaw ?? this.robotState.covYaw;
            }
            this.render();
          }
        } catch (e) {
          console.warn('Operator WS JSON error:', e);
        }
      };
      this.ws.onclose = () => {
        this.wsConnected = false;
        setTimeout(() => this.initWebSocket(), 3000);
      };
    } catch (e) {
      console.warn('WebSocket failed to initialize in Operator Mode:', e);
    }
  }

  drawRobotState() {
    if (!this.robotState || !this.hasReceivedTelemetry || this.robotState.has_pose === false) return;
    const imgPos = this.worldToImagePixel(this.robotState.x, this.robotState.y);
    const pt = this.imagePixelToScreen(imgPos.x, imgPos.y);
    const rad = (-this.robotState.yaw_deg * Math.PI) / 180;

    // 1. AMCL Covariance Ellipse
    const scale = parseFloat(this.calibration.scale);
    const covPxX = (this.robotState.covX * 100) * scale * this.view.zoom;
    const covPxY = (this.robotState.covY * 100) * scale * this.view.zoom;

    this.ctx.save();
    this.ctx.translate(pt.x, pt.y);
    this.ctx.beginPath();
    this.ctx.ellipse(0, 0, Math.max(covPxX, 8), Math.max(covPxY, 8), 0, 0, Math.PI * 2);
    this.ctx.fillStyle = 'rgba(13, 148, 136, 0.18)';
    this.ctx.fill();
    this.ctx.strokeStyle = 'rgba(13, 148, 136, 0.7)';
    this.ctx.lineWidth = 1.5;
    this.ctx.stroke();
    this.ctx.restore();

    // 2. Robot Directional Marker (Teal Triangle)
    this.ctx.save();
    this.ctx.translate(pt.x, pt.y);
    this.ctx.rotate(rad);

    const size = 12 * this.view.zoom;
    this.ctx.beginPath();
    this.ctx.moveTo(size, 0);
    this.ctx.lineTo(-size * 0.8, -size * 0.6);
    this.ctx.lineTo(-size * 0.4, 0);
    this.ctx.lineTo(-size * 0.8, size * 0.6);
    this.ctx.closePath();

    this.ctx.fillStyle = '#0d9488';
    this.ctx.fill();
    this.ctx.strokeStyle = '#ffffff';
    this.ctx.lineWidth = 2 * this.view.zoom;
    this.ctx.stroke();
    this.ctx.restore();
  }

  renderEdges() {
    this.ctx.lineWidth = Math.max(1, 1.2 * this.view.zoom);
    this.ctx.strokeStyle = this.currentTheme === 'dark' 
      ? 'rgba(0, 243, 255, 0.25)' 
      : 'rgba(2, 132, 199, 0.3)';
    this.ctx.fillStyle = this.currentTheme === 'dark' 
      ? 'rgba(0, 243, 255, 0.45)' 
      : 'rgba(2, 132, 199, 0.5)';

    const renderedEdgeKeys = new Set();

    Object.entries(this.waypoints).forEach(([id, wp]) => {
      if (!this.matchesFilter(id, wp)) return;
      if (!this.showUnconnected && this.isUnconnected(id, wp)) return;
      if (!wp.connected_to || wp.connected_to.length === 0) return;

      const pos = this.getWaypointUserPose(wp);
      const srcImg = this.worldToImagePixel(pos.x, pos.y);
      const srcScreen = this.imagePixelToScreen(srcImg.x, srcImg.y);
      const isSrcWS = wp.type === 'workstation' || this.isCharger(id, wp);

      wp.connected_to.forEach(edge => {
        const targetId = edge.target;
        const targetWp = this.waypoints[targetId];
        if (targetWp) {
          if (!this.matchesFilter(targetId, targetWp)) return;
          if (!this.showUnconnected && this.isUnconnected(targetId, targetWp)) return;

          const edgeKey = id < targetId ? `${id}__${targetId}` : `${targetId}__${id}`;
          const dirStr = (edge.direction || '').toLowerCase();
          const isTwoWay = dirStr === 'bidirectional' || dirStr === 'two_way' || dirStr === 'both' || dirStr === 'bi';

          const tPos = this.getWaypointUserPose(targetWp);
          const tgtImg = this.worldToImagePixel(tPos.x, tPos.y);
          const tgtScreen = this.imagePixelToScreen(tgtImg.x, tgtImg.y);
          const isTgtWS = targetWp.type === 'workstation' || this.isCharger(targetId, targetWp);

          // Draw Line
          this.ctx.beginPath();
          this.ctx.moveTo(srcScreen.x, srcScreen.y);
          this.ctx.lineTo(tgtScreen.x, tgtScreen.y);
          this.ctx.stroke();

          // Draw Arrowhead at Edge Endpoints
          if (this.showArrows) {
            if (!renderedEdgeKeys.has(edgeKey)) {
              this.drawEdgeArrowheads(
                srcScreen.x, srcScreen.y, isSrcWS,
                tgtScreen.x, tgtScreen.y, isTgtWS,
                isTwoWay
              );
              renderedEdgeKeys.add(edgeKey);
            }
          }
        }
      });
    });
  }

  drawEdgeArrowheads(x1, y1, isSrcWS, x2, y2, isTgtWS, isTwoWay) {
    const dx = x2 - x1;
    const dy = y2 - y1;
    const dist = Math.hypot(dx, dy);
    if (dist < 10) return;

    const angle = Math.atan2(dy, dx);
    const headLen = Math.max(5, 7.5 * Math.sqrt(this.view.zoom));

    const rSrc = Math.max(3, (isSrcWS ? 6.5 : 4.0) * Math.sqrt(this.view.zoom));
    const rTgt = Math.max(3, (isTgtWS ? 6.5 : 4.0) * Math.sqrt(this.view.zoom));

    // 1. Forward Arrowhead (Near Target, pointing towards Target)
    const fwdDist = dist - rTgt - 3;
    if (fwdDist > rSrc + 5) {
      const tipX = x1 + fwdDist * Math.cos(angle);
      const tipY = y1 + fwdDist * Math.sin(angle);

      this.ctx.beginPath();
      this.ctx.moveTo(tipX, tipY);
      this.ctx.lineTo(
        tipX - headLen * Math.cos(angle - Math.PI / 6),
        tipY - headLen * Math.sin(angle - Math.PI / 6)
      );
      this.ctx.lineTo(
        tipX - headLen * Math.cos(angle + Math.PI / 6),
        tipY - headLen * Math.sin(angle + Math.PI / 6)
      );
      this.ctx.closePath();
      this.ctx.fill();
    }

    // 2. Backward Arrowhead (Near Source, pointing towards Source if Bidirectional)
    if (isTwoWay) {
      const bwdDist = rSrc + 3;
      if (dist - bwdDist > rTgt + 5) {
        const tipX = x1 + bwdDist * Math.cos(angle);
        const tipY = y1 + bwdDist * Math.sin(angle);
        const revAngle = angle + Math.PI;

        this.ctx.beginPath();
        this.ctx.moveTo(tipX, tipY);
        this.ctx.lineTo(
          tipX - headLen * Math.cos(revAngle - Math.PI / 6),
          tipY - headLen * Math.sin(revAngle - Math.PI / 6)
        );
        this.ctx.lineTo(
          tipX - headLen * Math.cos(revAngle + Math.PI / 6),
          tipY - headLen * Math.sin(revAngle + Math.PI / 6)
        );
        this.ctx.closePath();
        this.ctx.fill();
      }
    }
  }

  renderNodes() {
    Object.entries(this.waypoints).forEach(([id, wp]) => {
      if (!this.matchesFilter(id, wp)) return;
      if (!this.showUnconnected && this.isUnconnected(id, wp)) return;

      const pos = this.getWaypointUserPose(wp);
      const imgPos = this.worldToImagePixel(pos.x, pos.y);
      const screenPos = this.imagePixelToScreen(imgPos.x, imgPos.y);

      const isSelected = id === this.selectedWpId;
      const isHovered = id === this.hoveredWpId;
      const isWS = wp.type === 'workstation';
      const isChargerNode = this.isCharger(id, wp);

      const baseRadius = Math.max(3, (isWS || isChargerNode ? 6.5 : 4.0) * Math.sqrt(this.view.zoom));
      
      // 클릭/선택 시 노드 1.5배 확대 (빛나는 글로우 대신 무난하고 깔끔한 크기 확대 + 외곽 링)
      const radius = isSelected ? baseRadius * 1.5 : (isHovered ? baseRadius * 1.25 : baseRadius);

      this.ctx.shadowBlur = 0; // 강한 번짐 글로우 제거

      // 선택/호버 시 외곽 링 Stroke
      if (isSelected) {
        const ringRadius = radius + Math.max(3, 4.0 * Math.sqrt(this.view.zoom));
        this.ctx.beginPath();
        this.ctx.arc(screenPos.x, screenPos.y, ringRadius, 0, 2 * Math.PI);
        this.ctx.lineWidth = Math.max(1.5, 2.2 * Math.sqrt(this.view.zoom));
        this.ctx.strokeStyle = this.currentTheme === 'dark' ? '#00f3ff' : '#0284c7';
        this.ctx.stroke();
      } else if (isHovered) {
        const ringRadius = radius + Math.max(2, 2.5 * Math.sqrt(this.view.zoom));
        this.ctx.beginPath();
        this.ctx.arc(screenPos.x, screenPos.y, ringRadius, 0, 2 * Math.PI);
        this.ctx.lineWidth = 1.5;
        this.ctx.strokeStyle = 'rgba(0, 243, 255, 0.45)';
        this.ctx.stroke();
      }

      // Draw Main Node Circle
      this.ctx.beginPath();
      this.ctx.arc(screenPos.x, screenPos.y, radius, 0, 2 * Math.PI);

      if (isChargerNode) {
        this.ctx.fillStyle = this.currentTheme === 'dark' ? '#ffb700' : '#d97706';
      } else if (isWS) {
        this.ctx.fillStyle = this.currentTheme === 'dark' ? '#00ff9d' : '#059669';
      } else {
        this.ctx.fillStyle = wp.status === 'active' 
          ? (this.currentTheme === 'dark' ? '#00f3ff' : '#0284c7') 
          : '#64748b';
      }

      this.ctx.fill();

      if (isSelected || isHovered) {
        this.ctx.lineWidth = Math.max(1.5, 2.0 * Math.sqrt(this.view.zoom));
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.stroke();
      }

      // Draw Docking Orientation Pointer ONLY for Workstation & Charger Nodes!
      if ((isWS || isChargerNode) && typeof pos.yaw === 'number') {
        const pointerLen = radius + Math.max(6, 10 * Math.sqrt(this.view.zoom));
        const yawRad = -(pos.yaw * Math.PI) / 180;
        const targetX = screenPos.x + pointerLen * Math.cos(yawRad);
        const targetY = screenPos.y + pointerLen * Math.sin(yawRad);

        this.ctx.beginPath();
        this.ctx.moveTo(screenPos.x, screenPos.y);
        this.ctx.lineTo(targetX, targetY);
        this.ctx.lineWidth = Math.max(1.5, 2.5 * this.view.zoom);
        this.ctx.strokeStyle = isChargerNode ? '#ffb700' : '#00ff9d';
        this.ctx.stroke();

        const tipSize = Math.max(4, 6 * Math.sqrt(this.view.zoom));
        this.ctx.fillStyle = isChargerNode ? '#ffb700' : '#00ff9d';
        this.ctx.beginPath();
        this.ctx.moveTo(targetX, targetY);
        this.ctx.lineTo(
          targetX - tipSize * Math.cos(yawRad - Math.PI / 6),
          targetY - tipSize * Math.sin(yawRad - Math.PI / 6)
        );
        this.ctx.lineTo(
          targetX - tipSize * Math.cos(yawRad + Math.PI / 6),
          targetY - tipSize * Math.sin(yawRad + Math.PI / 6)
        );
        this.ctx.closePath();
        this.ctx.fill();
      }

      // Draw Labels CENTERED BELOW the Node with Contrast Halo & Dynamic Scaling
      if (this.showLabels && (this.view.zoom > 0.6 || isWS || isChargerNode || isSelected || isHovered)) {
        const baseFontSize = Math.max(8, Math.round(9.5 * Math.sqrt(this.view.zoom)));
        const fontSize = isSelected ? Math.round(baseFontSize * 1.3) : (isHovered ? Math.round(baseFontSize * 1.15) : baseFontSize);
        const fontWeight = isSelected ? '800' : (isHovered ? '700' : '500');
        
        this.ctx.font = `${fontWeight} ${fontSize}px 'JetBrains Mono'`;
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'top';

        const labelY = screenPos.y + radius + 4;

        const textMetrics = this.ctx.measureText(id);
        const textWidth = textMetrics.width;
        const padX = 4;
        const padY = 2;
        const bgX = screenPos.x - textWidth / 2 - padX;
        const bgY = labelY - 1;
        const bgW = textWidth + padX * 2;
        const bgH = fontSize + padY * 2;

        // Draw crisp semi-transparent background pill to eliminate map text collision & character distortion
        this.ctx.fillStyle = isSelected 
          ? 'rgba(238, 242, 255, 0.96)' 
          : (isHovered ? 'rgba(241, 245, 249, 0.96)' : 'rgba(255, 255, 255, 0.90)');
        this.ctx.strokeStyle = isSelected 
          ? 'rgba(99, 102, 241, 0.6)' 
          : 'rgba(203, 213, 225, 0.8)';
        this.ctx.lineWidth = 1;

        this.ctx.beginPath();
        if (typeof this.ctx.roundRect === 'function') {
          this.ctx.roundRect(bgX, bgY, bgW, bgH, 3);
        } else {
          this.ctx.rect(bgX, bgY, bgW, bgH);
        }
        this.ctx.fill();
        this.ctx.stroke();

        // Fill text color
        if (isSelected) {
          this.ctx.fillStyle = '#4f46e5';
        } else if (isHovered) {
          this.ctx.fillStyle = '#0f172a';
        } else if (isChargerNode) {
          this.ctx.fillStyle = '#d97706';
        } else if (isWS) {
          this.ctx.fillStyle = '#059669';
        } else {
          this.ctx.fillStyle = '#334155';
        }

        this.ctx.fillText(id, screenPos.x, labelY + 1);
        
        this.ctx.textAlign = 'left';
        this.ctx.textBaseline = 'alphabetic';
      }
    });
  }

  applyTheme(theme) {
    this.currentTheme = 'light';
    document.documentElement.setAttribute('data-theme', 'light');
    localStorage.setItem('lsm_fms_theme', 'light');

    const icon = document.getElementById('icon-theme');
    const label = document.getElementById('label-theme');
    if (icon) icon.className = 'fa-solid fa-sun';
    if (label) label.textContent = 'Light Mode';
  }

  toggleSidebar() {
    this.sidebarCollapsed = !this.sidebarCollapsed;
    this.updateSidebarUI();
  }

  openSidebar() {
    if (this.sidebarCollapsed) {
      this.sidebarCollapsed = false;
      this.updateSidebarUI();
    }
  }

  updateSidebarUI() {
    const sidebar = document.getElementById('main-sidebar');
    const toggleBtn = document.getElementById('btn-toggle-sidebar');
    const icon = document.getElementById('icon-sidebar');
    const label = document.getElementById('label-sidebar');

    if (sidebar) {
      if (this.sidebarCollapsed) {
        sidebar.classList.add('collapsed');
        if (toggleBtn) toggleBtn.classList.remove('hidden');
        if (icon) icon.className = 'fa-solid fa-angles-left';
        if (label) label.textContent = 'Open Panel';
      } else {
        sidebar.classList.remove('collapsed');
        if (toggleBtn) toggleBtn.classList.add('hidden');
        if (icon) icon.className = 'fa-solid fa-angles-right';
        if (label) label.textContent = 'Close Panel';
      }
    }
  }

  setupEventListeners() {
    const btnTheme = document.getElementById('btn-theme-toggle');
    if (btnTheme) btnTheme.onclick = () => this.toggleTheme();

    const btnSidebar = document.getElementById('btn-toggle-sidebar');
    if (btnSidebar) btnSidebar.onclick = () => this.toggleSidebar();

    const btnCloseSidebar = document.getElementById('btn-close-sidebar');
    if (btnCloseSidebar) btnCloseSidebar.onclick = () => {
      this.sidebarCollapsed = true;
      this.updateSidebarUI();
    };

    const btnToggleOpt = document.getElementById('btn-toggle-options');
    const collOpt = document.getElementById('collapsible-options');
    const optChevron = document.getElementById('opt-chevron');
    if (btnToggleOpt && collOpt) {
      btnToggleOpt.onclick = () => {
        collOpt.classList.toggle('collapsed');
        if (optChevron) {
          optChevron.className = collOpt.classList.contains('collapsed') 
            ? 'fa-solid fa-chevron-down' 
            : 'fa-solid fa-chevron-up';
        }
      };
    }

    document.getElementById('chk-show-edges')?.addEventListener('change', (e) => { this.showEdges = e.target.checked; this.saveViewOptions(); this.render(); });
    document.getElementById('chk-show-arrows')?.addEventListener('change', (e) => { this.showArrows = e.target.checked; this.saveViewOptions(); this.render(); });
    document.getElementById('chk-show-labels')?.addEventListener('change', (e) => { this.showLabels = e.target.checked; this.saveViewOptions(); this.render(); });
    document.getElementById('chk-show-unconnected')?.addEventListener('change', (e) => { this.showUnconnected = e.target.checked; this.saveViewOptions(); this.render(); });

    const btnAutoFit = document.getElementById('btn-auto-fit');
    if (btnAutoFit) btnAutoFit.onclick = () => this.autoFitGraph(true);

    let mousedownPos = { x: 0, y: 0 };

    // Zoom & Pan Mouse Events
    this.container.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
      const newZoom = Math.max(0.2, Math.min(5.0, this.view.zoom * zoomFactor));

      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      this.view.panX = mouseX - (mouseX - this.view.panX) * (newZoom / this.view.zoom);
      this.view.panY = mouseY - (mouseY - this.view.panY) * (newZoom / this.view.zoom);
      this.view.zoom = newZoom;

      this.updateZoomBadge();
      this.render();
    });

    this.container.addEventListener('mousedown', (e) => {
      if (e.button === 0) { // Left click
        this.view.isDragging = true;
        this.view.dragStartX = e.clientX - this.view.panX;
        this.view.dragStartY = e.clientY - this.view.panY;
        mousedownPos = { x: e.clientX, y: e.clientY };
      }
    });

    window.addEventListener('mousemove', (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const coords = this.screenToWorld(mouseX, mouseY);
      const coordsEl = document.getElementById('cursor-coords');
      if (coordsEl) {
        coordsEl.textContent = `X: ${coords.wx.toFixed(2)}m | Y: ${coords.wy.toFixed(2)}m`;
      }

      if (this.view.isDragging) {
        this.view.panX = e.clientX - this.view.dragStartX;
        this.view.panY = e.clientY - this.view.dragStartY;
        this.render();
      } else {
        this.checkHoverNode(mouseX, mouseY);
      }
    });

    window.addEventListener('mouseup', () => {
      this.view.isDragging = false;
    });

    this.container.addEventListener('click', (e) => {
      const moveDist = Math.hypot(e.clientX - mousedownPos.x, e.clientY - mousedownPos.y);
      if (moveDist > 5) return; // Ignore drag/pan events

      const rect = this.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      this.selectNodeAtScreen(mouseX, mouseY);
    });

    // Toolbar Buttons
    document.getElementById('btn-zoom-in')?.addEventListener('click', () => {
      this.view.zoom = Math.min(5.0, this.view.zoom * 1.2);
      this.updateZoomBadge();
      this.render();
    });
    document.getElementById('btn-zoom-out')?.addEventListener('click', () => {
      this.view.zoom = Math.max(0.2, this.view.zoom / 1.2);
      this.updateZoomBadge();
      this.render();
    });
    document.getElementById('btn-reset-view')?.addEventListener('click', () => this.centerView());

    document.getElementById('chk-show-edges')?.addEventListener('change', (e) => { this.showEdges = e.target.checked; this.render(); });
    document.getElementById('chk-show-arrows')?.addEventListener('change', (e) => { this.showArrows = e.target.checked; this.render(); });
    document.getElementById('chk-show-labels')?.addEventListener('change', (e) => { this.showLabels = e.target.checked; this.render(); });
    document.getElementById('chk-show-unconnected')?.addEventListener('change', (e) => { this.showUnconnected = e.target.checked; this.populateWaypointsList(); this.render(); });

    // Sidebar Tab Switching
    document.querySelectorAll('.tab-btn').forEach(btn => {
      btn.onclick = () => {
        document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

        btn.classList.add('active');
        const tabId = btn.getAttribute('data-tab');
        const targetTab = document.getElementById(tabId);
        if (targetTab) targetTab.classList.add('active');
      };
    });

    // Calibration Controls Mapping
    const bindControl = (numId, rngId, propKey, isFloat = true) => {
      const num = document.getElementById(numId);
      const rng = document.getElementById(rngId);
      if (!num || !rng) return;

      const updateVal = (val) => {
        this.calibration[propKey] = isFloat ? parseFloat(val) : val;
        num.value = val;
        rng.value = val;
        this.render();
      };

      num.oninput = (e) => updateVal(e.target.value);
      rng.oninput = (e) => updateVal(e.target.value);
    };

    bindControl('num-offset-x', 'rng-offset-x', 'offsetX');
    bindControl('num-offset-y', 'rng-offset-y', 'offsetY');
    bindControl('num-scale', 'rng-scale', 'scale');
    bindControl('num-yaw', 'rng-yaw', 'yaw');

    const btnResetCal = document.getElementById('btn-reset-calibration');
    if (btnResetCal) btnResetCal.onclick = () => this.autoFitGraph(true);

    const btnSaveCal = document.getElementById('btn-save-calibration');
    if (btnSaveCal) btnSaveCal.onclick = () => this.saveCalibration();

    const btnExportCfg = document.getElementById('btn-export-config');
    if (btnExportCfg) {
      btnExportCfg.onclick = () => {
        const json = JSON.stringify(this.calibration, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = 'fms_map_calibration.json';
        a.click();
      };
    }

    // Inspector Filter & Search (Multi-select support!)
    document.querySelectorAll('.filter-pills .pill').forEach(pill => {
      pill.onclick = () => {
        const filter = pill.getAttribute('data-filter');

        if (filter === 'all') {
          // Select all 3 filters
          this.activeFilters = new Set(['workstation', 'charger', 'node']);
        } else {
          // Toggle clicked filter in activeFilters set
          if (this.activeFilters.has(filter)) {
            // Keep at least 1 filter active
            if (this.activeFilters.size > 1) {
              this.activeFilters.delete(filter);
            }
          } else {
            this.activeFilters.add(filter);
          }
        }

        // Update pill UI active states
        const allPill = document.querySelector('.filter-pills .pill[data-filter="all"]');
        const wsPill = document.querySelector('.filter-pills .pill[data-filter="workstation"]');
        const chargerPill = document.querySelector('.filter-pills .pill[data-filter="charger"]');
        const nodePill = document.querySelector('.filter-pills .pill[data-filter="node"]');

        const isAllActive = this.activeFilters.size === 3;
        if (allPill) allPill.classList.toggle('active', isAllActive);
        if (wsPill) wsPill.classList.toggle('active', this.activeFilters.has('workstation'));
        if (chargerPill) chargerPill.classList.toggle('active', this.activeFilters.has('charger'));
        if (nodePill) nodePill.classList.toggle('active', this.activeFilters.has('node'));

        this.populateWaypointsList();
        this.render();
      };
    });

    const inputSearch = document.getElementById('input-search-wp');
    if (inputSearch) {
      inputSearch.oninput = (e) => {
        const q = e.target.value.toLowerCase();
        this.populateWaypointsList(q);
      };
    }
  }

  checkHoverNode(sx, sy) {
    let foundId = null;

    for (const [id, wp] of Object.entries(this.waypoints)) {
      if (!this.matchesFilter(id, wp)) continue;
      if (!this.showUnconnected && this.isUnconnected(id, wp)) continue;

      const pos = this.getWaypointUserPose(wp);
      const imgPos = this.worldToImagePixel(pos.x, pos.y);
      const screenPos = this.imagePixelToScreen(imgPos.x, imgPos.y);

      const isWS = wp.type === 'workstation' || this.isCharger(id, wp);
      const hitRadius = Math.max(12, (isWS ? 14 : 10) * Math.sqrt(this.view.zoom));

      const dist = Math.hypot(sx - screenPos.x, sy - screenPos.y);
      if (dist <= hitRadius) {
        foundId = id;
        break;
      }
    }

    if (this.hoveredWpId !== foundId) {
      this.hoveredWpId = foundId;
      this.container.style.cursor = foundId ? 'pointer' : (this.view.isDragging ? 'grabbing' : 'grab');
      this.render();
    }
  }

  selectNodeAtScreen(sx, sy) {
    if (this.hoveredWpId) {
      this.selectNode(this.hoveredWpId);
      return;
    }

    let clickedId = null;

    for (const [id, wp] of Object.entries(this.waypoints)) {
      if (!this.matchesFilter(id, wp)) continue;
      if (!this.showUnconnected && this.isUnconnected(id, wp)) continue;

      const pos = this.getWaypointUserPose(wp);
      const imgPos = this.worldToImagePixel(pos.x, pos.y);
      const screenPos = this.imagePixelToScreen(imgPos.x, imgPos.y);

      const isWS = wp.type === 'workstation' || this.isCharger(id, wp);
      const hitRadius = Math.max(14, (isWS ? 16 : 12) * Math.sqrt(this.view.zoom));

      const dist = Math.hypot(sx - screenPos.x, sy - screenPos.y);
      if (dist <= hitRadius) {
        clickedId = id;
        break;
      }
    }

    this.selectNode(clickedId);
  }

  selectNode(id) {
    this.selectedWpId = id;
    this.render();
    
    if (id) {
      // Auto-open sidebar if closed and switch to Inspector tab for immediate verification
      this.openSidebar();
      this.switchToTab('tab-inspector');

      this.renderNodeDetail(id);
      document.querySelectorAll('.wp-item-row').forEach(row => {
        if (row.getAttribute('data-id') === id) {
          row.classList.add('selected');
          row.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
        } else {
          row.classList.remove('selected');
        }
      });
    } else {
      document.getElementById('node-detail-box').innerHTML = `
        <div class="empty-state">
          <i class="fa-solid fa-hand-pointer"></i>
          <p>Click any node on map to view detailed telemetry and docking pose</p>
        </div>
      `;
      document.querySelectorAll('.wp-item-row').forEach(row => row.classList.remove('selected'));
    }
  }

  renderNodeDetail(id) {
    const detailBox = document.getElementById('node-detail-box');
    const wp = this.waypoints[id];
    if (!wp) return;

    const pos = this.getWaypointUserPose(wp);
    const isChargerNode = this.isCharger(id, wp);

    let edgesHtml = '';
    if (wp.connected_to && wp.connected_to.length > 0) {
      edgesHtml = wp.connected_to.map(e =>
        `<span class="edge-chip">${e.target} (${e.distance_m || '?'}m)</span>`
      ).join('');
    } else {
      edgesHtml = '<span class="text-muted" style="font-size:10px;">No outgoing edges</span>';
    }

    const typeTagClass = isChargerNode ? 'node-type-tag charger' : 'node-type-tag';

    detailBox.innerHTML = `
      <div class="detail-card">
        <h4>
          <span>${id}</span>
          <span class="${typeTagClass}">${isChargerNode ? 'CHARGER' : wp.type.toUpperCase()}</span>
        </h4>
        <div class="detail-grid">
          <div class="detail-item">
            <div class="lbl">USER POSE X</div>
            <div class="val">${pos.x.toFixed(3)}m</div>
          </div>
          <div class="detail-item">
            <div class="lbl">USER POSE Y</div>
            <div class="val">${pos.y.toFixed(3)}m</div>
          </div>
          <div class="detail-item">
            <div class="lbl">DOCKING YAW</div>
            <div class="val">${typeof pos.yaw === 'number' ? pos.yaw.toFixed(1) + '°' : 'N/A'}</div>
          </div>
          <div class="detail-item">
            <div class="lbl">STATUS</div>
            <div class="val" style="color:var(--neon-green)">${wp.status || 'active'}</div>
          </div>
        </div>
        <div class="edges-title">CONNECTED TARGETS:</div>
        <div class="edges-list">${edgesHtml}</div>
      </div>
    `;
  }

  populateWaypointsList(filterQuery = '') {
    const listEl = document.getElementById('wp-list');
    listEl.innerHTML = '';

    Object.entries(this.waypoints).forEach(([id, wp]) => {
      if (!this.matchesFilter(id, wp)) return;
      if (!this.showUnconnected && this.isUnconnected(id, wp)) return;

      if (filterQuery && !id.toLowerCase().includes(filterQuery) && !wp.type.toLowerCase().includes(filterQuery)) return;

      const row = document.createElement('div');
      row.className = `wp-item-row ${id === this.selectedWpId ? 'selected' : ''}`;
      row.setAttribute('data-id', id);
      row.innerHTML = `
        <span class="wp-id">${id}</span>
        <span class="wp-type">${this.isCharger(id, wp) ? 'charger' : wp.type}</span>
      `;

      row.onclick = () => this.selectNode(id);
      listEl.appendChild(row);
    });
  }

  startRenderLoop() {
    let lastRenderTime = performance.now();
    const renderLoop = (now) => {
      lastRenderTime = now;
      this.render();
      requestAnimationFrame(renderLoop);
    };
    requestAnimationFrame(renderLoop);

    // Continuous background render fallback (20Hz) when browser tab/window is unfocused or in background
    setInterval(() => {
      const now = performance.now();
      if (now - lastRenderTime > 45) {
        this.render();
        lastRenderTime = now;
      }
    }, 50);
  }
}

// Instantiate App
window.addEventListener('DOMContentLoaded', () => {
  window.app = new FMSApp();
});
