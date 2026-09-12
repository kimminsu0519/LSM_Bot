/**
 * LSM FMS Developer Telemetry & Route Debugger
 * Author: Antigravity AI
 * Version: 1.0.0
 */

class DevFMSApp {
  constructor() {
    // Data States
    this.waypoints = {};
    this.adjGraph = {}; // wp_id -> list of {target, distance_m, direction}
    this.mapImage = new Image();
    this.mapLoaded = false;
    
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

    // Active Route & Path Planning State
    this.activeRoute = null; // Array of wp_id
    this.routeDetails = [];
    this.currentStepIdx = -1;
    this.activeGoalWp = null;

    // View Transforms (Pan & Zoom)
    this.view = {
      panX: 0,
      panY: 0,
      zoom: 1.0,
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0
    };

    // Calibration (Synced with default app.js)
    this.calibration = {
      offsetX: 22.8,
      offsetY: 12.65,
      scale: 1.0,
      yaw: 0.0,
      pxPerMeter: 100.0
    };

    // Toggles (Default showLabels = false)
    this.showEdges = true;
    this.showArrows = true;
    this.showLabels = false;
    this.showBranches = true;

    // WebSocket State
    this.ws = null;
    this.wsConnected = false;
    this.hasReceivedTelemetry = false;
    this.logPaused = false;
    this.wsUrl = `ws://${window.location.hostname || 'localhost'}:9090`;

    // Elements
    this.canvas = document.getElementById('map-canvas');
    this.ctx = this.canvas.getContext('2d');
    this.container = document.getElementById('canvas-container');

    // Debug Position Snapshots State
    this.snapshots = [];

    this.init();
  }

  async init() {
    this.loadSavedCalibration();
    this.loadSavedSnapshots();
    this.loadSavedViewOptions();
    this.setupEventListeners();
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());

    await Promise.all([this.loadMapImage(), this.loadYAMLData()]);

    this.fitGraphToViewport();
    this.populateWaypointSelects();
    this.initWebSocket();
    this.startRenderLoop();
  }

  loadSavedCalibration() {
    const saved = localStorage.getItem('lsm_fms_calibration');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        this.calibration = { ...this.calibration, ...parsed };
        this.updateCalibrationUI();
      } catch (e) {
        console.error('Error loading calibration:', e);
      }
    }
  }

  saveCalibration() {
    localStorage.setItem('lsm_fms_calibration', JSON.stringify(this.calibration));
    alert('Developer Calibration saved to LocalStorage!');
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

  setupEventListeners() {
    // Canvas Pan & Zoom
    this.container.addEventListener('mousedown', (e) => {
      this.view.isDragging = true;
      this.view.dragStartX = e.clientX - this.view.panX;
      this.view.dragStartY = e.clientY - this.view.panY;
    });

    window.addEventListener('mousemove', (e) => {
      if (this.view.isDragging) {
        this.view.panX = e.clientX - this.view.dragStartX;
        this.view.panY = e.clientY - this.view.dragStartY;
      }
      this.updateCursorCoords(e);
    });

    window.addEventListener('mouseup', () => {
      this.view.isDragging = false;
    });

    this.container.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
      const mouseX = e.clientX - this.container.getBoundingClientRect().left;
      const mouseY = e.clientY - this.container.getBoundingClientRect().top;

      const newZoom = Math.min(Math.max(this.view.zoom * zoomFactor, 0.1), 10.0);
      this.view.panX = mouseX - (mouseX - this.view.panX) * (newZoom / this.view.zoom);
      this.view.panY = mouseY - (mouseY - this.view.panY) * (newZoom / this.view.zoom);
      this.view.zoom = newZoom;

      const zoomBadge = document.getElementById('zoom-level');
      if (zoomBadge) zoomBadge.textContent = `${Math.round(this.view.zoom * 100)}%`;
    }, { passive: false });

    // Toolbar Controls
    document.getElementById('btn-zoom-in')?.addEventListener('click', () => this.adjustZoom(1.2));
    document.getElementById('btn-zoom-out')?.addEventListener('click', () => this.adjustZoom(0.8));
    document.getElementById('btn-reset-view')?.addEventListener('click', () => this.centerView());

    document.getElementById('chk-show-edges')?.addEventListener('change', (e) => { this.showEdges = e.target.checked; this.saveViewOptions(); });
    document.getElementById('chk-show-arrows')?.addEventListener('change', (e) => { this.showArrows = e.target.checked; this.saveViewOptions(); });
    document.getElementById('chk-show-labels')?.addEventListener('change', (e) => { this.showLabels = e.target.checked; this.saveViewOptions(); });
    document.getElementById('chk-show-branches')?.addEventListener('change', (e) => { this.showBranches = e.target.checked; this.saveViewOptions(); });
    document.getElementById('chk-use-gz-truth')?.addEventListener('change', () => this.saveViewOptions());

    // Calibration Controls Mapping
    const bindControl = (numId, rngId, propKey, isFloat = true) => {
      const num = document.getElementById(numId);
      const rng = document.getElementById(rngId);
      if (!num || !rng) return;

      const updateVal = (val) => {
        this.calibration[propKey] = isFloat ? parseFloat(val) : val;
        num.value = val;
        rng.value = val;
      };

      num.oninput = (e) => updateVal(e.target.value);
      rng.oninput = (e) => updateVal(e.target.value);
    };

    bindControl('num-offset-x', 'rng-offset-x', 'offsetX');
    bindControl('num-offset-y', 'rng-offset-y', 'offsetY');
    bindControl('num-scale', 'rng-scale', 'scale');
    bindControl('num-yaw', 'rng-yaw', 'yaw');

    document.getElementById('btn-reset-calibration')?.addEventListener('click', () => {
      this.calibration.offsetX = 22.8;
      this.calibration.offsetY = 12.65;
      this.calibration.scale = 1.0;
      this.calibration.yaw = 0.0;
      this.updateCalibrationUI();
    });

    document.getElementById('btn-auto-fit')?.addEventListener('click', () => {
      this.calibration.offsetX = 22.8;
      this.calibration.offsetY = 12.65;
      this.calibration.scale = 1.0;
      this.calibration.yaw = 0.0;
      this.updateCalibrationUI();
    });

    document.getElementById('btn-save-calibration')?.addEventListener('click', () => this.saveCalibration());

    document.getElementById('btn-export-config')?.addEventListener('click', () => {
      const json = JSON.stringify(this.calibration, null, 2);
      const blob = new Blob([json], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'fms_map_calibration.json';
      a.click();
    });

    // Sidebar Tabs
    const tabBtns = document.querySelectorAll('.sidebar-tabs .tab-btn');
    tabBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        tabBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const targetTab = btn.getAttribute('data-tab');
        document.querySelectorAll('.tab-content').forEach(tc => tc.classList.remove('active'));
        document.getElementById(targetTab)?.classList.add('active');
      });
    });

    // Sidebar Collapse & Close
    const updateSidebarUI = () => {
      const sidebar = document.getElementById('main-sidebar');
      const toggleBtn = document.getElementById('btn-toggle-sidebar');
      const icon = document.getElementById('icon-sidebar');
      const label = document.getElementById('label-sidebar');
      if (sidebar) {
        const isCollapsed = sidebar.classList.contains('collapsed');
        if (toggleBtn) {
          if (isCollapsed) toggleBtn.classList.remove('hidden');
          else toggleBtn.classList.add('hidden');
        }
        if (icon) icon.className = isCollapsed ? 'fa-solid fa-angles-left' : 'fa-solid fa-angles-right';
        if (label) label.textContent = isCollapsed ? 'Open Panel' : 'Close Panel';
      }
    };

    // Set initial sidebar toggle UI
    updateSidebarUI();

    document.getElementById('btn-toggle-sidebar')?.addEventListener('click', () => {
      const sidebar = document.getElementById('main-sidebar');
      if (sidebar) {
        sidebar.classList.toggle('collapsed');
        updateSidebarUI();
      }
    });

    document.getElementById('btn-close-sidebar')?.addEventListener('click', () => {
      const sidebar = document.getElementById('main-sidebar');
      if (sidebar) {
        sidebar.classList.add('collapsed');
        updateSidebarUI();
      }
    });

    // View Options Toggle
    const btnToggleOpt = document.getElementById('btn-toggle-options');
    const collOpt = document.getElementById('collapsible-options');
    const optChevron = document.getElementById('opt-chevron');
    if (btnToggleOpt && collOpt) {
      btnToggleOpt.addEventListener('click', () => {
        collOpt.classList.toggle('collapsed');
        if (optChevron) {
          optChevron.className = collOpt.classList.contains('collapsed') 
            ? 'fa-solid fa-chevron-down' 
            : 'fa-solid fa-chevron-up';
        }
      });
    }

    // Route Planner & WS Call Scenario Actions
    document.getElementById('btn-set-start-bot')?.addEventListener('click', () => this.setStartToBotPose());
    document.getElementById('btn-plan-route')?.addEventListener('click', () => this.calculateAndPreviewRoute());
    document.getElementById('btn-dispatch-route')?.addEventListener('click', () => this.dispatchRouteGoal());

    document.getElementById('btn-call-ws-a1')?.addEventListener('click', () => this.dispatchWorkstationCall('wp-ws-a-1'));
    document.getElementById('btn-call-ws-a2')?.addEventListener('click', () => this.dispatchWorkstationCall('wp-ws-a-2'));
    document.getElementById('btn-call-charger')?.addEventListener('click', () => this.dispatchWorkstationCall('wp-charge-1'));

    // WS Log Controls
    document.getElementById('btn-pause-log')?.addEventListener('click', (e) => {
      this.logPaused = !this.logPaused;
      e.target.innerHTML = this.logPaused ? '<i class="fa-solid fa-play"></i> Resume' : '<i class="fa-solid fa-pause"></i> Pause';
    });

    document.getElementById('btn-clear-log')?.addEventListener('click', () => {
      const consoleEl = document.getElementById('ws-log-console');
      if (consoleEl) consoleEl.innerHTML = '<div class="log-entry system">[SYSTEM] Log cleared.</div>';
    });

    // Position Recorder & Offset Debugger Actions
    const recordHandler = () => this.recordPoseSnapshot();
    document.getElementById('btn-quick-record-snap')?.addEventListener('click', () => {
      const tabBtn = document.querySelector('.sidebar-tabs .tab-btn[data-tab="tab-recorder"]');
      if (tabBtn) tabBtn.click();
      const sidebar = document.getElementById('main-sidebar');
      if (sidebar && sidebar.classList.contains('collapsed')) {
        document.getElementById('btn-toggle-sidebar')?.click();
      }
      recordHandler();
    });
    document.getElementById('btn-trigger-record-snap')?.addEventListener('click', recordHandler);
    document.getElementById('btn-apply-delta-to-calib')?.addEventListener('click', () => this.applyDeltaToCalibration());
    document.getElementById('btn-reset-to-charger')?.addEventListener('click', () => {
      if (this.wsConnected && this.ws) {
        this.sendWSMessage({ type: 'reset_initial_pose', wp_id: 'wp-charge-1' });
        alert('⚡ 로봇 초기 위치(AMCL initialpose)를 wp-charge-1로 설정했습니다!');
      } else {
        alert('WebSocket 연결을 확인해주세요.');
      }
    });
    document.getElementById('btn-clear-snapshots')?.addEventListener('click', () => {
      if (confirm('모든 디버깅 캡처 기록을 삭제하시겠습니까?')) {
        this.snapshots = [];
        localStorage.removeItem('lsm_pose_snapshots');
        this.renderSnapshotsList();
      }
    });
    document.getElementById('btn-export-snapshots')?.addEventListener('click', () => this.exportSnapshotsJSON());
  }

  saveViewOptions() {
    const options = {
      showEdges: this.showEdges,
      showArrows: this.showArrows,
      showLabels: this.showLabels,
      showBranches: this.showBranches,
      useGzTruth: document.getElementById('chk-use-gz-truth')?.checked || false
    };
    localStorage.setItem('lsm_fms_dev_view_options', JSON.stringify(options));
  }

  loadSavedViewOptions() {
    try {
      const saved = localStorage.getItem('lsm_fms_dev_view_options');
      if (saved) {
        const opts = JSON.parse(saved);
        if (typeof opts.showEdges === 'boolean') this.showEdges = opts.showEdges;
        if (typeof opts.showArrows === 'boolean') this.showArrows = opts.showArrows;
        if (typeof opts.showLabels === 'boolean') this.showLabels = opts.showLabels;
        if (typeof opts.showBranches === 'boolean') this.showBranches = opts.showBranches;
        const gzChk = document.getElementById('chk-use-gz-truth');
        if (gzChk && typeof opts.useGzTruth === 'boolean') gzChk.checked = opts.useGzTruth;
      }
    } catch (e) {
      console.warn('Failed to load saved dev view options:', e);
    }
    this.updateToggleUI();
  }

  updateToggleUI() {
    const chkEdges = document.getElementById('chk-show-edges');
    const chkArrows = document.getElementById('chk-show-arrows');
    const chkLabels = document.getElementById('chk-show-labels');
    const chkBranches = document.getElementById('chk-show-branches');

    if (chkEdges) chkEdges.checked = this.showEdges;
    if (chkArrows) chkArrows.checked = this.showArrows;
    if (chkLabels) chkLabels.checked = this.showLabels;
    if (chkBranches) chkBranches.checked = this.showBranches;
  }

  adjustZoom(factor) {
    const center = { x: this.canvas.width / 2, y: this.canvas.height / 2 };
    const newZoom = Math.min(Math.max(this.view.zoom * factor, 0.1), 10.0);
    this.view.panX = center.x - (center.x - this.view.panX) * (newZoom / this.view.zoom);
    this.view.panY = center.y - (center.y - this.view.panY) * (newZoom / this.view.zoom);
    this.view.zoom = newZoom;
    const badge = document.getElementById('zoom-level');
    if (badge) badge.textContent = `${Math.round(this.view.zoom * 100)}%`;
  }

  resizeCanvas() {
    this.canvas.width = this.container.clientWidth;
    this.canvas.height = this.container.clientHeight;
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
        done();
      };
      this.mapImage.onerror = () => {
        console.warn('Map image failed to load.');
        this.mapLoaded = false;
        done();
      };
      setTimeout(() => {
        if (!isDone) {
          console.warn('Developer map image load timeout fallback');
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
        console.warn('Developer JSON fetch failed, attempting YAML fallback...', e);
      }

      if (!data) {
        const res = await fetch('config/waypoints_graph2.yaml');
        const text = await res.text();
        if (window.jsyaml) {
          data = jsyaml.load(text);
        }
      }

      this.waypoints = (data && data.waypoints) ? data.waypoints : {};
      this.buildAdjacencyGraph();
      this.fitGraphToViewport();
    } catch (e) {
      console.error('Failed to load YAML/JSON graph data:', e);
    }
  }

  buildAdjacencyGraph() {
    this.adjGraph = {};
    for (const [wpId, wp] of Object.entries(this.waypoints)) {
      this.adjGraph[wpId] = [];
      const posePos = wp.pose ? wp.pose.position : null;
      if (posePos && typeof posePos.x === 'number') {
        wp.x = posePos.x;
        wp.y = posePos.y;
      } else {
        const pos = wp.user_pose || {};
        wp.x = pos.x || 0.0;
        wp.y = pos.y || 0.0;
      }
      wp.yaw_deg = (wp.user_pose ? wp.user_pose.yaw_deg : 0.0);
    }

    for (const [srcId, wp] of Object.entries(this.waypoints)) {
      for (const edge of wp.connected_to || []) {
        const tgtId = edge.target;
        const dist = edge.distance_m || 1.0;
        const dir = edge.direction || 'one_way';
        if (this.waypoints[tgtId]) {
          this.adjGraph[srcId].push({ target: tgtId, distance_m: dist, direction: dir });
        }
      }
    }
  }

  populateWaypointSelects() {
    const startSelect = document.getElementById('select-start-wp');
    const goalSelect = document.getElementById('select-goal-wp');
    if (!startSelect || !goalSelect) return;

    startSelect.innerHTML = '';
    goalSelect.innerHTML = '';

    const sortedIds = Object.keys(this.waypoints).sort((a, b) => {
      const typeA = this.waypoints[a].type;
      const typeB = this.waypoints[b].type;
      if (typeA !== typeB) return typeA.localeCompare(typeB);
      return a.localeCompare(b);
    });

    sortedIds.forEach(id => {
      const wp = this.waypoints[id];
      const opt1 = document.createElement('option');
      opt1.value = id;
      opt1.textContent = `${id} (${wp.type})`;

      const opt2 = document.createElement('option');
      opt2.value = id;
      opt2.textContent = `${id} (${wp.type})`;

      startSelect.appendChild(opt1);
      goalSelect.appendChild(opt2);
    });

    // Default selections
    if (this.waypoints['wp-charge-1']) startSelect.value = 'wp-charge-1';
    if (this.waypoints['wp-ws-a-1']) goalSelect.value = 'wp-ws-a-1';
  }

  setStartToBotPose() {
    if (!this.waypoints) return;
    let closestId = null;
    let minDist = Infinity;

    for (const [id, wp] of Object.entries(this.waypoints)) {
      const dist = Math.hypot(wp.x - this.robotState.x, wp.y - this.robotState.y);
      if (dist < minDist) {
        minDist = dist;
        closestId = id;
      }
    }

    if (closestId) {
      const startSelect = document.getElementById('select-start-wp');
      if (startSelect) startSelect.value = closestId;
      this.logToConsole(`[ROBOT POSE] Set start waypoint to closest node: ${closestId} (${minDist.toFixed(2)}m away)`);
    }
  }

  /* --- In-Browser A* Path Planner --- */
  calculateAndPreviewRoute() {
    const startWp = document.getElementById('select-start-wp').value;
    const goalWp = document.getElementById('select-goal-wp').value;

    if (!startWp || !goalWp) return;

    const result = this.findAStarPath(startWp, goalWp);
    if (!result.path) {
      alert(`No path found from ${startWp} to ${goalWp}`);
      return;
    }

    this.activeRoute = result.path;
    this.routeDetails = this.extractRouteDetails(result.path);
    this.activeGoalWp = goalWp;
    this.currentStepIdx = 0;

    // Update UI
    document.getElementById('route-total-dist').textContent = `${result.totalDist.toFixed(2)} m`;
    document.getElementById('route-wp-count').textContent = `${result.path.length}`;
    document.getElementById('route-est-time').textContent = `${Math.ceil(result.totalDist / 0.8)}s`;
    document.getElementById('hud-goal').textContent = goalWp;

    this.renderRouteQueueList();
    this.renderBranchDecisionBox(0);
    this.logToConsole(`[A* PATH CALC] Computed path ${startWp} -> ${goalWp} (${result.path.length} waypoints, ${result.totalDist.toFixed(2)}m)`);
  }

  findAStarPath(startWp, goalWp) {
    if (startWp === goalWp) return { path: [startWp], totalDist: 0.0 };

    const heuristic = (a, b) => Math.hypot(this.waypoints[b].x - this.waypoints[a].x, this.waypoints[b].y - this.waypoints[a].y);

    const openSet = new Set([startWp]);
    const cameFrom = {};
    const gScore = {};
    const fScore = {};

    for (const wpId of Object.keys(this.waypoints)) {
      gScore[wpId] = Infinity;
      fScore[wpId] = Infinity;
    }

    gScore[startWp] = 0;
    fScore[startWp] = heuristic(startWp, goalWp);

    while (openSet.size > 0) {
      // Get node with lowest fScore
      let current = null;
      let lowestF = Infinity;
      for (const node of openSet) {
        if (fScore[node] < lowestF) {
          lowestF = fScore[node];
          current = node;
        }
      }

      if (current === goalWp) {
        // Reconstruct path
        const path = [current];
        while (current in cameFrom) {
          current = cameFrom[current];
          path.unshift(current);
        }
        return { path, totalDist: gScore[goalWp] };
      }

      openSet.delete(current);

      for (const edge of this.adjGraph[current] || []) {
        const neighbor = edge.target;
        const tentativeG = gScore[current] + edge.distance_m;

        if (tentativeG < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeG;
          fScore[neighbor] = tentativeG + heuristic(neighbor, goalWp);
          openSet.add(neighbor);
        }
      }
    }

    return { path: null, totalDist: 0 };
  }

  extractRouteDetails(path) {
    const details = [];
    for (let i = 0; i < path.length; i++) {
      const wpId = path[i];
      const wp = this.waypoints[wpId];
      const nextWp = path[i + 1] || null;
      const branches = (this.adjGraph[wpId] || []).map(e => e.target);

      details.push({
        step: i,
        wpId,
        type: wp.type,
        x: wp.x,
        y: wp.y,
        yaw_deg: wp.yaw_deg,
        nextWp,
        branches
      });
    }
    return details;
  }

  dispatchWorkstationCall(wsId) {
    this.setStartToBotPose();
    const startWp = document.getElementById('select-start-wp')?.value || 'wp-charge-1';
    const goalSelect = document.getElementById('select-goal-wp');
    if (goalSelect && this.waypoints[wsId]) goalSelect.value = wsId;

    this.calculateAndPreviewRoute();

    if (this.wsConnected && this.ws) {
      const payload = {
        type: 'ws_call_request',
        robot_id: this.robotState.id,
        ws_id: wsId,
        start_wp: startWp
      };
      this.sendWSMessage(payload);
    }
    this.dispatchRouteGoal();
  }

  renderRouteQueueList() {
    const container = document.getElementById('route-queue-list');
    if (!container || !this.activeRoute) return;

    container.innerHTML = '';
    this.activeRoute.forEach((wpId, idx) => {
      const wp = this.waypoints[wpId];
      const item = document.createElement('div');
      item.className = `queue-item ${idx === this.currentStepIdx ? 'current' : (idx < this.currentStepIdx ? 'passed' : '')}`;

      let statusTag = 'PENDING';
      let tagClass = 'pending';
      if (idx < this.currentStepIdx) { statusTag = 'PASSED'; tagClass = 'passed'; }
      else if (idx === this.currentStepIdx) { statusTag = 'CURRENT'; tagClass = 'current'; }
      else if (idx === this.currentStepIdx + 1) { statusTag = 'NEXT'; tagClass = 'next'; }

      item.innerHTML = `
        <span class="wp-seq">#${idx + 1}</span>
        <span class="wp-name"><strong>${wpId}</strong> <small>(${wp.type})</small></span>
        <span class="queue-status-tag ${tagClass}">${statusTag}</span>
      `;
      container.appendChild(item);
    });
  }

  renderBranchDecisionBox(stepIdx) {
    const box = document.getElementById('branch-decision-box');
    if (!box || !this.activeRoute || stepIdx >= this.activeRoute.length) return;

    const currentWpId = this.activeRoute[stepIdx];
    const nextWpId = this.activeRoute[stepIdx + 1] || null;
    const branches = this.adjGraph[currentWpId] || [];

    if (branches.length <= 1) {
      box.innerHTML = `<div class="empty-state-sm"><p>Node <strong>${currentWpId}</strong> has single path (${nextWpId || 'End'}).</p></div>`;
      document.getElementById('hud-branch').textContent = nextWpId || 'DESTINATION';
      return;
    }

    document.getElementById('hud-branch').textContent = `${branches.length} CHOICES @ ${currentWpId}`;

    let html = `<div style="font-size:11px; margin-bottom:4px; color:var(--text-muted);">Decision Intersection @ <strong>${currentWpId}</strong> (${branches.length} choices):</div>`;
    branches.forEach(b => {
      const isChosen = b.target === nextWpId;
      html += `
        <div class="branch-choice-row ${isChosen ? 'chosen' : ''}">
          <span>--> <strong>${b.target}</strong> (${b.distance_m.toFixed(2)}m)</span>
          <span class="badge-choice">${isChosen ? 'A* OPTIMAL CHOICE' : 'ALTERNATIVE'}</span>
        </div>
      `;
    });
    box.innerHTML = html;
  }

  dispatchRouteGoal() {
    if (!this.activeRoute || this.activeRoute.length === 0) {
      alert('Please calculate a path before dispatching.');
      return;
    }

    const payload = {
      type: 'dispatch_route',
      robot_id: this.robotState.id,
      start_wp: this.activeRoute[0],
      goal_wp: this.activeGoalWp,
      route: this.activeRoute
    };

    this.sendWSMessage(payload);
    this.robotState.status = 'NAVIGATING';
    document.getElementById('dev-robot-status').className = 'robot-status-pill navigating';
    document.getElementById('dev-robot-status').textContent = 'NAVIGATING';
    document.getElementById('nav2-action-state').textContent = 'EXECUTING';
    document.getElementById('nav2-goal-id').textContent = `GOAL-${Date.now().toString().slice(-6)}`;

    this.logToConsole(`[DISPATCH] Sent route goal to ${this.robotState.id} (Sequence: ${this.activeRoute.join(' -> ')})`);
  }

  /* --- WebSocket Interface --- */
  initWebSocket() {
    this.updateWSBadge(false);
    try {
      this.ws = new WebSocket(this.wsUrl);
      
      this.ws.onopen = () => {
        this.wsConnected = true;
        this.updateWSBadge(true);
        this.logToConsole(`[WS CONNECTED] Server online at ${this.wsUrl}`);
      };

      this.ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data);
          this.handleWSMessage(msg);
        } catch (e) {
          this.logToConsole(`[WS RAW] ${evt.data}`, 'in');
        }
      };

      this.ws.onerror = (err) => {
        this.wsConnected = false;
        this.updateWSBadge(false);
      };

      this.ws.onclose = () => {
        this.wsConnected = false;
        this.updateWSBadge(false);
        // Retry connection in 3 seconds
        setTimeout(() => this.initWebSocket(), 3000);
      };
    } catch (e) {
      this.updateWSBadge(false);
    }
  }

  updateWSBadge(connected) {
    const badge = document.getElementById('ws-status-badge');
    const text = document.getElementById('ws-status-text');
    if (!badge || !text) return;

    if (connected) {
      badge.className = 'status-badge live';
      text.textContent = 'WS ONLINE';
    } else {
      badge.className = 'status-badge disconnected';
      text.textContent = 'WS OFFLINE';
    }
  }

  sendWSMessage(data) {
    const jsonStr = JSON.stringify(data);
    if (this.ws && this.wsConnected) {
      this.ws.send(jsonStr);
    }
    this.logToConsole(`[WS OUT] ${jsonStr}`, 'out');
  }

  handleWSMessage(msg) {
    if (!this.logPaused) {
      if (msg.type === 'telemetry') {
        if (this.lastLoggedStatus !== msg.status) {
          this.lastLoggedStatus = msg.status;
          this.logToConsole(`[WS IN] Telemetry Status: ${msg.status}`, 'in');
        }
      } else {
        this.logToConsole(`[WS IN] ${JSON.stringify(msg)}`, 'in');
      }
    }

    if (msg.type === 'telemetry') {
      if (msg.has_pose !== false) {
        this.hasReceivedTelemetry = true;
      }
      this.robotState = { ...this.robotState, ...msg };
      this.updateTelemetryUI();
    } else if (msg.type === 'route_progress') {
      this.currentStepIdx = msg.current_step || 0;
      this.renderRouteQueueList();
      this.renderBranchDecisionBox(this.currentStepIdx);
    }
  }

  updateTelemetryUI() {
    const rx = this.robotState.ros_x ?? (this.robotState.x - 2.457);
    const ry = this.robotState.ros_y ?? (this.robotState.y - 0.358);
    const cx = this.robotState.x;
    const cy = this.robotState.y;
    const gzx = this.robotState.gz_x || 0;
    const gzy = this.robotState.gz_y || 0;
    const gzyaw = this.robotState.gz_yaw || 0;
    const dx = gzx - rx;
    const dy = gzy - ry;

    document.getElementById('tel-x').textContent = `${rx.toFixed(2)} m`;
    document.getElementById('tel-y').textContent = `${ry.toFixed(2)} m`;
    document.getElementById('tel-yaw').textContent = `${this.robotState.yaw_deg.toFixed(1)} °`;
    document.getElementById('tel-v').textContent = `${(this.robotState.v || 0.0).toFixed(2)} m/s`;
    document.getElementById('tel-w').textContent = `${(this.robotState.w || 0.0).toFixed(2)} rad/s`;
    document.getElementById('tel-battery').textContent = `${this.robotState.battery || 100}%`;

    document.getElementById('cov-x').textContent = `${(this.robotState.covX || 0.002).toFixed(3)}m`;
    document.getElementById('cov-y').textContent = `${(this.robotState.covY || 0.002).toFixed(3)}m`;
    document.getElementById('cov-yaw').textContent = `${(this.robotState.covYaw || 0.01).toFixed(2)}rad`;

    // Update Active Recorder Card
    const recRos = document.getElementById('rec-ros-pose');
    const recGz = document.getElementById('rec-gz-pose');
    const recDelta = document.getElementById('rec-delta-offset');
    if (recRos) recRos.textContent = `X: ${rx.toFixed(3)}m | Y: ${ry.toFixed(3)}m | Yaw: ${this.robotState.yaw_deg.toFixed(1)}°`;
    if (recGz) recGz.textContent = `X: ${gzx.toFixed(3)}m | Y: ${gzy.toFixed(3)}m | Yaw: ${gzyaw.toFixed(1)}°`;
    if (recDelta) recDelta.textContent = `ΔX: ${dx.toFixed(3)}m | ΔY: ${dy.toFixed(3)}m`;

    document.getElementById('hud-pose').textContent = `Map: X ${cx.toFixed(2)}m, Y ${cy.toFixed(2)}m (ROS2: X ${rx.toFixed(2)}m, Y ${ry.toFixed(2)}m) | Yaw: ${this.robotState.yaw_deg.toFixed(1)}°`;
  }

  /* --- Position Recorder & Debugger Methods --- */
  loadSavedSnapshots() {
    const saved = localStorage.getItem('lsm_pose_snapshots');
    if (saved) {
      try {
        this.snapshots = JSON.parse(saved);
        this.renderSnapshotsList();
      } catch (e) {
        console.error('Error loading snapshots:', e);
      }
    }
  }

  recordPoseSnapshot() {
    const ros_x = this.robotState.ros_x ?? (this.robotState.x - 2.457);
    const ros_y = this.robotState.ros_y ?? (this.robotState.y - 0.358);
    const canvas_x = this.robotState.x;
    const canvas_y = this.robotState.y;
    const yaw_deg = this.robotState.yaw_deg || 0;
    const gz_x = this.robotState.gz_x || 0;
    const gz_y = this.robotState.gz_y || 0;
    const gz_yaw = this.robotState.gz_yaw || 0;
    const delta_x = gz_x - ros_x;
    const delta_y = gz_y - ros_y;

    // Capture Map Canvas screenshot
    let image_b64 = '';
    try {
      image_b64 = this.canvas.toDataURL('image/png');
    } catch (e) {
      console.warn('Canvas export warning:', e);
    }

    const now = new Date();
    const tsStr = now.toLocaleTimeString() + `.${now.getMilliseconds()}`;

    const snap = {
      id: 'snap_' + Date.now(),
      timestamp: now.toISOString(),
      tsStr: tsStr,
      ros_map: { x: ros_x, y: ros_y, yaw_deg },
      fms_canvas: { x: canvas_x, y: canvas_y },
      gazebo_odom: { x: gz_x, y: gz_y, yaw_deg: gz_yaw },
      offset_delta: { delta_x, delta_y },
      image_b64: image_b64
    };

    this.snapshots.unshift(snap);
    if (this.snapshots.length > 50) this.snapshots.pop();

    try {
      localStorage.setItem('lsm_pose_snapshots', JSON.stringify(this.snapshots));
    } catch (e) {
      console.warn('LocalStorage snapshot save warning:', e);
    }

    if (this.wsConnected && this.ws) {
      this.sendWSMessage({
        type: 'save_pose_snapshot',
        ros_x, ros_y, yaw_deg,
        canvas_x, canvas_y,
        gz_x, gz_y, gz_yaw,
        image_b64,
        note: `Manual debug capture @ ${tsStr}`
      });
    }

    this.renderSnapshotsList();
    this.logToConsole(`[RECORDER] Saved Pose Snapshot: ROS(${ros_x.toFixed(2)}, ${ros_y.toFixed(2)}) | GZ(${gz_x.toFixed(2)}, ${gz_y.toFixed(2)}) | Δ(${delta_x.toFixed(2)}, ${delta_y.toFixed(2)})`);
  }

  renderSnapshotsList() {
    const container = document.getElementById('snapshots-list');
    const countEl = document.getElementById('snap-count');
    if (!container) return;

    if (countEl) countEl.textContent = this.snapshots.length;

    if (this.snapshots.length === 0) {
      container.innerHTML = `<div class="empty-state-sm"><p>기록된 캡처가 없습니다. 위 [위치 기록 & 캔버스 캡처 저장] 버튼을 클릭하세요.</p></div>`;
      return;
    }

    let html = '';
    this.snapshots.forEach((snap, idx) => {
      const rx = snap.ros_map.x.toFixed(3);
      const ry = snap.ros_map.y.toFixed(3);
      const gzx = snap.gazebo_odom.x.toFixed(3);
      const gzy = snap.gazebo_odom.y.toFixed(3);
      const dx = snap.offset_delta.delta_x.toFixed(3);
      const dy = snap.offset_delta.delta_y.toFixed(3);
      const timeTag = new Date(snap.timestamp).toLocaleTimeString();

      html += `
        <div class="snapshot-item-card">
          <div class="snapshot-header">
            <span>📸 #${this.snapshots.length - idx} [${timeTag}]</span>
            <span class="neon-green">ΔX: ${dx}m | ΔY: ${dy}m</span>
          </div>
          <div class="snapshot-body">
            ${snap.image_b64 ? `<img src="${snap.image_b64}" class="snapshot-thumb" onclick="const w=window.open('');w.document.write('<img src=\\'${snap.image_b64}\\' style=\\'max-width:100%\\'>')" title="Click to view full screenshot">` : ''}
            <div class="snapshot-details">
              <div><strong>ROS Map:</strong> (${rx}, ${ry}) | Yaw: ${snap.ros_map.yaw_deg.toFixed(1)}°</div>
              <div><strong>Gazebo:</strong> (${gzx}, ${gzy}) | Yaw: ${snap.gazebo_odom.yaw_deg.toFixed(1)}°</div>
              <div><strong>Canvas:</strong> (${snap.fms_canvas.x.toFixed(3)}, ${snap.fms_canvas.y.toFixed(3)})</div>
            </div>
          </div>
        </div>
      `;
    });
    container.innerHTML = html;
  }

  applyDeltaToCalibration() {
    const rx = this.robotState.ros_x ?? (this.robotState.x - 2.457);
    const ry = this.robotState.ros_y ?? (this.robotState.y - 0.358);
    const gz_x = this.robotState.gz_x || 0;
    const gz_y = this.robotState.gz_y || 0;
    const delta_x = gz_x - rx;
    const delta_y = gz_y - ry;

    this.calibration.offsetX = parseFloat(delta_x.toFixed(3));
    this.calibration.offsetY = parseFloat(delta_y.toFixed(3));
    this.updateCalibrationUI();
    this.saveCalibration();

    alert(`⚡ GZ ↔ ROS Offset Delta가 Calibration에 즉시 반영되었습니다!\nOffset X: ${this.calibration.offsetX}m, Offset Y: ${this.calibration.offsetY}m`);
  }

  exportSnapshotsJSON() {
    if (this.snapshots.length === 0) {
      alert('내보낼 캡처 데이터가 없습니다.');
      return;
    }
    const cleanSnaps = this.snapshots.map(s => {
      const { image_b64, ...rest } = s;
      return rest;
    });
    const json = JSON.stringify(cleanSnaps, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `lsm_position_debug_report_${Date.now()}.json`;
    a.click();
  }

  logToConsole(text, type = 'system') {
    const consoleEl = document.getElementById('ws-log-console');
    if (!consoleEl || this.logPaused) return;

    const timeStr = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.textContent = `[${timeStr}] ${text}`;

    consoleEl.appendChild(entry);
    consoleEl.scrollTop = consoleEl.scrollHeight;
  }

  /* --- Main Canvas Render Loop --- */
  startRenderLoop() {
    let lastRenderTime = performance.now();
    const render = (now) => {
      lastRenderTime = now;
      this.drawCanvas();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);

    // Continuous background render fallback (20Hz) when browser tab/window is unfocused or in background
    setInterval(() => {
      const now = performance.now();
      if (now - lastRenderTime > 45) {
        this.drawCanvas();
        lastRenderTime = now;
      }
    }, 50);
  }

  saveViewState() {
    try {
      localStorage.setItem('lsm_fms_dev_view_state', JSON.stringify({
        zoom: this.view.zoom,
        panX: this.view.panX,
        panY: this.view.panY
      }));
    } catch (e) {}
  }

  loadSavedViewState() {
    try {
      const saved = localStorage.getItem('lsm_fms_dev_view_state');
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

  updateZoomBadge() {
    const badge = document.getElementById('zoom-level');
    if (badge) badge.textContent = `${Math.round(this.view.zoom * 100)}%`;
  }

  centerView() {
    this.fitGraphToViewport(true);
  }

  fitGraphToViewport(forceReset = false) {
    if (!forceReset && this.loadSavedViewState()) {
      return;
    }

    // Default Zoomed-In View (~87% zoom, centered around active warehouse aisles & charging pad)
    const targetZoom = 0.87;
    this.view.zoom = targetZoom;

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;

    if (this.waypoints && Object.keys(this.waypoints).length > 0) {
      Object.values(this.waypoints).forEach(wp => {
        const pxX = 160.0 + wp.x * 100.0;
        const pxY = 880.0 - wp.y * 100.0;
        if (pxX < minX) minX = pxX;
        if (pxX > maxX) maxX = pxX;
        if (pxY < minY) minY = pxY;
        if (pxY > maxY) maxY = pxY;
      });
    }

    const centerX = (minX !== Infinity) ? (minX + maxX) / 2 : 550.0;
    const centerY = (minY !== Infinity) ? (minY + maxY) / 2 : 550.0;

    this.view.panX = (this.canvas.width / 2) - (centerX * targetZoom);
    this.view.panY = (this.canvas.height / 2) - (centerY * targetZoom);

    this.updateZoomBadge();
    this.saveViewState();
  }

  updateCursorCoords(e) {
    const rect = this.container.getBoundingClientRect();
    const canvasX = e.clientX - rect.left;
    const canvasY = e.clientY - rect.top;

    const world = this.canvasToWorld(canvasX, canvasY);
    const coordsEl = document.getElementById('cursor-coords');
    if (coordsEl) {
      coordsEl.textContent = `X: ${world.x.toFixed(2)}m | Y: ${world.y.toFixed(2)}m`;
    }
  }

  worldToCanvas(worldX, worldY) {
    let calX = worldX + parseFloat(this.calibration.offsetX);
    let calY = worldY + parseFloat(this.calibration.offsetY);

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

    return {
      x: px * this.view.zoom + this.view.panX,
      y: py * this.view.zoom + this.view.panY
    };
  }

  canvasToWorld(canvasX, canvasY) {
    const px = (canvasX - this.view.panX) / this.view.zoom;
    const py = (canvasY - this.view.panY) / this.view.zoom;

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

    return { x: wx, y: wy };
  }

  drawCanvas() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // 1. Draw Map Image (Fixed position)
    if (this.mapLoaded) {
      this.ctx.drawImage(
        this.mapImage,
        this.view.panX,
        this.view.panY,
        this.mapImage.width * this.view.zoom,
        this.mapImage.height * this.view.zoom
      );
    }

    // 2. Draw Graph Edges
    if (this.showEdges) {
      this.drawEdges();
    }

    // 3. Draw Active Planned Route Line
    if (this.activeRoute && this.activeRoute.length > 1) {
      this.drawActiveRoute();
    }

    // 4. Draw Decision Branch Options
    if (this.showBranches && this.activeRoute && this.currentStepIdx >= 0 && this.currentStepIdx < this.activeRoute.length) {
      this.drawBranchChoiceOverlays();
    }

    // 5. Draw Waypoint Nodes
    this.drawNodes();

    // 6. Draw AMR Robot Position Icon & Covariance
    this.drawRobotState();
  }

  drawEdges() {
    this.ctx.lineWidth = Math.max(1, 1.2 * this.view.zoom);
    this.ctx.strokeStyle = 'rgba(2, 132, 199, 0.35)';
    this.ctx.fillStyle = 'rgba(2, 132, 199, 0.55)';

    const renderedEdgeKeys = new Set();

    Object.entries(this.waypoints).forEach(([id, wp]) => {
      if (!wp.connected_to || wp.connected_to.length === 0) return;

      const srcPt = this.worldToCanvas(wp.x, wp.y);
      const isSrcWS = wp.type === 'workstation' || id.toLowerCase().includes('charger') || id.toLowerCase().includes('wp-ch');

      wp.connected_to.forEach(edge => {
        const targetId = edge.target;
        const targetWp = this.waypoints[targetId];
        if (targetWp) {
          const edgeKey = id < targetId ? `${id}__${targetId}` : `${targetId}__${id}`;
          const dirStr = (edge.direction || '').toLowerCase();
          const isTwoWay = dirStr === 'bidirectional' || dirStr === 'two_way' || dirStr === 'both' || dirStr === 'bi';

          const tgtPt = this.worldToCanvas(targetWp.x, targetWp.y);
          const isTgtWS = targetWp.type === 'workstation' || targetId.toLowerCase().includes('charger') || targetId.toLowerCase().includes('wp-ch');

          // Draw Line
          this.ctx.beginPath();
          this.ctx.moveTo(srcPt.x, srcPt.y);
          this.ctx.lineTo(tgtPt.x, tgtPt.y);
          this.ctx.stroke();

          // Draw Arrowhead at Edge Endpoints (Identical to Operator Monitoring)
          if (this.showArrows) {
            if (!renderedEdgeKeys.has(edgeKey)) {
              this.drawEdgeArrowheads(
                srcPt.x, srcPt.y, isSrcWS,
                tgtPt.x, tgtPt.y, isTgtWS,
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

    // 2. Backward Arrowhead (Near Source if Bidirectional)
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

  drawActiveRoute() {
    this.ctx.beginPath();
    for (let i = 0; i < this.activeRoute.length; i++) {
      const wp = this.waypoints[this.activeRoute[i]];
      if (!wp) continue;
      const pt = this.worldToCanvas(wp.x, wp.y);
      if (i === 0) this.ctx.moveTo(pt.x, pt.y);
      else this.ctx.lineTo(pt.x, pt.y);
    }
    this.ctx.strokeStyle = '#059669';
    this.ctx.lineWidth = 4 * this.view.zoom;
    this.ctx.setLineDash([8 * this.view.zoom, 4 * this.view.zoom]);
    this.ctx.stroke();
    this.ctx.setLineDash([]);
  }

  drawBranchChoiceOverlays() {
    const currentWpId = this.activeRoute[this.currentStepIdx];
    const currentWp = this.waypoints[currentWpId];
    if (!currentWp) return;

    const nextWpId = this.activeRoute[this.currentStepIdx + 1] || null;
    const branches = this.adjGraph[currentWpId] || [];

    if (branches.length <= 1) return;

    const srcPt = this.worldToCanvas(currentWp.x, currentWp.y);

    branches.forEach(b => {
      const tgtWp = this.waypoints[b.target];
      if (!tgtWp) return;
      const tgtPt = this.worldToCanvas(tgtWp.x, tgtWp.y);
      const isChosen = b.target === nextWpId;

      this.ctx.beginPath();
      this.ctx.moveTo(srcPt.x, srcPt.y);
      this.ctx.lineTo(tgtPt.x, tgtPt.y);
      this.ctx.strokeStyle = isChosen ? '#059669' : '#d97706';
      this.ctx.lineWidth = (isChosen ? 5 : 3) * this.view.zoom;
      this.ctx.stroke();
    });
  }

  drawNodes() {
    for (const [wpId, wp] of Object.entries(this.waypoints)) {
      const pt = this.worldToCanvas(wp.x, wp.y);
      const isChargerNode = wp.type === 'charger' || wpId.toLowerCase().includes('charger') || wpId.toLowerCase().includes('wp-ch');
      const isWS = wp.type === 'workstation';

      const baseRadius = Math.max(3, (isWS || isChargerNode ? 6.5 : 4.0) * Math.sqrt(this.view.zoom));
      const radius = baseRadius;

      // Circle Node
      this.ctx.beginPath();
      this.ctx.arc(pt.x, pt.y, radius, 0, Math.PI * 2);
      this.ctx.fillStyle = isChargerNode ? '#d97706' : (isWS ? '#059669' : '#0284c7');
      this.ctx.fill();
      this.ctx.strokeStyle = '#ffffff';
      this.ctx.lineWidth = Math.max(1.2, 1.5 * Math.sqrt(this.view.zoom));
      this.ctx.stroke();

      // Draw Docking Orientation Pointer for Workstation & Charger Nodes
      const yawVal = wp.yaw_deg || (wp.user_pose ? wp.user_pose.yaw_deg : null);
      if ((isWS || isChargerNode) && typeof yawVal === 'number') {
        const pointerLen = radius + Math.max(6, 10 * Math.sqrt(this.view.zoom));
        const yawRad = -(yawVal * Math.PI) / 180;
        const targetX = pt.x + pointerLen * Math.cos(yawRad);
        const targetY = pt.y + pointerLen * Math.sin(yawRad);

        this.ctx.beginPath();
        this.ctx.moveTo(pt.x, pt.y);
        this.ctx.lineTo(targetX, targetY);
        this.ctx.lineWidth = Math.max(1.5, 2.5 * this.view.zoom);
        this.ctx.strokeStyle = isChargerNode ? '#d97706' : '#059669';
        this.ctx.stroke();

        const tipSize = Math.max(4, 6 * Math.sqrt(this.view.zoom));
        this.ctx.fillStyle = isChargerNode ? '#d97706' : '#059669';
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

      // Node ID Labels Centered Below Node (Identical to Operator Monitoring)
      if (this.showLabels && (this.view.zoom > 0.6 || isWS || isChargerNode)) {
        const baseFontSize = Math.max(8, Math.round(9.5 * Math.sqrt(this.view.zoom)));
        const fontSize = baseFontSize;
        this.ctx.font = `600 ${fontSize}px 'JetBrains Mono', monospace`;
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'top';

        const labelY = pt.y + radius + 4;
        const textMetrics = this.ctx.measureText(wpId);
        const textWidth = textMetrics.width;
        const padX = 4;
        const padY = 2;
        const bgX = pt.x - textWidth / 2 - padX;
        const bgY = labelY - 1;
        const bgW = textWidth + padX * 2;
        const bgH = fontSize + padY * 2;

        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.90)';
        this.ctx.strokeStyle = 'rgba(203, 213, 225, 0.8)';
        this.ctx.lineWidth = 1;

        this.ctx.beginPath();
        if (typeof this.ctx.roundRect === 'function') {
          this.ctx.roundRect(bgX, bgY, bgW, bgH, 3);
        } else {
          this.ctx.rect(bgX, bgY, bgW, bgH);
        }
        this.ctx.fill();
        this.ctx.stroke();

        this.ctx.fillStyle = isChargerNode ? '#d97706' : (isWS ? '#059669' : '#334155');
        this.ctx.fillText(wpId, pt.x, labelY + 1);

        this.ctx.textAlign = 'left';
        this.ctx.textBaseline = 'alphabetic';
      }
    }
  }

  drawRobotState() {
    if (!this.hasReceivedTelemetry || this.robotState.has_pose === false) return;
    
    let posX = this.robotState.x;
    let posY = this.robotState.y;
    let yawDeg = this.robotState.yaw_deg || 0;

    const useGzTruth = document.getElementById('chk-use-gz-truth')?.checked;
    if (useGzTruth && typeof this.robotState.gz_x === 'number') {
      posX = this.robotState.gz_x;
      posY = this.robotState.gz_y;
      yawDeg = this.robotState.gz_yaw || yawDeg;
    }

    const pt = this.worldToCanvas(posX, posY);
    const rad = (-yawDeg * Math.PI) / 180;

    // 1. AMCL Covariance Ellipse
    const covPxX = (this.robotState.covX * 100) * this.calibration.scale * this.view.zoom;
    const covPxY = (this.robotState.covY * 100) * this.calibration.scale * this.view.zoom;

    this.ctx.save();
    this.ctx.translate(pt.x, pt.y);
    this.ctx.beginPath();
    this.ctx.ellipse(0, 0, Math.max(covPxX, 8), Math.max(covPxY, 8), 0, 0, Math.PI * 2);
    this.ctx.fillStyle = useGzTruth ? 'rgba(234, 179, 8, 0.22)' : 'rgba(99, 102, 241, 0.18)';
    this.ctx.fill();
    this.ctx.strokeStyle = useGzTruth ? '#eab308' : 'rgba(99, 102, 241, 0.7)';
    this.ctx.lineWidth = 1.5;
    this.ctx.stroke();
    this.ctx.restore();

    // 2. Robot Directional Marker (Triangle)
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

    this.ctx.fillStyle = useGzTruth ? '#eab308' : '#6366f1';
    this.ctx.fill();
    this.ctx.strokeStyle = '#ffffff';
    this.ctx.lineWidth = 2 * this.view.zoom;
    this.ctx.stroke();
    this.ctx.restore();
  }
}

// Initialize on DOM load
window.addEventListener('DOMContentLoaded', () => {
  window.devApp = new DevFMSApp();
});
