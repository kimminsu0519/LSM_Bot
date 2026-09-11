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
    this.sidebarCollapsed = false;

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
      offsetX: 0.0,
      offsetY: 0.0,
      scale: 1.0,
      yaw: 0.0,
      pxPerMeter: 100.0 // 1m = 100pt Uniform Scale
    };

    // Display Toggles (Default showUnconnected = false)
    this.showEdges = true;
    this.showArrows = true;
    this.showLabels = true;
    this.showUnconnected = false; // 기본: 고립 노드 숨김
    this.activeFilters = new Set(['workstation', 'charger', 'node']); // 다중 선택 필터 (기본: 모두 선택)

    this.incomingEdgesCount = {};

    // Elements
    this.canvas = document.getElementById('map-canvas');
    this.ctx = this.canvas.getContext('2d');
    this.container = document.getElementById('canvas-container');

    this.init();
  }

  async init() {
    this.applyTheme(this.currentTheme);
    this.loadSavedCalibration();
    this.setupEventListeners();
    this.resizeCanvas();
    window.addEventListener('resize', () => this.resizeCanvas());

    await Promise.all([this.loadMapImage(), this.loadYAMLData()]);

    this.centerView();
    this.render();
  }

  applyTheme(theme) {
    this.currentTheme = theme;
    document.documentElement.setAttribute('data-theme', theme);
    localStorage.setItem('lsm_fms_theme', theme);

    const icon = document.getElementById('icon-theme');
    const label = document.getElementById('label-theme');
    if (theme === 'dark') {
      icon.className = 'fa-solid fa-sun';
      label.textContent = 'Light Mode';
    } else {
      icon.className = 'fa-solid fa-moon';
      label.textContent = 'Dark Mode';
    }
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
    document.getElementById('num-offset-x').value = this.calibration.offsetX;
    document.getElementById('rng-offset-x').value = this.calibration.offsetX;
    document.getElementById('num-offset-y').value = this.calibration.offsetY;
    document.getElementById('rng-offset-y').value = this.calibration.offsetY;
    document.getElementById('num-scale').value = this.calibration.scale;
    document.getElementById('rng-scale').value = this.calibration.scale;
    document.getElementById('num-yaw').value = this.calibration.yaw;
    document.getElementById('rng-yaw').value = this.calibration.yaw;
  }

  async loadMapImage() {
    return new Promise((resolve) => {
      this.mapImage.src = 'assets/창고맵_이미지용_점제거.png';
      this.mapImage.onload = () => {
        this.mapLoaded = true;
        console.log(`Map Image Loaded: ${this.mapImage.width}x${this.mapImage.height} pt`);
        resolve();
      };
      this.mapImage.onerror = () => {
        console.warn('Map Image load failed from assets/창고맵_이미지용_점제거.png');
        resolve();
      };
    });
  }

  async loadYAMLData() {
    try {
      const res = await fetch('config/waypoints_graph2.yaml');
      const text = await res.text();
      this.yamlData = jsyaml.load(text);
      
      this.metadata = this.yamlData.metadata || {};
      this.waypoints = this.yamlData.waypoints || {};

      this.calculateEdgeDegree();
      this.updateStats();
      this.populateWaypointsList();
      console.log(`Loaded ${Object.keys(this.waypoints).length} waypoints.`);
    } catch (err) {
      console.error('Error fetching waypoints_graph2.yaml:', err);
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
    this.calibration.offsetX = 0.0;
    this.calibration.offsetY = 0.0;
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

    document.getElementById('stat-total-wp').textContent = total;
    document.getElementById('stat-workstations').textContent = wsCount;
    document.getElementById('stat-chargers').textContent = chargerCount;
    document.getElementById('stat-nodes').textContent = nodeCount;
    document.getElementById('cnt-all').textContent = total;
    document.getElementById('cnt-ws').textContent = wsCount;
    document.getElementById('cnt-charger').textContent = chargerCount;
    document.getElementById('cnt-node').textContent = nodeCount;
  }

  resizeCanvas() {
    this.canvas.width = this.container.clientWidth;
    this.canvas.height = this.container.clientHeight;
    this.render();
  }

  centerView() {
    if (this.mapLoaded) {
      this.view.panX = (this.canvas.width - this.mapImage.width * this.view.zoom) / 2;
      this.view.panY = (this.canvas.height - this.mapImage.height * this.view.zoom) / 2;
    } else {
      this.view.panX = this.canvas.width / 2;
      this.view.panY = this.canvas.height / 2;
    }
    this.updateZoomBadge();
  }

  updateZoomBadge() {
    document.getElementById('zoom-level').textContent = `${Math.round(this.view.zoom * 100)}%`;
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

        // Draw contrast halo outline so label is 100% visible on white/dark backgrounds
        this.ctx.lineWidth = Math.max(2.5, 3.5 * Math.sqrt(this.view.zoom));
        this.ctx.strokeStyle = this.currentTheme === 'dark' ? 'rgba(10, 14, 23, 0.85)' : 'rgba(255, 255, 255, 0.95)';
        this.ctx.strokeText(id, screenPos.x, labelY);

        // Fill text color (Selected: Neon Cyan / Accent Blue, enlarged)
        if (isSelected) {
          this.ctx.fillStyle = this.currentTheme === 'dark' ? '#00f3ff' : '#0284c7';
        } else if (isHovered) {
          this.ctx.fillStyle = this.currentTheme === 'dark' ? '#ffffff' : '#0f172a';
        } else if (isChargerNode) {
          this.ctx.fillStyle = this.currentTheme === 'dark' ? '#ffb700' : '#d97706';
        } else if (isWS) {
          this.ctx.fillStyle = this.currentTheme === 'dark' ? '#00ff9d' : '#059669';
        } else {
          this.ctx.fillStyle = this.currentTheme === 'dark' ? '#94a3b8' : '#64748b';
        }

        this.ctx.fillText(id, screenPos.x, labelY);
        
        this.ctx.textAlign = 'left';
        this.ctx.textBaseline = 'alphabetic';
      }
    });
  }

  // Event Listeners Setup
  setupEventListeners() {
    document.getElementById('btn-theme-toggle').onclick = () => this.toggleTheme();
    document.getElementById('btn-toggle-sidebar').onclick = () => this.toggleSidebar();
    document.getElementById('btn-auto-fit').onclick = () => this.autoFitGraph(true);

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
      document.getElementById('cursor-coords').textContent =
        `X: ${coords.wx.toFixed(2)}m | Y: ${coords.wy.toFixed(2)}m`;

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
    document.getElementById('btn-zoom-in').onclick = () => {
      this.view.zoom = Math.min(5.0, this.view.zoom * 1.2);
      this.updateZoomBadge();
      this.render();
    };

    document.getElementById('btn-zoom-out').onclick = () => {
      this.view.zoom = Math.max(0.2, this.view.zoom / 1.2);
      this.updateZoomBadge();
      this.render();
    };

    document.getElementById('btn-reset-view').onclick = () => {
      this.view.zoom = 1.0;
      this.centerView();
      this.render();
    };

    document.getElementById('chk-show-edges').onchange = (e) => {
      this.showEdges = e.target.checked;
      this.render();
    };

    document.getElementById('chk-show-arrows').onchange = (e) => {
      this.showArrows = e.target.checked;
      this.render();
    };

    document.getElementById('chk-show-labels').onchange = (e) => {
      this.showLabels = e.target.checked;
      this.render();
    };

    document.getElementById('chk-show-unconnected').onchange = (e) => {
      this.showUnconnected = e.target.checked;
      this.render();
    };

    // Sidebar Tab Switching
    document.querySelectorAll('.tab-btn').forEach(btn => {
      btn.onclick = () => {
        document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

        btn.classList.add('active');
        const tabId = btn.getAttribute('data-tab');
        document.getElementById(tabId).classList.add('active');
      };
    });

    // Calibration Controls Mapping
    const bindControl = (numId, rngId, propKey, isFloat = true) => {
      const num = document.getElementById(numId);
      const rng = document.getElementById(rngId);

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

    document.getElementById('btn-reset-calibration').onclick = () => {
      this.autoFitGraph(true);
    };

    document.getElementById('btn-save-calibration').onclick = () => this.saveCalibration();

    document.getElementById('btn-export-config').onclick = () => {
      const json = JSON.stringify(this.calibration, null, 2);
      const blob = new Blob([json], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'fms_map_calibration.json';
      a.click();
    };

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

    document.getElementById('input-search-wp').oninput = (e) => {
      const q = e.target.value.toLowerCase();
      this.populateWaypointsList(q);
    };
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
}

// Instantiate App
window.addEventListener('DOMContentLoaded', () => {
  window.app = new FMSApp();
});
