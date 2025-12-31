// Questo script è complesso: gestisce il disegno su Canvas e la conversione coordinate
const ROS_URL = 'ws://' + window.location.hostname + ':9090';
const ros = new ROSLIB.Ros({ url: ROS_URL });

// Variabili stato
let currentMode = null; // 'room' o 'no_go'
let isDrawing = false;
let startPixel = null;
let currentRect = null; // Oggetto grafico EaselJS
let stage = null; // EaselJS Stage

// Inizializza Viewer
const viewer = new ROS2D.Viewer({
    divID: 'editor-canvas',
    width: window.innerWidth,
    height: window.innerHeight,
    background: '#222'
});

// Layer mappa
const gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
});

// Layer disegno (sopra la mappa)
const drawLayer = new createjs.Container();
viewer.scene.addChild(drawLayer);

// Gestione Zoom/Pan automatico all'avvio
gridClient.on('change', () => {
    viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    stage = viewer.scene.stage; // Ottieni riferimento allo stage
    setupInteraction();
});

function setupInteraction() {
    // Mouse Events sul Canvas
    const canvas = document.querySelector('#editor-canvas canvas');
    
    canvas.addEventListener('mousedown', (e) => {
        if(!currentMode) return;
        isDrawing = true;
        // Ottieni coordinate relative al mondo ROS (non pixel schermo)
        const coords = viewer.scene.globalToLocal(e.clientX, e.clientY);
        startPixel = coords;
        
        // Crea rettangolo provvisorio
        currentRect = new createjs.Shape();
        drawLayer.addChild(currentRect);
    });

    canvas.addEventListener('mousemove', (e) => {
        if(!isDrawing || !startPixel) return;
        const coords = viewer.scene.globalToLocal(e.clientX, e.clientY);
        
        currentRect.graphics.clear();
        const color = currentMode === 'room' ? 'rgba(0, 255, 0, 0.3)' : 'rgba(255, 0, 0, 0.3)';
        const border = currentMode === 'room' ? '#0f0' : '#f00';
        
        currentRect.graphics.beginStroke(border)
                            .beginFill(color)
                            .drawRect(startPixel.x, startPixel.y, coords.x - startPixel.x, coords.y - startPixel.y);
    });

    canvas.addEventListener('mouseup', (e) => {
        if(!isDrawing) return;
        isDrawing = false;
        
        // Apri modale per salvare
        const modal = new bootstrap.Modal(document.getElementById('nameModal'));
        modal.show();
        
        // Salva riferimento all'ultimo rettangolo disegnato
        window.lastDrawnRect = {
            start: startPixel,
            end: viewer.scene.globalToLocal(e.clientX, e.clientY)
        };
    });
}

// Bottoni Toolbar
document.getElementById('btn-draw-room').onclick = () => { currentMode = 'room'; };
document.getElementById('btn-draw-nogo').onclick = () => { currentMode = 'no_go'; };

// Salvataggio
document.getElementById('btn-confirm-name').onclick = async () => {
    const name = document.getElementById('zone-name-input').value;
    if(!name) return;

    // Converti da coordinate interne Viewer a Metri Reali
    // Nota: ROS2D usa già coordinate metriche se gridClient è caricato bene.
    // Ma l'origine potrebbe essere traslata.
    
    const p1 = window.lastDrawnRect.start;
    const p2 = window.lastDrawnRect.end;
    
    // Normalizza rettangolo (min/max)
    const x1 = Math.min(p1.x, p2.x);
    const y1 = Math.min(p1.y, p2.y);
    const x2 = Math.max(p1.x, p2.x);
    const y2 = Math.max(p1.y, p2.y);

    const zoneData = {
        name: name,
        type: currentMode,
        coordinates: [
            {x: x1, y: y1},
            {x: x2, y: y1},
            {x: x2, y: y2},
            {x: x1, y: y2}
        ]
    };

    // Invia al Backend
    await fetch('/api/zones', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(zoneData)
    });

    // Reset UI
    bootstrap.Modal.getInstance(document.getElementById('nameModal')).hide();
    document.getElementById('zone-name-input').value = '';
    drawLayer.removeAllChildren();
    currentMode = null;
    alert('Zone Saved!');
};