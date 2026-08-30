function connectWS() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(proto + '://' + location.host + '/ws');

    ws.onopen = () => {
        const el = document.getElementById('ws-status');
        el.textContent = 'Connected';
        el.className = 'badge bg-success';
    };

    ws.onclose = () => {
        const el = document.getElementById('ws-status');
        el.textContent = 'Disconnected';
        el.className = 'badge bg-danger';
        setTimeout(connectWS, 2000);
    };

    ws.onmessage = (ev) => {
        try {
            const data = JSON.parse(ev.data);
            if ('credit' in data) document.getElementById('credit').textContent = data.credit;
            if ('bank' in data) document.getElementById('bank').textContent = data.bank;
            if ('gameCount' in data) document.getElementById('gameCount').textContent = data.gameCount;
            if ('xfer' in data) document.getElementById('transfer').textContent = data.xfer;
            if ('incomeTotal' in data) document.getElementById('incomeTotal').textContent = data.incomeTotal;
            if ('payoutTotal' in data) document.getElementById('payoutTotal').textContent = data.payoutTotal;
            if ('vol' in data) document.getElementById('volume').textContent = data.vol;
        } catch (e) {
            console.error('WS parse error', e);
        }
    };
}

connectWS();

document.getElementById('volume').addEventListener('input', async e => {
    try {
        const res = await fetch('/api/status', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: "{\"volume\": " + e.target.value + "}",
        });
        const text = await res.text();
        statusEl.textContent = text;
    } catch (err) {
        statusEl.textContent = 'Update failed: ' + err;
    }
});

// OTA upload
document.getElementById('ota-form').addEventListener('submit', async (e) => {
    e.preventDefault();
    var otafile = document.getElementById('ota-file');
    const statusEl = document.getElementById('ota-status');
    if (!fileInput.files.length) return;

    const file = fileInput.files[0];
    statusEl.textContent = 'Uploading...';

    try {
        const res = await fetch('/update', {
            method: 'POST',
            headers: {'Content-Type': 'application/octet-stream'},
            body: file
        });
        const text = await res.text();
        statusEl.textContent = text;
    } catch (err) {
        statusEl.textContent = 'Upload failed: ' + err;
    }
});