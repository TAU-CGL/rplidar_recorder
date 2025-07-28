function sendNewContraption() {
    fetch('/api/contraption/register')
        .then(response => response.text())
        .then(data => {
            document.getElementById('new-contraption-result').value = data;
        })
        .catch(error => {
            document.getElementById('new-contraption-result').value = 'Error: ' + error;
        });
}

function deleteAllScans() {
    fetch('/api/contraption/scan/delete', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        alert('All scans deleted successfully');
    })
    .catch(error => {
        alert('Error deleting scans: ' + error);
    });
}

async function fetch_all_recent_scans() {
    let response = await fetch('/api/contraption/list');
    let data = await response.json();
    let devices = [];
    data.forEach(dev => devices.push(dev.nickname));
    devices.sort();
    console.log(devices);
}