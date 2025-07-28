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

function fetch_all_recent_scans() {
    fetch('/api/contraption/list')
        .then(response => response.json().then(data => {
            let devices = [];
            data.forEach(dev => devices.push(dev.nickname));
            console.log(devices);
    }))
}