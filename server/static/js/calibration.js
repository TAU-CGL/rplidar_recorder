async function get_device_list() {
    let response = await fetch('/api/contraption/list');
    let data = await response.json();
    let devices = [];
    data.forEach(dev => devices.push(dev.nickname));
    devices.sort();
    return devices;
}

async function fetch_recent_scans() {
    let devices = await get_device_list();
    let result = {};
    
    for(let i = 0; i < devices.length; i++) {
        let device = devices[i];

        // Fetch all timestamps and find out which was is most recent
        let payload = new URLSearchParams({contraption_nickname: device}).toString();
        let response = await fetch("/api/contraption/list/scans", {
            method: "POST",
            headers: {
                "Content-Type": "application/x-www-form-urlencoded"
            },
            body: payload
        });
        let data = await response.json();
        let timestamps = data.map(scan => new Object({ts: scan.timestamp, id: scan.id}));
        timestamps.sort((a, b) => new Date(a.ts) - new Date(b.ts));
        let lastTimestamp = timestamps[timestamps.length - 1];
        console.log(lastTimestamp);

        // Fetch the most recent timestamp's data
        payload = new URLSearchParams({
            contraption_nickname: device,
            scan_id: lastTimestamp.id
        }).toString();
        response = await fetch("/api/contraption/get/scan", {
            method: "POST",
            headers: {
                "Content-Type": "application/x-www-form-urlencoded"
            },
            body: payload
        });
        data = await response.json();
        let ranges = data["ranges"];
        
        // Convert ranges to points
        let rawScan = JSON.parse(ranges.replaceAll("\'", "\"").replaceAll("inf", "-1"));
        let points = [];
        let N = rawScan.ranges.length;
        for (let i = 0; i < N; i++) {
            let theta = i / (N-1) * 2 * Math.PI;
            let val = rawScan.ranges[i];
            if (val < 0) continue;
            let x = val * Math.cos(theta);
            let y = val * Math.sin(theta);
            points.push([x, y])
        }

        result[device] = points;
    }

    return result;
}

async function populate_use_alternative_radius() {
    let devices = await get_device_list();
    let div = document.getElementById("contraption-checkboxes");
    devices.forEach(device => {
        let label = document.createElement("label");
        let checkbox = document.createElement("input");
        checkbox.type = "checkbox";
        checkbox.name = "contraption";
        checkbox.value = device;
        checkbox.checked = false; // Default to unchecked
        label.appendChild(checkbox);
        label.appendChild(document.createTextNode(` ${device} `));
        div.appendChild(label);
    });
}

async function fit_circles() {
    let scans = await fetch_recent_scans();
    let radius = parseFloat(document.getElementById("radius").value);
    let altRadius = parseFloat(document.getElementById("alternativeRadius").value);
    let altRadiusDevices = [];
    let checkboxes = document.querySelectorAll('input[name="contraption"]:checked');
    checkboxes.forEach(checkbox => {
        altRadiusDevices.push(checkbox.value);
    });
    let payload = new URLSearchParams({
        scans: JSON.stringify(scans),
        radius: radius,
        altRadius: altRadius,
        altRadiusDevices: JSON.stringify(altRadiusDevices)
    }).toString();

    let response = await fetch("/api/calibration/fit_circles", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded"
        },
        body: payload
    });

    return response.text();
}
function fit_circles_with_ui(id) {
    fit_circles().then(result => {
        let text = document.getElementById("text" + id.toString());
        text.value = result.toString();
    })
}

async function visualize_fit_circles(id) {
    let scans = await fetch_recent_scans();
    let circles = document.getElementById("text" + id.toString()).value;
    let payload = new URLSearchParams({
        scans: JSON.stringify(scans),
        circles: circles
    }).toString();
    let response = await fetch("/api/calibration/fit_circles/visualize", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded"
        },
        body: payload
    });
    response.blob().then(blob => {
        let img = document.getElementById("image" + id.toString());
        img.src = URL.createObjectURL(blob);
        img.onload = function() {
            URL.revokeObjectURL(img.src); // Clean up the object URL
        };
    });
}

async function calibrate() {
    let circle1 = document.getElementById("text1").value;
    let circle2 = document.getElementById("text2").value;

    let payload = new URLSearchParams({
        circle1: circle1,
        circle2: circle2
    }).toString();

    let response = await fetch("/api/calibration/calibrate", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded"
        },
        body: payload
    });

    let text = document.getElementById("calibration-text");
    text.value = await response.text();
}

// Call populate_use_alternative_radius on page load
document.addEventListener("DOMContentLoaded", function() {
    populate_use_alternative_radius();
});