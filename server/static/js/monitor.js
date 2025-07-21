var monitorLeftPanel = document.querySelector(".monitor-left-panel");
var monitorRightPanel = document.querySelector(".monitor-right-panel");

var nextScanButton = document.querySelector("#next-scan");
nextScanButton.addEventListener("click", event => on_prev_next_scan_click(1, event));
var prevScanButton = document.querySelector("#prev-scan");
prevScanButton.addEventListener("click", event => on_prev_next_scan_click(-1, event));

var monitorNoSelection = document.querySelector(".monitor-no-selection-text");
var monitorViewport = document.querySelector(".monitor-preview-container");

var activeScans = {
    name: "",
    timestamps: [],
    currentIndex: -1,
    currentScan: null
}

function format_ts(ts) {
    return ts.split(".")[0];
}

function update_current_scan() {
    // Update the scan label
    document.querySelector("#monitor-preview-label-idx").innerText = 
        (activeScans.currentIndex + 1).toString() + "/" + (activeScans.timestamps.length).toString();
    document.querySelector("#monitor-preview-label-ts").innerText = format_ts(activeScans.timestamps[activeScans.currentIndex].ts);

    let payload = new URLSearchParams({
        contraption_nickname: activeScans.name,
        scan_id: activeScans.timestamps[activeScans.currentIndex].id
    }).toString();
    fetch("/api/contraption/get/scan", {
        method: "POST",
        headers: {            
            "Content-Type": "application/x-www-form-urlencoded"
        }, 
        body: payload
    }).then(response => {
        response.json().then(data => {
            activeScans.currentScan = data["ranges"];
            update_current_scan_viewport();
        })
    })
}

function update_current_scan_viewport() {
    monitorNoSelection.style.display = 'none';
    monitorViewport.innerText = activeScans.currentScan;
}

function on_prev_next_scan_click(dir, event) {
    activeScans.currentIndex += dir;
    if (activeScans.currentIndex >= activeScans.timestamps.length)
        activeScans.currentIndex = activeScans.timestamps.length - 1;
    if (activeScans.currentIndex < 0)
        activeScans.currentIndex = 0;
    update_current_scan();
}


function on_contraption_click(name, event) {
    console.log("Contraption clicked:", name);
    let payload = new URLSearchParams({
        contraption_nickname: name
    }).toString();
    fetch("/api/contraption/list/scans", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded"
        },
        body: payload
    }).then(response => {
        console.log("Response from contraption scans:", response);
        response.json().then(data => {
            activeScans.timestamps = data.map(scan => new Object({ts: scan.timestamp, id: scan.id}));
            activeScans.timestamps.sort((a, b) => new Date(a.ts) - new Date(b.ts));
            activeScans.currentIndex = activeScans.timestamps.length - 1;
            activeScans.name = name;
            update_current_scan();
        })
    })
}




function add_contraption_div(name, lastSeen, online) {
    let contraption = document.createElement("div");
    contraption.classList.add("monitor-contraption");
    contraption.addEventListener("click", (event) => on_contraption_click(name, event));

    let contraptionHeader = document.createElement("h3");
    contraptionHeader.innerText = name;
    contraption.appendChild(contraptionHeader);

    let contraptionLastSeen = document.createElement("p");
    contraptionLastSeen.innerText = "Last seen:";
    contraptionLastSeen.classList.add("monitor-last-seen");
    contraptionLastSeen.appendChild(document.createElement("br"));
    let contraptionLastSeenTime = document.createElement("span");
    contraptionLastSeenTime.innerText = lastSeen;
    contraptionLastSeen.appendChild(contraptionLastSeenTime);
    contraption.appendChild(contraptionLastSeen);

    let contraptionStatus = document.createElement("span");
    contraptionStatus.classList.add("monitor-status");
    if (online)
        contraptionStatus.classList.add("monitor-online");
    else
        contraptionStatus.classList.add("monitor-offline");
    contraption.appendChild(contraptionStatus);

    monitorLeftPanel.appendChild(contraption);
}
function populate_contraptions(data) {
    monitorLeftPanel.innerHTML = "";
    data.sort((a,b) => a.nickname > b.nickname).forEach(element => {
        let lastSeen = element.last_scan != null ? element.last_scan : "---";
        lastSeen = format_ts(lastSeen);
        add_contraption_div(element.nickname, lastSeen, element.online);
    })
}

function periodic_update() {
    fetch("/api/contraption/list")
    .then(response => {
        return response.json();
    }).then(data => {
        console.log(data);
        populate_contraptions(data);
    });
}

add_contraption_div("dev1", "2025-07-07T20:42", true);
add_contraption_div("dev2", "2025-07-07T20:41", false);
add_contraption_div("dev3", "2025-07-07T20:44", false);
add_contraption_div("dev4", "2025-07-07T20:42", true);
add_contraption_div("dev5", "2025-07-07T20:44", true);

periodic_update();