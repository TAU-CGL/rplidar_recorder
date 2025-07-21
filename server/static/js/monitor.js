const { act } = require("react");

var monitorLeftPanel = document.querySelector(".monitor-left-panel");
var monitorRightPanel = document.querySelector(".monitor-right-panel");


var activeScans = {
    name: "",
    timestamps: [],
    currentIndex: -1,
    currentScan: null
}

function update_current_scan() {
    let payload = new URLSearchParams({
        contraption_nickname: activeScans.name,
        timestamp: activeScans.timestamps[activeScans.currentIndex]
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
            console.log(activeScans);
        })
    })
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
            activeScans.timestamps = data.map(scan => scan.timestamp);
            activeScans.timestamps.sort((a, b) => new Date(a) - new Date(b));
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
        lastSeen = lastSeen.split(".")[0];
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