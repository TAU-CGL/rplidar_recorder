var monitorLeftPanel = document.querySelector(".monitor-left-panel");
var monitorRightPanel = document.querySelector(".monitor-right-panel");

function add_contraption_div(name, lastSeen, online) {
    let contraption = document.createElement("div");
    contraption.classList.add("monitor-contraption");

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