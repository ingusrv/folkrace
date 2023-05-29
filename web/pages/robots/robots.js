const main = document.querySelector("main");
fetch("/api/v1/robots").then(async (res) => {
    // robotId, key, status, lastUpdated
    const body = await res.json();
    console.log(body);

    if (body.data == {}) {
        return;
    }

    body.data.forEach((item) => {
        const el = createPanel(item);
        main.appendChild(el);
    });

    setupPanels();
});

// TODO: make this more responsive
const addRobot = document.querySelector("#add-robot");
const statusMessage = document.querySelector("#status-message");
addRobot.addEventListener("click", (e) => {
    fetch("/api/v1/robot", {
        method: "POST",
        mode: "same-origin",
    }).then(async (res) => {
        const body = await res.json();
        console.log(body);
        if (res.status >= 400) {
            statusMessage.style.color = "lightcoral";
            statusMessage.innerText = body.message;
        }
        if (res.status === 201) {
            statusMessage.style.color = "lightgreen";
            statusMessage.innerText = body.message;
        }
    });
});

function createPanel(robotData) {
    const div = document.createElement("div");
    // TODO: fix potential XSS
    div.innerHTML = `
    <div class="panel grid-1col-gap relative" data-robot-id="${robotData.robotId}">
        <h2 class="margin-0">Robots '${robotData.robotId}'</h2>
        <button class="button delete-robot" data-delete-robot>Izdzēst</button>
        <div class="connection-token" data-connection-token>
            <button class="button primary show-connection-token" data-show-connection-token>Parādīt savienošanās atslēgu</button>
            <input class="input connection-token-input" type="password" name="token" value="${robotData.key}" readonly>
            <button class="button primary generate-new-token" data-generate-new-token>Izveidot jaunu savienošanās atslēgu</button>
        </div>
        <div class="server-controls" data-server-controls>
            <button class="button primary inline" data-connect-to-server>Savienoties</button>
            <p class="inline">Savienojuma statuss: <span class="text-red" data-server-status>Nav Savienots</span>
            </p>
        </div>
        <div class="robot-controls hidden" data-robot-controls>
            <button class="button primary inline" data-start-robot>Sākt robota
                programmu</button>
            <p class="inline">Robota statuss: <span class="text-gray" data-robot-status>Bezsaistē</span></p>
        </div>
    </div>
    `;
    return div;
}

function setupPanels() {
    const connectionTokens = document.querySelectorAll("[data-connection-token]");
    connectionTokens.forEach((root) => {
        const toggle = root.querySelector("[data-show-connection-token]");
        const token = root.querySelector("input");
        const generateNewToken = root.querySelector("[data-generate-new-token]");
        const robotId = root.parentElement.dataset.robotId;
        toggle.addEventListener("click", (e) => {
            if (token.type === "password") {
                token.type = "text";
                toggle.innerText = "Paslēpt savienošanās atslēgu";
            } else {
                token.type = "password";
                toggle.innerText = "Parādīt savienošanās atslēgu";
            }
        });
        generateNewToken.addEventListener("click", (e) => {
            fetch(`/api/v1/robotToken/${robotId}`, {
                method: "POST",
                mode: "same-origin",
            }).then(async (res) => {
                const body = await res.json();
                console.log(body);
                if (res.status >= 400) {
                    statusMessage.style.color = "lightcoral";
                    statusMessage.innerText = body.message;
                }
                if (res.status === 200) {
                    statusMessage.style.color = "lightgreen";
                    statusMessage.innerText = body.message;
                }
            });
        });
    });

    const panels = document.querySelectorAll("[data-robot-id]");
    panels.forEach((panel) => {
        const deleteRobot = panel.querySelector("[data-delete-robot]");
        const robotId = panel.dataset.robotId;
        deleteRobot.addEventListener("click", (e) => {
            fetch(`/api/v1/robot/${robotId}`, {
                method: "DELETE",
                mode: "same-origin",
            }).then(async (res) => {
                const body = await res.json();
                console.log(body);
                if (res.status >= 400) {
                    statusMessage.style.color = "lightcoral";
                    statusMessage.innerText = body.message;
                }
                if (res.status === 200) {
                    statusMessage.style.color = "lightgreen";
                    statusMessage.innerText = body.message;
                }
            });
        });
    });

    let openSockets = [];
    const serverControls = document.querySelectorAll("[data-server-controls]");
    serverControls.forEach((root) => {
        const connectToServer = root.querySelector("[data-connect-to-server]");
        const serverStatus = root.querySelector("[data-server-status]");
        const robotId = root.parentElement.dataset.robotId;
        const robotControls = root.parentElement.querySelector("[data-robot-controls]");
        const startRobot = robotControls.querySelector("[data-start-robot]");
        const robotStatus = robotControls.querySelector("[data-robot-status]");
        connectToServer.addEventListener("click", (e) => {
            if (connectToServer.innerText === "Atvienoties") {
                console.log(openSockets[robotId]);
                openSockets[robotId].close(1000, String(robotId));
                return;
            }
            console.log(robotId);
            const ws = new WebSocket(`ws://${window.location.host}/api/v1/panel`);
            let programStarted = false;
            ws.addEventListener("close", (e) => {
                console.log(e);
                connectToServer.innerText = "Savienoties";
                serverStatus.classList.remove("text-green");
                serverStatus.classList.add("text-red");
                serverStatus.innerText = "Nav savienots";
                robotControls.classList.add("hidden");
            });
            ws.addEventListener("message", (e) => {
                const data = JSON.parse(e.data);
                console.log(data);
                if (data.code === 200 && data.type === "connect" && data.origin === "server") {
                    serverStatus.classList.remove("text-red");
                    serverStatus.classList.add("text-green");
                    serverStatus.innerText = data.message;
                    robotControls.classList.remove("hidden");
                    startRobot.addEventListener("click", (e) => {
                        if (programStarted === true) {
                            ws.send(JSON.stringify({ robotId: robotId, type: "stop" }));
                            startRobot.innerText = "Sākt robota programmu";
                            programStarted = false;
                        } else {
                            programStarted = true;
                            ws.send(JSON.stringify({ robotId: robotId, type: "start" }));
                            startRobot.innerText = "Beigt robota programmu";
                        }
                    });
                    return;
                }
                if (data.code === 200 && data.type === "connect" && data.origin === "robot") {
                    robotStatus.classList.remove("text-gray");
                    robotStatus.classList.remove("text-red");
                    robotStatus.classList.add("text-green");
                    robotStatus.innerText = data.message;
                    return;
                }
                if (data.code === 200 && data.type === "start") {
                    robotStatus.classList.remove("text-gray");
                    robotStatus.classList.remove("text-red");
                    robotStatus.classList.add("text-green");
                    robotStatus.innerText = data.message;
                    return;
                }
                if (data.code === 200 && data.type === "stop") {
                    robotStatus.classList.remove("text-gray");
                    robotStatus.classList.remove("text-green");
                    robotStatus.classList.add("text-red");
                    robotStatus.innerText = data.message;
                    return;
                }
            });
            ws.addEventListener("open", (e) => {
                ws.send(JSON.stringify({ robotId: robotId, type: "connect" }));
                connectToServer.innerText = "Atvienoties";
                openSockets[robotId] = ws;
            });

        });
    });
}
