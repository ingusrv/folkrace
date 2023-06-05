import Notification from "../notification.js";

const addRobot = document.querySelector("#add-robot");
const robotTable = document.querySelector("#robot-table").querySelector("tbody");
const openSockets = {};

function loadRobots() {
    robotTable.replaceChildren();
    fetch("/api/robots").then(async (res) => {
        if (res.status >= 400) {
            new Notification({ type: "error", text: "Notika kļūda ielādējot lietotāju datus!" });
            return;
        }

        const body = await res.json();
        console.log(body);

        if (Object.entries(body.data).length === 0) {
            robotTable.textContent = "Nav neviena robota";
            return;
        }

        body.data.forEach((robot, i) => {
            const row = document.createElement("tr");

            // dati
            const number = document.createElement("td");
            number.textContent = ++i;
            const robotId = document.createElement("td");
            robotId.textContent = robot.robotId;
            const createdAt = document.createElement("td");
            createdAt.textContent = robot.createdAt;
            const keyContainer = document.createElement("div");
            keyContainer.style.width = "max-content";
            const key = document.createElement("td");
            const toggleKeyVisibility = document.createElement("button");
            toggleKeyVisibility.classList.add("none", "text-outline");
            // kad būs ikonas šis mainīsies
            toggleKeyVisibility.style.fontSize = "2rem";
            toggleKeyVisibility.innerHTML = "&#9788;";
            const keyValue = document.createElement("input");
            keyValue.readOnly = true;
            keyValue.classList.add("input", "key");
            keyValue.type = "password";
            keyValue.value = robot.key;
            toggleKeyVisibility.addEventListener("click", (e) => {
                if (keyValue.type === "password") {
                    keyValue.type = "text";
                    toggleKeyVisibility.innerHTML = "&#9728;";
                } else {
                    keyValue.type = "password";
                    toggleKeyVisibility.innerHTML = "&#9788;";
                }
            });
            keyContainer.append(toggleKeyVisibility, keyValue);
            key.append(keyContainer);
            const delay = document.createElement("td");
            const delayInput = document.createElement("input");
            delayInput.classList.add("input", "delay");
            delayInput.type = "text";
            delayInput.value = 0;
            delay.append(delayInput);
            const serverStatus = document.createElement("td");
            const serverStatusValue = document.createElement("span");
            serverStatusValue.classList.add("pill", "danger");
            serverStatusValue.textContent = "Nav savienots";
            serverStatus.append(serverStatusValue);
            const robotStatus = document.createElement("td");
            const robotStatusValue = document.createElement("span");
            robotStatusValue.classList.add("pill", "danger");
            robotStatusValue.textContent = "Nav savienots";
            robotStatus.append(robotStatusValue);

            // darbības
            // actions konteinteris pazudīs kad būs ikonas
            const actionsContainer = document.createElement("div");
            actionsContainer.style.width = "max-content";
            const actions = document.createElement("td");
            const connectToServer = document.createElement("button");
            connectToServer.classList.add("button", "primary");
            connectToServer.textContent = "Savienoties";
            const startRobot = document.createElement("button");
            startRobot.classList.add("button", "primary");
            startRobot.textContent = "Sākt robota programmu";
            const openSettings = document.createElement("button");
            openSettings.classList.add("button", "secondary");
            openSettings.textContent = "Iestatījumi";
            const deleteRobot = document.createElement("button");
            deleteRobot.classList.add("button", "danger");
            deleteRobot.textContent = "Izdzēst";

            actionsContainer.append(connectToServer, startRobot, openSettings, deleteRobot);
            actions.append(actionsContainer);

            // pārbaudam vai savienojums ar serveri jau izveidots
            if (openSockets[robot.robotId]) {
                serverStatusValue.classList.remove("danger");
                serverStatusValue.classList.add("success");
                serverStatusValue.textContent = "Savienots";
                connectToServer.textContent = "Atvienoties";
            }

            // pogu funkcionalitāte
            let ws = undefined;
            let connectedToServer = false;
            let robotRunning = false;
            connectToServer.addEventListener("click", () => {
                console.log(openSockets, connectedToServer);
                if (connectedToServer) {
                    console.log(`${robot.robotId} socket atvienojas`);
                    openSockets[robot.robotId].close();
                    return;
                }

                ws = openSockets[robot.robotId];
                if (!ws) {
                    ws = new WebSocket(`ws://${window.location.host}/api/panel`);
                }

                ws.addEventListener("open", (e) => {
                    ws.send(JSON.stringify({ type: "connect", robotId: robot.robotId }));
                });

                ws.addEventListener("close", (e) => {
                    delete openSockets[robot.robotId];
                    connectToServer.textContent = "Savienoties";
                    connectedToServer = false;

                    serverStatusValue.classList.remove("success");
                    serverStatusValue.classList.add("danger");
                    serverStatusValue.textContent = "Nav savienots";
                });

                ws.addEventListener("message", (e) => {
                    const data = JSON.parse(e.data);
                    console.log(data);

                    switch (data.type) {
                        case "connect":
                            openSockets[robot.robotId] = ws;
                            connectToServer.textContent = "Atvienoties";
                            connectedToServer = true;

                            serverStatusValue.classList.remove("danger");
                            serverStatusValue.classList.add("success");
                            serverStatusValue.textContent = data.message;

                            startRobot.addEventListener("click", (e) => {
                                if (robotRunning) {
                                    ws.send(JSON.stringify({ robotId: robot.robotId, type: "stop" }));
                                    return;
                                }

                                ws.send(JSON.stringify({ robotId: robot.robotId, delay: Number(delayInput.value), type: "start" }));
                            });

                            break;
                        case "status":
                            switch (data.status.code) {
                                case 0:
                                    robotStatusValue.classList.remove("success")
                                    robotStatusValue.classList.add("danger");
                                    robotStatusValue.textContent = data.status.message;
                                    break;
                                case 1:
                                    robotStatusValue.classList.remove("danger")
                                    robotStatusValue.classList.add("success");
                                    robotStatusValue.textContent = data.status.message;
                                    break;
                                default:
                                    console.log(`Nezināms statusa kods: ${data.status.code}`);
                                    break;
                            }

                            robotRunning = data.status.running;
                            if (robotRunning) {
                                startRobot.textContent = "Beigt robota programmu";
                                return;
                            }

                            startRobot.textContent = "Sākt robota programmu";
                            break;
                        default:
                            console.log(`Nezināms ziņas tips: ${data.type}`);
                            break;
                    }
                });
            });
            openSettings.addEventListener("click", () => { });
            deleteRobot.addEventListener("click", () => {
                fetch(`/api/robot/${robot.robotId}`, {
                    method: "DELETE",
                    mode: "same-origin",
                }).then(async (res) => {
                    const body = await res.json();
                    console.log(body);
                    if (res.status >= 400) {
                        new Notification({ type: "error", text: body.message });
                        return;
                    }

                    new Notification({ type: "success", text: body.message });
                    // nav optimāli pārlādēt visu sarakstu, bet pagaidām strādā
                    loadRobots();
                });
            });

            row.append(number, robotId, createdAt, key, delay, serverStatus, robotStatus, actions);
            robotTable.appendChild(row);
        });
    });
}

addRobot.addEventListener("click", (e) => {
    fetch("/api/robot", {
        method: "POST",
        mode: "same-origin",
    }).then(async (res) => {
        const body = await res.json();
        console.log(body);
        if (res.status >= 400) {
            new Notification({ type: "error", text: body.message });
            return;
        }

        new Notification({ type: "success", text: body.message });
        // nav optimāli pārlādēt visu sarakstu, bet pagaidām strādā
        loadRobots();
    });
});

loadRobots();