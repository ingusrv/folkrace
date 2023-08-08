import { addDriveData, getRobotByKey, getRobotById } from "../src/db.js";
import { WebSocketServer } from "ws";

import { mongoClient } from "../src/databaseClient.js";

export default function websocketRouter(req, socket, head) {
    const pathname = req.url;
    console.log("requested connection upgrade on", pathname);

    if (pathname === `/api/panel`) {
        // TODO: auth
        robotPanelWss.handleUpgrade(req, socket, head, (ws) => {
            robotPanelWss.emit("connection", ws, req);
        });
    } else if (pathname === `/api/robot`) {
        robotControlWss.handleUpgrade(req, socket, head, (ws) => {
            robotControlWss.emit("connection", ws, req);
        });
    } else {
        socket.destroy();
    }
}

const connectedClients = {};
const robotPanelWss = new WebSocketServer({ noServer: true });
robotPanelWss.on("connection", (ws) => {
    ws.on("close", (e) => {
        console.log(`client connection stopped with code ${e}`);
    });

    ws.on("message", async (e) => {
        const data = JSON.parse(e);
        console.log(data);

        if (!data.robotId || !data.type) {
            ws.send(JSON.stringify({ code: 400, type: "error", message: "Notika kļūda" }));
            return;
        }

        const robot = await getRobotById(mongoClient, data.robotId);

        if (!robot) {
            ws.send(JSON.stringify({ code: 404, type: "error", message: `Robots "${data.robotId}" netika atrasts` }));
            return;
        }

        const robotClient = connectedRobots[robot.robotId];
        let robotWs = undefined;
        let robotStatus = { code: 0, running: false, message: "Nav savienots" };
        if (robotClient) {
            robotWs = robotClient.websocket;
            robotStatus = robotClient.status;
        }

        switch (data.type) {
            case "connect":
                connectedClients[robot.robotId] = {
                    websocket: ws,
                    connected: true
                };

                ws.send(JSON.stringify({
                    code: 200,
                    type: "connect",
                    robotId: robot.robotId,
                    message: "Savienots"
                }));

                ws.send(JSON.stringify({ type: "status", robotId: robot.robotId, status: robotStatus }));
                break;
            case "start":
                if (!robotWs) {
                    ws.send(JSON.stringify({ code: 500, type: "error", message: `Robots "${robot.robotId}" nav savienots` }));
                    return;
                }

                if (!data.delay) {
                    data.delay = 0;
                }

                robotWs.send(JSON.stringify({
                    type: "start",
                    delay: data.delay
                }));
                break;
            case "stop":
                if (!robotWs) {
                    ws.send(JSON.stringify({ code: 500, type: "error", message: `Robots "${robot.robotId}" nav savienots` }));
                    return;
                }

                robotWs.send(JSON.stringify({
                    type: "stop",
                }));
                break;
            case "status":
                ws.send(JSON.stringify({ type: "status", robotId: robot.robotId, status: robotStatus }));
                break;
            default:
                ws.send(JSON.stringify({ code: 400, type: "error", message: "Nezināms ziņas tips" }));
                break;
        }
    });
});

const connectedRobots = {};
const robotControlWss = new WebSocketServer({ noServer: true });
robotControlWss.on("connection", (ws) => {
    ws.on("close", (e) => {
        console.log(`robot: client connection stopped with code ${e}`);
        for (const key in connectedRobots) {
            console.log(key);
            const robot = connectedRobots[key];

            if (robot.websocket.readyState === 3) {
                robot.connected = false;
                robot.status = { code: 0, running: false, message: "Nav savienots" };

                const clientWs = connectedClients[key];
                if (clientWs) {
                    clientWs.websocket.send(JSON.stringify({ type: "status", robotId: key, status: robot.status }));
                }
            }
        }
    });

    ws.on("message", async (e) => {
        const data = JSON.parse(e);
        console.log("robot: ", data);

        if (!data.key || !data.type) {
            ws.send(JSON.stringify({ code: 400, type: "error", message: "Notika kļūda" }));
            return;
        }

        const robot = await getRobotByKey(mongoClient, data.key);

        if (!robot) {
            ws.send(JSON.stringify({ code: 404, type: "error", message: "Robots ar tādu atslēgu netika atrasts" }));
            return;
        }

        // ja nav klienta, tad izveidojam fake klientu lai viss tālāk strādātu
        const client = connectedClients[robot.robotId];
        let clientWs = { send: () => { } };
        if (client) {
            clientWs = client.websocket;
        }

        switch (data.type) {
            case "connect":
                console.log(robot);
                connectedRobots[robot.robotId] = {
                    websocket: ws,
                    connected: true,
                    status: { code: 1, running: false, message: "Savienots" }
                };

                ws.send(JSON.stringify({ code: 200, type: "connect", robotId: `${robot.robotId}`, message: "Robots savienots" }))
                break;
            case "start":
                console.log("we have received start");
                connectedRobots[robot.robotId].status = { code: 1, running: true, message: "Programma startēta" };
                break;
            case "stop":
                console.log("we have received stop");
                connectedRobots[robot.robotId].status = { code: 1, running: false, message: "Programma apstādināta" };
                break;
            case "data":
                console.log("robot data: ", data);

                // paši svarīgākie dati kurus vienmēr vajag
                if (!data.elapsedTime) {
                    ws.send(JSON.stringify({ code: 400, type: "data", message: "Robota datos nav braukšanas laika" }));
                    return;
                }
                if (!data.fps || data.fps.length === 0) {
                    ws.send(JSON.stringify({ code: 400, type: "data", message: "Robota datos nav fps" }));
                    return;
                }

                // saglabājam datus
                const result = {};

                result.algorithm = data.algorithm;
                result.version = data.version;
                result.data = data.data;
                result.elapsedTime = data.elapsedTime.toFixed(2);
                result.fps = data.fps;

                const sum = data.fps.reduce((partialSum, a) => partialSum + a, 0);
                result.averageFps = Number((sum / data.fps.length).toFixed(1));

                result.robotId = robot.robotId;
                result.owner = robot.owner;
                result.sharedWith = [];
                result.createdAt = new Date().toLocaleString("lv");
                result.note = "";

                // mazāk svarīgie dai, kur var būt "tukši"
                if (!data.algorithm || data.algorithm === "") {
                    result.algorithm = "n/a";
                }
                if (!data.version || data.version === "") {
                    result.version = "n/a";
                }
                if (!data.data) {
                    result.data = [];
                }

                addDriveData(mongoClient, result).then((res) => {
                    if (!res) {
                        console.log("Robota dati netika veiksmīgi saglabāti");
                        return;
                    }

                    console.log("Robota dati veiksmīgi saglabāti");
                });

                ws.send(JSON.stringify({ code: 200, type: "data", message: "Robota dati saņemti" }));

                break;
            default:
                console.log("Nezināms ziņas tips: ", data.type);
                ws.send(JSON.stringify({ code: 400, type: "error", message: "Nezināms ziņas tips" }));
                break;
        }

        clientWs.send(JSON.stringify({
            type: "status",
            robotId: robot.robotId,
            status: connectedRobots[robot.robotId].status
        }));
    });
});