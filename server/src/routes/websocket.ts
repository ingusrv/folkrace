import { getUserByToken, addDriveData, getRobotByToken } from "../databaseMethods.js";
import { WebSocket, WebSocketServer } from "ws";
import { getMongoInstance } from "../database.js";
import { IncomingMessage } from "http";
import { Duplex } from "stream";
import { DriveData, MessageType, Robot, RobotState } from "../types.js";
import { ObjectId } from "mongodb";

type ConnectedRobot = {
    ws: WebSocket;
    robot: Robot;
    state: RobotState;
}

const userWss = new WebSocketServer({ noServer: true });
const robotWss = new WebSocketServer({ noServer: true });
const connectedUsers = new Map<string, WebSocket>();
const connectedRobots = new Map<string, ConnectedRobot>();

userWss.on("connection", async (ws: WebSocket, req: IncomingMessage) => {
    if (!req.url) {
        console.log("nav url");
        ws.close();
        return;
    }

    const url = new URL(req.url, `ws://${req.headers.origin}`);
    const token = url.searchParams.get("token");
    if (!token) {
        console.log("nav token");
        ws.close();
        return;
    }

    const mongoClient = getMongoInstance();
    const user = await getUserByToken(mongoClient, token);
    if (!user) {
        console.log("nav lietotājs");
        ws.close();
        return;
    }
    connectedUsers.set(user._id.toString(), ws);

    console.log(`lietotājs ${user.username} (${user._id}) pieslēdzās pie websocket`);
    ws.send("test");

    ws.on("message", async (e) => {
        const data = JSON.parse(e.toString("utf8"));
        const robot = connectedRobots.get(data.robotId.toString());

        switch (data.type) {
            case MessageType.start:
                if (!robot) {
                    ws.send(JSON.stringify({
                        type: MessageType.error,
                        message: "robots netika atrasts/nav savienots"
                    }));
                    return;
                }

                if (!data.delay) {
                    data.delay = 0;
                }

                robot.ws.send(JSON.stringify({
                    type: MessageType.start,
                    delay: data.delay
                }));
                break;
            case MessageType.stop:
                if (!robot) {
                    ws.send(JSON.stringify({
                        type: MessageType.error,
                        message: "robots netika atrasts/nav savienots"
                    }));
                    return;
                }

                robot.ws.send(JSON.stringify({
                    type: MessageType.stop,
                }));
                break;
            case MessageType.status:
                if (!robot) {
                    ws.send(JSON.stringify({
                        type: MessageType.status,
                        robotId: data.robotId,
                        state: RobotState.disconnected
                    }));
                    break;
                }
                ws.send(JSON.stringify({
                    type: MessageType.status,
                    robotId: robot.robot._id.toString(),
                    state: robot.state
                }));
                break;
            default:
                console.log("Nezināms ziņas tips: ", data.type);
                ws.send(JSON.stringify({
                    type: MessageType.error,
                    message: "Nezināms ziņas tips"
                }));
                break;
        }
    });

    ws.on("close", (e) => {
        connectedUsers.delete(user._id.toString());
        console.log(`${user.username} (${user._id}) atslēdzās no websocket`);
    });
});

robotWss.on("connection", async (ws: WebSocket, req: IncomingMessage) => {
    if (!req.url) {
        ws.close();
        return;
    }

    const url = new URL(req.url, `ws://${req.headers.origin}`);
    const token = url.searchParams.get("token");
    if (!token) {
        ws.close();
        return;
    }

    const mongoClient = getMongoInstance();
    const robot = await getRobotByToken(mongoClient, token);
    if (!robot) {
        ws.close();
        return;
    }

    connectedRobots.set(robot._id.toString(), {
        ws: ws,
        robot: robot,
        state: RobotState.connected
    });

    console.log(`robots ${robot.name} (${robot._id}) pieslēdzās pie websocket`);
    const userWs = connectedUsers.get(robot.owner.toString());
    if (userWs) {
        userWs.send(JSON.stringify({
            type: MessageType.status,
            robotId: robot._id.toString(),
            state: RobotState.connected
        }));
    }

    ws.on("message", async (e) => {
        const data = JSON.parse(e.toString("utf8"));
        console.log(`robots ${robot.name} (${robot._id}) atsūtīja:`, data);

        const userWs = connectedUsers.get(robot.owner.toString());

        switch (data.type) {
            case MessageType.start:
                connectedRobots.set(robot._id.toString(), {
                    ws: ws,
                    robot: robot,
                    state: RobotState.running
                });

                if (userWs) {
                    userWs.send(JSON.stringify({
                        type: MessageType.status,
                        robotId: robot._id.toString(),
                        state: RobotState.running
                    }));
                }
                break;
            case MessageType.stop:
                connectedRobots.set(robot._id.toString(), {
                    ws: ws,
                    robot: robot,
                    state: RobotState.connected
                });

                if (userWs) {
                    userWs.send(JSON.stringify({
                        type: MessageType.status,
                        robotId: robot._id.toString(),
                        state: RobotState.connected
                    }));
                }
                break;
            case MessageType.data:
                // saglabājam datus
                const result: DriveData = {
                    _id: new ObjectId(),
                    owner: robot.owner,
                    robot: robot._id,
                    name: "",
                    algorithm: data.algorithm || "",
                    version: data.version || "",
                    elapsedTime: Number(data.elapsedTime) || 0,
                    data: data.data || [],
                    fps: data.fps || [],
                    averageFps: data.fps.length > 0 ? Number((data.fps.reduce((sum: number, num: number) => sum + num, 0) / data.fps.length).toFixed(1)) : 0,
                    createdAt: new Date(),
                }

                addDriveData(mongoClient, result).then((res) => {
                    if (!res) {
                        console.log("notika kļūda saglabājot robota datus");
                        if (userWs) {
                            userWs.send(JSON.stringify({
                                type: MessageType.error,
                                message: "neizdevās saglabāt robota datus"
                            }));
                        }
                        return;
                    }

                    console.log("robota dati saglabāti");
                });

                ws.send(JSON.stringify({ type: MessageType.data, message: "Robota dati saņemti" }));

                break;
            default:
                console.log("Nezināms ziņas tips: ", data.type);
                ws.send(JSON.stringify({
                    type: MessageType.error,
                    message: "Nezināms ziņas tips"
                }));
                break;
        }
    });

    ws.on("close", (e) => {
        connectedRobots.delete(robot._id.toString());
        console.log(`robots ${robot.name} (${robot._id}) atslēdzās no websocket`);

        const userWs = connectedUsers.get(robot.owner.toString());
        if (userWs) {
            userWs.send(JSON.stringify({
                type: MessageType.status,
                robotId: robot._id.toString(),
                state: RobotState.disconnected
            }));
        }
    });
});

export default function websocketRouter(request: IncomingMessage, socket: Duplex, head: Buffer) {
    if (!request.url) {
        socket.destroy();
        return;
    }

    const { pathname } = new URL(request.url, `ws://${request.headers.host}`);
    switch (pathname) {
        case "/user":
            userWss.handleUpgrade(request, socket, head, (ws, req) => {
                userWss.emit("connection", ws, req);
            });
            break;
        case "/robot":
            robotWss.handleUpgrade(request, socket, head, (ws, req) => {
                robotWss.emit("connection", ws, req);
            });
            break;
        default:
            socket.destroy();
            break;
    }
}

