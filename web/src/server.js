import "dotenv/config";
import express from "express";
import cookieParser from "cookie-parser";
import { parse } from "url";
import favicon from "serve-favicon";
import { WebSocketServer } from "ws";
import path from "path";
import jwt from "jsonwebtoken";
import bcrypt from "bcrypt";
import crypto from "crypto";
import { MongoClient } from "mongodb";
import { getUser, getUsers, addUser, getDriveDataByOwner, addDriveData, removeUser, getRobotsByOwner, addRobot, removeRobot, updateRobotKey, getRobotByKey, getRobotById } from "./db.js";

const app = express();
const DATABASE_URI = process.env.DATABASE_URI;
const PORT = process.env.ENVIRONMENT === "prod" ? process.env.PROD_PORT : process.env.DEV_PORT;
const PRIVATE_KEY = process.env.PRIVATE_KEY;
const SALT_ROUNDS = 12;
const __dirname = path.resolve();
const mongoClient = new MongoClient(DATABASE_URI);
await mongoClient.connect();

app.use(express.json());
app.use(cookieParser());
app.use(express.static(path.join(__dirname, "pages")));
app.use(favicon(path.join(__dirname, "pages/favicon.ico")));

// check if root/default account exists
const admin = await getUser(mongoClient, process.env.ROOT_USERNAME);
if (!admin) {
    console.log("No default account found, creating new");
    const adminUser = {
        username: process.env.ROOT_USERNAME,
        password: await bcrypt.hash(process.env.ROOT_PASSWORD, SALT_ROUNDS),
        root: true,
        admin: true,
        createdAt: new Date().toLocaleString("lv")
    };
    await addUser(mongoClient, adminUser);
}

// auth
app.use((req, res, next) => {
    if (req.path === "/login/") {
        next();
        return;
    }
    if (req.path === `/api/driveData` && req.method === "POST") {
        next()
        return;
    }
    if (!req.cookies.token) {
        res.status(401).redirect("/login/");
        return;
    }

    try {
        const decoded = jwt.verify(req.cookies.token, PRIVATE_KEY);
        console.log(decoded);
        next();
    } catch (err) {
        res.status(401).redirect("/login/");
        console.error(err.message);
    }
});

app.get("/", (req, res) => {
    res.redirect("/data/");
})

app.get("/login/", (req, res) => {
    res.sendFile(path.join(__dirname, "pages/login/login.html"));
});

app.post("/login/", async (req, res) => {
    console.log(JSON.stringify(req.body));
    const user = {
        username: req.body.username,
        password: req.body.password
    };

    const userFromDb = await getUser(mongoClient, user.username);
    if (!userFromDb) {
        res.status(400).json({ message: "nepareizs lietotājvārds!" });
        return;
    }

    console.log(`${user.password} and ${userFromDb.password}`);
    bcrypt.compare(user.password, userFromDb.password, (err, result) => {
        if (err) {
            res.status(500).json({ message: "radās kļūda!" });
            console.error(err.message);
            return;
        }

        if (!result) {
            res.status(400).json({ message: "nepareiza parole!" });
            return;
        }

        const token = jwt.sign({ user: userFromDb.username }, PRIVATE_KEY, { expiresIn: "30m" });
        console.log(`setting new token for ${user.username}: ${token}`);
        // res.setHeader("Set-Cookie", `token=${token}`);
        res.status(200).cookie("token", token, { maxAge: new Date(Date.now() + 1800000), sameSite: "strict" }).json({ message: "success" });
    });
});

app.get("/data/", (req, res) => {
    res.sendFile(path.join(__dirname, "pages/data/data.html"));
});

app.get("/robots/", (req, res) => {
    res.sendFile(path.join(__dirname, "pages/robots/robots.html"));
});

app.get("/users/", (req, res) => {
    res.sendFile(path.join(__dirname, "pages/users/users.html"));
});

app.get("/logout", (req, res) => {
    // TODO: jwt blacklist for tokens that still work after logout
    res.clearCookie("token");
    res.redirect("/login/");
});

// api
app.get(`/api/driveData`, async (req, res) => {
    const currentUsername = jwt.verify(req.cookies.token, PRIVATE_KEY).user;
    const data = await getDriveDataByOwner(mongoClient, currentUsername);
    res.status(200).json({ data: data });
});

app.post(`/api/driveData`, async (req, res) => {
    const data = req.body;

    if (!data.elapsedTime || !data.data || data.data === []) {
        res.status(400).json({ message: "nepareizi vai trūkstoši dati!" });
        return;
    }

    data.createdAt = new Date().toLocaleString();
    console.log(data);
    const result = await addDriveData(mongoClient, data);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "dati pievienoti veiksmīgi!" });
});

app.get(`/api/users`, async (req, res) => {
    const data = await getUsers(mongoClient);
    data.forEach(item => {
        delete item.password;
        delete item.root;
    });
    res.status(200).json({ data: data });
});

app.get(`/api/user`, async (req, res) => {
    // const data = await getUsers(client);
    const data = {
        username: jwt.verify(req.cookies.token, PRIVATE_KEY).user
    };
    res.status(200).json(data);
});

app.post(`/api/user`, async (req, res) => {
    // TODO: check if user is admin
    const user = req.body;

    if (!user.username || !user.password) {
        res.status(400).json({ message: "nepareizs lietotājvārds vai parole!" });
        return;
    }

    const currentUsername = jwt.verify(req.cookies.token, PRIVATE_KEY).user;
    const currentUserFromDb = await getUser(mongoClient, currentUsername);
    if (currentUserFromDb.admin === false) {
        res.status(403).json({ message: "nevar izveidot kontu!" });
        return;
    }

    const userFromDb = await getUser(mongoClient, user.username);
    if (userFromDb) {
        res.status(400).json({ message: "lietotājvārds jau aizņemts!" });
        return;
    }

    if (!user.admin) {
        user.admin = false;
    }

    delete user.root;

    user.createdAt = new Date().toLocaleString();

    const hash = await bcrypt.hash(user.password, SALT_ROUNDS);
    user.password = hash;

    console.log(user);
    const result = await addUser(mongoClient, user);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "lietotājs veiksmīgi izveidots!" });
});

app.delete(`/api/user/:username`, async (req, res) => {
    const username = req.params.username;
    if (username === "") {
        res.status(400).json({ message: "netika norādīts lietotājvārds!" });
        return;
    }

    // TODO: change to environment variable or something
    if (username === "admin") {
        res.status(403).json({ message: "nevar noņemt admin kontu!" });
        return;
    }

    const userFromDb = await getUser(mongoClient, username);
    if (!userFromDb) {
        res.status(404).json({ message: "lieotājs netika atrasts!" });
        return;
    }

    const currentUsername = jwt.verify(req.cookies.token, PRIVATE_KEY).user;
    const currentUserFromDb = await getUser(mongoClient, currentUsername);
    if (userFromDb.admin === true && currentUserFromDb.admin === false) {
        res.status(403).json({ message: "nevar noņemt kontu!" });
        return;
    }

    const result = await removeUser(mongoClient, username);

    res.status(200).json({ message: "lietotājs noņemts" });
});

app.get(`/api/robots`, async (req, res) => {
    const currentUsername = jwt.verify(req.cookies.token, PRIVATE_KEY).user;
    const data = await getRobotsByOwner(mongoClient, currentUsername);
    res.status(200).json({ data: data });
});

app.post(`/api/robot`, async (req, res) => {
    const currentUsername = jwt.verify(req.cookies.token, PRIVATE_KEY).user;

    // TODO: return inserted robot
    const robot = {
        robotId: crypto.randomBytes(2).toString('hex'),
        key: crypto.randomBytes(8).toString('hex'),
        owner: currentUsername,
        createdAt: new Date().toLocaleString()
    };

    const result = await addRobot(mongoClient, robot);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "robots veiksmīgi izveidots!" });
});

app.delete(`/api/robot/:robotId`, async (req, res) => {
    const robotId = req.params.robotId;

    if (robotId === "") {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    const result = await removeRobot(mongoClient, robotId);
    if (result === 0) {
        res.status(404).json({ message: "robots netika atrasts!" });
        return;
    }

    res.status(200).json({ message: "robots izdzēsts!" });
});

app.post(`/api/robotToken/:robotId`, async (req, res) => {
    const robotId = req.params.robotId;

    if (robotId === "") {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    const result = await updateRobotKey(mongoClient, robotId, crypto.randomBytes(8).toString('hex'));
    if (result === 0) {
        res.status(404).json({ message: "robots netika atrasts!" });
        return;
    }

    res.status(200).json({ message: "jauna savienošanās atslēga izveidota" });
});

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

const server = app.listen(PORT, () => console.log(`server started at http://localhost:${PORT}`));

// handle websocket upgrade
server.on("upgrade", (req, socket, head) => {
    const { pathname } = parse(req.url);

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
});