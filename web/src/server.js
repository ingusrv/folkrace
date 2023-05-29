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
import { getUser, getUsers, addUser, getDriveData, addDriveData, removeUser, getRobots, addRobot, removeRobot, updateRobotKey, getRobotByKey, getRobotById, updateRobotStatus } from "./db.js";

const app = express();
const uri = "mongodb://127.0.0.1:27017";
const client = new MongoClient(uri);
const port = 3000;
const api = "/api/v1";
const __dirname = path.resolve();
const privateKey = "thisIsMySecurePrivateKey";
const saltRounds = 10;

await client.connect();

app.use(express.json());
app.use(cookieParser());
app.use(express.static(path.join(__dirname, "pages")));
app.use(favicon(path.join(__dirname, "pages/favicon.ico")));

// check if root/default account exists
const admin = await getUser(client, "admin");
if (!admin) {
    console.log("No default account found, creating new");
    const admin = {
        username: "admin",
        password: await bcrypt.hash("admin", saltRounds),
        root: true,
        admin: true,
        createdAt: new Date().toLocaleString()
    };
    await addUser(client, admin);
}

// auth
app.use((req, res, next) => {
    if (req.path === "/login/") {
        next();
        return;
    }
    if (req.path === `${api}/driveData` && req.method === "POST") {
        next()
        return;
    }
    if (!req.cookies.token) {
        res.status(401).redirect("/login/");
        return;
    }

    try {
        const decoded = jwt.verify(req.cookies.token, privateKey);
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

    const userFromDb = await getUser(client, user.username);
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

        const token = jwt.sign({ user: userFromDb.username }, privateKey, { expiresIn: "30m" });
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
app.get(`${api}/driveData`, async (req, res) => {
    const data = await getDriveData(client);
    res.status(200).json({ data: data });
});

app.post(`${api}/driveData`, async (req, res) => {
    const data = req.body;

    if (!data.elapsedTime || !data.data || data.data === []) {
        res.status(400).json({ message: "nepareizi vai trūkstoši dati!" });
        return;
    }

    data.createdAt = new Date().toLocaleString();
    console.log(data);
    const result = await addDriveData(client, data);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "dati pievienoti veiksmīgi!" });
});

app.get(`${api}/users`, async (req, res) => {
    const data = await getUsers(client);
    data.forEach(item => {
        delete item.password;
        delete item.root;
    });
    res.status(200).json({ data: data });
});

app.get(`${api}/user`, async (req, res) => {
    // const data = await getUsers(client);
    const data = {
        username: jwt.verify(req.cookies.token, privateKey).user
    };
    res.status(200).json(data);
});

app.post(`${api}/user`, async (req, res) => {
    // TODO: check if user is admin
    const user = req.body;

    if (!user.username || !user.password) {
        res.status(400).json({ message: "nepareizs lietotājvārds vai parole!" });
        return;
    }

    const currentUsername = jwt.verify(req.cookies.token, privateKey).user;
    const currentUserFromDb = await getUser(client, currentUsername);
    if (currentUserFromDb.admin === false) {
        res.status(403).json({ message: "nevar izveidot kontu!" });
        return;
    }

    const userFromDb = await getUser(client, user.username);
    if (userFromDb) {
        res.status(400).json({ message: "lietotājvārds jau aizņemts!" });
        return;
    }

    if (!user.admin) {
        user.admin = false;
    }

    delete user.root;

    user.createdAt = new Date().toLocaleString();

    const hash = await bcrypt.hash(user.password, saltRounds);
    user.password = hash;

    console.log(user);
    const result = await addUser(client, user);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "lietotājs veiksmīgi izveidots!" });
});

app.delete(`${api}/user/:username`, async (req, res) => {
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

    const userFromDb = await getUser(client, username);
    if (!userFromDb) {
        res.status(404).json({ message: "lieotājs netika atrasts!" });
        return;
    }

    const currentUsername = jwt.verify(req.cookies.token, privateKey).user;
    const currentUserFromDb = await getUser(client, currentUsername);
    if (userFromDb.admin === true && currentUserFromDb.admin === false) {
        res.status(403).json({ message: "nevar noņemt kontu!" });
        return;
    }

    const result = await removeUser(client, username);

    res.status(200).json({ message: "lietotājs noņemts" });
});

app.get(`${api}/robots`, async (req, res) => {
    const data = await getRobots(client);
    res.status(200).json({ data: data });
});

app.post(`${api}/robot`, async (req, res) => {
    // TODO: return inserted robot
    const robot = {
        robotId: crypto.randomBytes(2).toString('hex'),
        key: crypto.randomBytes(8).toString('hex'),
        status: "Bezsaistē",
        lastUpdated: new Date().toLocaleString(),
        createdAt: new Date().toLocaleString()
    };

    const result = await addRobot(client, robot);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "robots veiksmīgi izveidots!" });
});

app.delete(`${api}/robot/:robotId`, async (req, res) => {
    const robotId = req.params.robotId;

    if (robotId === "") {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    const result = await removeRobot(client, robotId);
    if (result === 0) {
        res.status(404).json({ message: "robots netika atrasts!" });
        return;
    }

    res.status(200).json({ message: "robots izdzēsts!" });
});

app.post(`${api}/robotToken/:robotId`, async (req, res) => {
    const robotId = req.params.robotId;

    if (robotId === "") {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    const result = await updateRobotKey(client, robotId, crypto.randomBytes(8).toString('hex'));
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

        const robot = await getRobotById(client, data.robotId);

        if (!robot) {
            ws.send(JSON.stringify({ code: 404, type: "error", message: `Robots "${data.robotId}" netika atrasts` }));
            return;
        }

        const robotWs = connectedRobots[robot.robotId];

        switch (data.type) {
            case "connect":
                ws.send(JSON.stringify({
                    code: 200,
                    origin: "server",
                    type: "connect",
                    robotId: robot.robotId,
                    status: robot.status,
                    lastUpdated: robot.lastUpdated,
                    message: "Savienots"
                }));

                connectedClients[robot.robotId] = ws;
                break;
            case "start":
                if (!robotWs) {
                    ws.send(JSON.stringify({ code: 500, type: "error", message: `Robots "${robot.robotId}" nav savienots` }));
                    return;
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
    });

    ws.on("message", async (e) => {
        const data = JSON.parse(e);
        console.log("robot: ", data);

        if (!data.key || !data.type) {
            ws.send(JSON.stringify({ code: 400, type: "error", message: "Notika kļūda" }));
            return;
        }

        const robot = await getRobotByKey(client, data.key);

        if (!robot) {
            ws.send(JSON.stringify({ code: 404, type: "error", message: "Robots ar tādu atslēgu netika atrasts" }));
            return;
        }

        const clientWs = connectedClients[robot.robotId];

        switch (data.type) {
            case "connect":
                console.log(robot);
                connectedRobots[robot.robotId] = ws;
                ws.send(JSON.stringify({ code: 200, type: "connect", robotId: `${robot.robotId}`, message: "Robots savienots" }))
                updateRobotStatus(client, robot.robotId, "Savienots");

                if (clientWs) {
                    console.log("connect: we have client");
                    clientWs.send(JSON.stringify({ code: 200, origin: "robot", type: "connect", message: "Robots savienots" }));
                }

                break;
            case "start":
                console.log("we have received start");
                updateRobotStatus(client, robot.robotId, "Programma startēta");

                if (clientWs) {
                    console.log("start: we have client");
                    clientWs.send(JSON.stringify({ code: 200, type: "start", message: "Programma startēta" }));
                }

                break;
            case "stop":
                console.log("we have received stop");
                updateRobotStatus(client, robot.robotId, "Programma apstādināta");

                if (clientWs) {
                    console.log("stop: we have client");
                    clientWs.send(JSON.stringify({ code: 200, type: "stop", message: "Programma apstādināta" }));
                }

                break;
            default:
                ws.send(JSON.stringify({ code: 400, type: "error", message: "Nezināms ziņas tips" }));
                break;
        }
    });
});

const server = app.listen(port, () => console.log(`server started at http://localhost:${port}`));

// handle websocket upgrade
server.on("upgrade", (req, socket, head) => {
    const { pathname } = parse(req.url);

    if (pathname === `${api}/panel`) {
        // TODO: auth
        robotPanelWss.handleUpgrade(req, socket, head, (ws) => {
            robotPanelWss.emit("connection", ws, req);
        });
    } else if (pathname === `${api}/robot`) {
        robotControlWss.handleUpgrade(req, socket, head, (ws) => {
            robotControlWss.emit("connection", ws, req);
        });
    } else {
        socket.destroy();
    }
});