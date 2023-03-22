import express from "express";
import cookieParser from "cookie-parser";
import favicon from "serve-favicon";
import expressWs from "express-ws";
import path from "path";
import jwt from "jsonwebtoken";
import bcrypt from "bcrypt";
import crypto from "crypto";
import { MongoClient } from "mongodb";
import { getUser, getUsers, addUser, getDriveData, addDriveData, removeUser, getRobots, addRobot, removeRobot, updateRobotKey } from "./db.js";

const app = express();
expressWs(app);
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

// app.websocket("/api/v1/robot/:id") {}
app.ws(`${api}/panel`, (ws, req) => {
    ws.on("close", (e) => {
        console.log(`client connection stopped with code ${e}`);
    });
    ws.on("message", (e) => {
        const data = JSON.parse(e);
        console.log(data);
        // TODO: check if robotId exists
        if (data.type === "connect") {
            ws.send(JSON.stringify({ code: 200, type: "connect", message: "Savienots" }));
        }
        if (data.type === "start") {
            // start robot
            ws.send(JSON.stringify({ code: 200, type: "started", message: "Sākta programmas izpilde" }));
        }
        if (data.type === "stop") {
            // stop robot
            ws.send(JSON.stringify({ code: 200, type: "stopped", message: "Apstādināta programmas izpilde" }));
        }
    });
});

app.listen(port, () => console.log(`server started at http://localhost:${port}`));