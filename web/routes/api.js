import { TOKEN_SECRET, SALT_ROUNDS } from "../src/config.js";
import express from "express";
import jwt from "jsonwebtoken";
import bcrypt from "bcrypt";
import crypto from "crypto";
import { getMongoInstance } from "../src/database.js";
import { getUser, getUsers, addUser, getDriveDataByOwner, removeUser, getRobotsByOwner, addRobot, removeRobot, updateRobotKey } from "../src/databaseMethods.js";
const router = express.Router();

router.get(`/driveData`, async (req, res) => {
    const currentUsername = jwt.verify(req.cookies.token, TOKEN_SECRET).user;
    const mongoClient = getMongoInstance();
    const data = await getDriveDataByOwner(mongoClient, currentUsername);
    res.status(200).json({ data: data });
});

router.get(`/users`, async (req, res) => {
    const mongoClient = getMongoInstance();
    const data = await getUsers(mongoClient);
    data.forEach(item => {
        delete item.password;
        delete item.root;
    });
    res.status(200).json({ data: data });
});

router.get(`/user`, async (req, res) => {
    // const data = await getUsers(client);
    const data = {
        username: jwt.verify(req.cookies.token, TOKEN_SECRET).user
    };
    res.status(200).json(data);
});

router.post(`/user`, async (req, res) => {
    // TODO: check if user is admin
    const user = req.body;

    if (!user.username || !user.password) {
        res.status(400).json({ message: "nepareizs lietotājvārds vai parole!" });
        return;
    }

    const currentUsername = jwt.verify(req.cookies.token, TOKEN_SECRET).user;
    const mongoClient = getMongoInstance();
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

router.delete(`/user/:username`, async (req, res) => {
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

    const mongoClient = getMongoInstance();
    const userFromDb = await getUser(mongoClient, username);
    if (!userFromDb) {
        res.status(404).json({ message: "lieotājs netika atrasts!" });
        return;
    }

    const currentUsername = jwt.verify(req.cookies.token, TOKEN_SECRET).user;
    const currentUserFromDb = await getUser(mongoClient, currentUsername);
    if (userFromDb.admin === true && currentUserFromDb.admin === false) {
        res.status(403).json({ message: "nevar noņemt kontu!" });
        return;
    }

    const result = await removeUser(mongoClient, username);

    res.status(200).json({ message: "lietotājs noņemts" });
});

router.get(`/robots`, async (req, res) => {
    const currentUsername = jwt.verify(req.cookies.token, TOKEN_SECRET).user;
    const mongoClient = getMongoInstance();
    const data = await getRobotsByOwner(mongoClient, currentUsername);
    res.status(200).json({ data: data });
});

router.post(`/robot`, async (req, res) => {
    const currentUsername = jwt.verify(req.cookies.token, TOKEN_SECRET).user;

    // TODO: return inserted robot
    const robot = {
        robotId: crypto.randomBytes(2).toString('hex'),
        key: crypto.randomBytes(8).toString('hex'),
        owner: currentUsername,
        createdAt: new Date().toLocaleString()
    };

    const mongoClient = getMongoInstance();
    const result = await addRobot(mongoClient, robot);

    if (!result) {
        res.status(500).json({ message: "radās kļūda!" });
        return;
    }

    res.status(201).json({ message: "robots veiksmīgi izveidots!" });
});

router.delete(`/robot/:robotId`, async (req, res) => {
    const robotId = req.params.robotId;

    if (robotId === "") {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    const mongoClient = getMongoInstance();
    const result = await removeRobot(mongoClient, robotId);
    if (result === 0) {
        res.status(404).json({ message: "robots netika atrasts!" });
        return;
    }

    res.status(200).json({ message: "robots izdzēsts!" });
});

router.post(`/robotToken/:robotId`, async (req, res) => {
    const robotId = req.params.robotId;

    if (robotId === "") {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    const mongoClient = getMongoInstance();
    const result = await updateRobotKey(mongoClient, robotId, crypto.randomBytes(8).toString('hex'));
    if (result === 0) {
        res.status(404).json({ message: "robots netika atrasts!" });
        return;
    }

    res.status(200).json({ message: "jauna savienošanās atslēga izveidota" });
});

export default router;