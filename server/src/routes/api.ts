import { PASSWORD_SALT_ROUNDS } from "../config.js";
import express from "express";
import bcrypt from "bcrypt";
import authorize from "../auth.js";
import { getMongoInstance } from "../database.js";
import {
    getDriveDataByOwner, removeDriveData,
    getRobotsByOwner, addRobot, removeRobot, setRobotToken,
    getAllUsers, getUserByUsername, getUser, addUser, removeUser
} from "../databaseMethods.js";
import { Role } from "../types.js";
import { ObjectId } from "mongodb";
import { generateToken } from "../utils.js";
const router = express.Router();

router.get(`/driveData`, authorize, async (req, res) => {
    console.log(`GET /driveData no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoClient = getMongoInstance();
    const driveData = await getDriveDataByOwner(mongoClient, res.locals.user._id);
    res.status(200).json(driveData);
});

router.delete(`/driveData/:id`, authorize, async (req, res) => {
    const id = req.params.id;
    if (!id) {
        res.status(400).json({ message: "netika norādīts id!" });
        return;
    }

    console.log(`DELETE /driveData no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoclient = getMongoInstance();
    const success = await removeDriveData(mongoclient, res.locals.user._id, new ObjectId(id));

    if (!success) {
        res.status(500).json({ message: "nevarēja izdēst datus" });
        return;
    }

    res.status(200).json({ message: "dati izdzēsti!" });

});

router.get(`/robots`, authorize, async (req, res) => {
    console.log(`GET /robots no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoClient = getMongoInstance();
    const robots = await getRobotsByOwner(mongoClient, res.locals.user._id);
    res.status(200).json(robots);
});

router.post(`/robot`, authorize, async (req, res) => {
    if (!req.body.name) {
        res.status(400).json({ message: "nederīgs robota nosaukums" });
        return;
    }


    console.log(`POST /robot no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoClient = getMongoInstance();
    const robot = await addRobot(mongoClient, {
        ownerId: res.locals.user._id,
        name: req.body.name,
        token: generateToken()
    });

    if (!robot) {
        res.status(500).json({ message: "nevarēja izveidot robotu" });
        return;
    }

    res.status(201).json(robot);
});

router.delete(`/robot/:id`, authorize, async (req, res) => {
    const id = req.params.id;
    if (!id) {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    console.log(`DELETE /robot no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoClient = getMongoInstance();
    const success = await removeRobot(mongoClient, res.locals.user._id, new ObjectId(id));

    if (!success) {
        res.status(500).json({ message: "nevarēja izdēst robotu" });
        return;
    }

    res.status(200).json({ message: "robots izdzēsts!" });
});

router.get(`/robot/token/:id`, authorize, async (req, res) => {
    const id = req.params.id;
    if (!id) {
        res.status(400).json({ message: "netika norādīts robota id!" });
        return;
    }

    console.log(`GET /robot/token no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoClient = getMongoInstance();
    const token = generateToken();
    const success = await setRobotToken(mongoClient, res.locals.user._id, new ObjectId(id), token);
    if (!success) {
        res.status(500).json({ message: "neizdevās izveidot jaunu token" });
        return;
    }

    res.status(200).json({ token: token });
});

router.get(`/users`, authorize, async (req, res) => {
    if (res.locals.user.role !== Role.admin && res.locals.user.role !== Role.default) {
        res.status(403).json({ message: "lietotājs nav autorizēts lai skatītu lietotājus" });
        return;
    }

    console.log(`GET /users no ${req.res.locals.user.username} (${res.locals.user._id})`);
    const mongoClient = getMongoInstance();
    const users = await getAllUsers(mongoClient);
    // nesūtam paroli un token
    users.forEach((user) => {
        user.password = "";
        user.token = "";
    });
    res.status(200).json(users);
});

router.post(`/user`, authorize, async (req, res) => {
    console.log(`POST /user no ${req.res.locals.user.username} (${res.locals.user._id})`);
    if (res.locals.user.role !== Role.admin && res.locals.user.role !== Role.default) {
        res.status(403).json({ message: "nevar izveidot kontu!" });
        return;
    }

    const user = req.body;
    if (!user.username || !user.password) {
        res.status(400).json({ message: "nepareizs lietotājvārds vai parole!" });
        return;
    }

    const mongoClient = getMongoInstance();
    const existingUser = await getUserByUsername(mongoClient, user.username);
    if (existingUser) {
        res.status(400).json({ message: "lietotājvārds jau aizņemts!" });
        return;
    }

    const createdUser = await addUser(mongoClient, {
        username: user.username,
        password: await bcrypt.hash(user.password, PASSWORD_SALT_ROUNDS),
        role: user.admin ? Role.admin : Role.user
    });

    if (!createdUser) {
        res.status(500).json({ message: "could not create user" });
        return;
    }

    // nesūtam paroli un token lietotājam 
    createdUser.password = "";
    createdUser.token = "";
    res.status(201).json(createdUser);
});

router.delete(`/user/:id`, authorize, async (req, res) => {
    const id = req.params.id;
    if (!id) {
        res.status(400).json({ message: "nepareizs id" });
        return;
    }

    console.log(`DELETE /user no ${req.res.locals.user.username} (${res.locals.user._id})`);
    if (res.locals.user.role !== Role.admin && res.locals.user.role !== Role.default) {
        res.status(403).json({ message: "lietotājs nav autorizēts lai dzēstu kontus" });
        return;
    }

    const mongoClient = getMongoInstance();
    const user = await getUser(mongoClient, new ObjectId(id));
    if (!user) {
        res.status(404).json({ message: "lieotājs netika atrasts!" });
        return;
    }

    if (user.role === Role.default) {
        res.status(403).json({ message: "nevar izdēst noklusējuma kontu!" });
        return;
    }

    const result = await removeUser(mongoClient, user._id);
    if (result) {
        res.status(200).json({ message: "lietotājs izdēsts" });
    } else {
        res.status(500).json({ message: "neizdevās izdzēst lietotāju" });
    }
});

export default router;
