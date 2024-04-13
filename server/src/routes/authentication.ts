import express, { Request, Response } from "express";
import bcrypt from "bcrypt";
import { getUserByUsername, setToken } from "../databaseMethods.js";
import { getMongoInstance } from "../database.js";
import { generateToken } from "../utils.js";
import authorize from "../auth.js";
const router = express.Router();

router.post("/login", async (req, res) => {
    if (!req.body.username || !req.body.password) {
        res.status(400).json({ message: "nederīgs lietotājvārds vai parole" });
        return;
    }

    const mongoClient = getMongoInstance();
    const user = await getUserByUsername(mongoClient, req.body.username);
    if (!user) {
        res.status(400).json({ message: "nepareizs lietotājvārds" });
        return;
    }

    bcrypt.compare(req.body.password, user.password, async (err, result) => {
        if (err) {
            res.status(500).json({ message: "radās kļūda pārbaudot paroli" });
            console.error("kļūda pārbaudot paroli:", err.message);
            return;
        }

        if (!result) {
            res.status(400).json({ message: "nepareiza parole" });
            return;
        }

        const token = generateToken();
        const success = await setToken(mongoClient, user._id, token);
        if (success) {
            console.log(`${user.username} (${user._id}) pieteicās`);
            user.token = token;
            user.password = "";
            res.status(200).json(user);
        } else {
            console.error(`neizdevās autentificēt ${user.username} (${user._id})`);
            res.status(500).json({ message: "neizdevās autentificēt lietotāju" });
        }
    });
});

router.get("/logout", authorize, async (req: Request, res: Response) => {
    console.log(req.url);
    const user = res.locals.user;
    const mongoClient = getMongoInstance();
    const success = await setToken(mongoClient, user._id, null);
    if (success) {
        console.log(`${user.username} (${user.token} atteicās`);
        res.status(200).json({ message: "lietotājs atteikts" });
    } else {
        console.error(`neizdevās atteikt ${user.username} (${user._id})`);
        res.status(500).json({ message: "radās kļūda atteicot lietotāju" });
    }
});

export default router;
