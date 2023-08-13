import express from "express";
import bcrypt from "bcrypt";
import jwt from "jsonwebtoken";
import { getUser } from "../src/databaseMethods.js";
import { getMongoInstance } from "../src/database.js";
const router = express.Router();

router.post("/login", async (req, res) => {
    console.log(JSON.stringify(req.body));
    const user = {
        username: req.body.username,
        password: req.body.password
    };

    const mongoClient = getMongoInstance();
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

        const token = jwt.sign({ user: userFromDb.username }, process.env.PRIVATE_KEY, { expiresIn: "30m" });
        console.log(`setting new token for ${user.username}: ${token}`);
        // res.setHeader("Set-Cookie", `token=${token}`);
        res.status(200).cookie("token", token, { maxAge: new Date(Date.now() + 1800000), sameSite: "strict" }).json({ message: "success" });
    });
});

router.get("/logout", (req, res) => {
    // TODO: jwt blacklist for tokens that still work after logout
    res.clearCookie("token");
    res.redirect("/login");
});

export default router;