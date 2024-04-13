import { NextFunction, Request, Response } from "express";
import { getUserByToken } from "./databaseMethods.js";
import { getMongoInstance } from "./database.js";

export default async function authorize(req: Request, res: Response, next: NextFunction) {
    const header = req.header("authorization");

    if (!header) {
        res.status(401).json({ message: "nav autorizācijas header" });
        return;
    }

    const token = header?.split(" ")[1];

    if (!token) {
        res.status(401).json({ message: "nav autorizācijas atslēga" });
        return;
    }

    const mongoClient = getMongoInstance();
    const user = await getUserByToken(mongoClient, token);
    if (!user) {
        res.status(401).json({ message: "nederīga autorizācijas atslēga" });
        return;
    }

    console.log(`${user.username} (${user._id}) autorizēts`);
    res.locals.user = user;
    next();
}
