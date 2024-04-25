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
        console.log(`lietotājs no ${req.headers.host} centās pieslēgties bez atslēgas`);
        return;
    }

    const mongoClient = getMongoInstance();
    const user = await getUserByToken(mongoClient, token);
    if (!user) {
        res.status(401).json({ message: "nederīga autorizācijas atslēga" });
        console.log(`neizdevās autorizēt lietotāju no ${req.headers.host} atslēga: ${token}`);
        return;
    }

    res.locals.user = user;
    next();
}
