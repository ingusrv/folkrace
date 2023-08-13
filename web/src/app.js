import "dotenv/config";
import { PORT, MONGO_URI, ROOT_USERNAME, ROOT_PASSWORD } from "./config.js";
import express from "express";
import cookieParser from "cookie-parser";
import favicon from "serve-favicon";
import path from "path";
import verifyToken from "./verifyToken.js";
import { initMongoDb } from "./database.js";
import setupDefaultUser from "./defaultUser.js";

import authRouter from "../routes/auth.js";
import loginRouter from "../routes/login.js";
import dataRouter from "../routes/data.js";
import robotsRouter from "../routes/robots.js";
import usersRouter from "../routes/users.js";
import apiRouter from "../routes/api.js";
import websocketRouter from "../routes/websocket.js";

const app = express();
const __dirname = path.resolve();

app.use(express.json());
app.use(cookieParser());
app.use(express.static(path.join(__dirname, "public")));
app.use(favicon(path.join(__dirname, "public/favicon.ico")));

// auth
app.use(verifyToken);

// auth routes
app.use("/api/auth", authRouter);

// frontend routes
app.get("/", (req, res) => {
    res.redirect("/data");
});
app.use("/login", loginRouter);
app.use("/data", dataRouter);
app.use("/robots", robotsRouter);
app.use("/users", usersRouter);

// api routes
app.use("/api", apiRouter);

initMongoDb(MONGO_URI).then((mongoClient) => {
    setupDefaultUser(mongoClient, ROOT_USERNAME, ROOT_PASSWORD);
    const server = app.listen(PORT, () => console.log(`Serveris darbojas portā ${PORT}`));
    server.on("upgrade", websocketRouter);
}).catch((err) => {
    console.error(err)
});