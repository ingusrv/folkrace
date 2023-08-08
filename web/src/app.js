import "dotenv/config";
import express from "express";
import cookieParser from "cookie-parser";
import favicon from "serve-favicon";
import path from "path";

import authRouter from "../routes/auth.js";
import loginRouter from "../routes/login.js";
import dataRouter from "../routes/data.js";
import robotsRouter from "../routes/robots.js";
import usersRouter from "../routes/users.js";
import apiRouter from "../routes/api.js";
import websocketRouter from "../routes/websocket.js";

const app = express();
const PORT = process.env.ENVIRONMENT === "prod" ? process.env.PROD_PORT : process.env.DEV_PORT;

const __dirname = path.resolve();
import { mongoClient } from "./databaseClient.js";

app.use(express.json());
app.use(cookieParser());
app.use(express.static(path.join(__dirname, "public")));
app.use(favicon(path.join(__dirname, "public/favicon.ico")));

// setup default account
import setupDefaultUser from "./defaultUser.js";
setupDefaultUser(mongoClient, process.env.ROOT_USERNAME, process.env.ROOT_PASSWORD);

// auth
import verifyToken from "./verifyToken.js";
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

// start server
const server = app.listen(PORT, () => console.log(`server started at http://localhost:${PORT}`));

// handle websocket upgrade
server.on("upgrade", websocketRouter);