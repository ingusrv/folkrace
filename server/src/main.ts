import "dotenv/config";
import { SERVER_PORT, MONGO_URI } from "./config.js";
import express from "express";
import { initMongoDb } from "./database.js";
import setupDefaultUser from "./defaultUser.js";

import authRouter from "./routes/authentication.js";
import apiRouter from "./routes/api.js";
import websocketRouter from "./routes/websocket.js";

const app = express();

app.use(express.json());

app.use("/auth", authRouter);
app.use("/", apiRouter);

initMongoDb(MONGO_URI).then((mongoClient) => {
    setupDefaultUser(mongoClient);
    const server = app.listen(SERVER_PORT, () => console.log("Serveris darbojas portā", SERVER_PORT));
    server.on("upgrade", websocketRouter);
}).catch((err) => {
    console.error(err);
});
