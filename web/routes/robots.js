import express from "express";
import path from "path";
const router = express.Router();
const __dirname = path.resolve();

router.get("/", (req, res) => {
    res.sendFile(path.join(__dirname, "pages/robots.html"));
});

export default router;