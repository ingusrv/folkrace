import { TOKEN_SECRET } from "./config.js";
import jwt from "jsonwebtoken";

export default function verifyToken(req, res, next) {
    console.log("verifying token");

    if (req.path === "/login") {
        next();
        return;
    }

    if (req.path === "/api/auth/login") {
        next();
        return;
    }

    if (!req.cookies.token) {
        console.log("no token found, redirecting to /login");
        res.status(401).redirect("/login");
        return;
    }

    try {
        const decoded = jwt.verify(req.cookies.token, TOKEN_SECRET);
        console.log("token verified", decoded);
        next();
    } catch (err) {
        res.status(401).redirect("/login");
        console.error(err.message);
    }
}