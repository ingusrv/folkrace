import crypto from "crypto";

export default function generateTempPassword(length) {
    return crypto.randomBytes(length * 0.5).toString("hex");
}