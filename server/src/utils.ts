import crypto from "crypto";

export function generateToken(length: number = 4): string {
    const randomValues = crypto.getRandomValues(new Uint32Array(length));
    const token = Array.from(randomValues, (num) => {
        return num.toString(16);
    }).join("");

    return token;
}
