import assert from "assert";

export const PASSWORD_SALT_ROUNDS = 14;
export const SERVER_PORT = process.env.PORT || 3000;
assert.ok(process.env.MONGO_URI, "MONGO_URI nav definēts!");
export const MONGO_URI = process.env.MONGO_URI;
export const DEFAULT_USERNAME = process.env.DEFAULT_USERNAME || null;
export const DEFAULT_PASSWORD = process.env.DEFAULT_PASSWORD || null;
