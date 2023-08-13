import assert from "assert";

export const SALT_ROUNDS = 12;

export const PORT = process.env.PORT || 3000;
export const MONGO_URI = process.env.MONGO_URI;
assert.ok(MONGO_URI, "Database URI is not defined!");
export const TOKEN_SECRET = process.env.TOKEN_SECRET;
assert.ok(TOKEN_SECRET, "Token secret is not defined!");
export const ROOT_USERNAME = process.env.ROOT_USERNAME;
export const ROOT_PASSWORD = process.env.ROOT_PASSWORD;