import bcrypt from "bcrypt";
import { getUser, addUser } from "./db.js";

export default async function setupDefaultUser(mongoClient, username, password) {
    const admin = await getUser(mongoClient, process.env.ROOT_USERNAME);
    if (!admin) {
        console.log("No default account found, creating new");
        const adminUser = {
            username: username,
            password: await bcrypt.hash(password, SALT_ROUNDS),
            root: true,
            admin: true,
            createdAt: new Date().toLocaleString("lv")
        };
        await addUser(mongoClient, adminUser);
    }
}