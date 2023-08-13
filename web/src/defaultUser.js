import bcrypt from "bcrypt";
import generateTempPassword from "./generateTempPassword.js";
import { getRootUser, addUser, updateUserUsername, updateUserPassword } from "./databaseMethods.js";

const SALT_ROUNDS = 12;
const rootUserTemplate = {
    username: undefined,
    password: undefined,
    root: true,
    admin: true,
    createdAt: undefined
}

export default async function setupDefaultUser(mongoClient, username, password) {
    const rootUser = await getRootUser(mongoClient);

    if (rootUser && !username && !password) {
        return;
    }

    if (!rootUser && username && password) {
        console.info("Noklusējuma lietotājs netika atrasts! Tiks izveidots jauns lietotājs no .env faila vērtībām...");
        const newRootUser = rootUserTemplate;
        newRootUser.username = username;
        newRootUser.password = await bcrypt.hash(password, SALT_ROUNDS);
        newRootUser.createdAt = new Date().toLocaleString("lv");

        await addUser(mongoClient, newRootUser);
        return;
    }

    if (!rootUser && !username && !password) {
        console.info("Noklusējuma lietotājs netika atrasts! Tiks automātiski izveidots jauns lietotājs...");

        const randomPassword = generateTempPassword(16);

        const newRootUser = rootUserTemplate;
        newRootUser.username = "root";
        newRootUser.password = await bcrypt.hash(randomPassword, SALT_ROUNDS);
        newRootUser.createdAt = new Date().toLocaleString("lv");

        await addUser(mongoClient, newRootUser);

        console.info(`Noklusējuma lietotājvārds: root\nNoklusējuma Parole: ${randomPassword}`);
        return;
    }

    if (rootUser && username) {
        if (rootUser.username !== username) {
            await updateUserUsername(mongoClient, rootUser._id, username);
            console.info("Noklusējuma lietotājvārds atjaunināts!");
        }
    }

    if (rootUser && password) {
        const hashedPassword = await bcrypt.hash(password, SALT_ROUNDS);
        await updateUserPassword(mongoClient, rootUser._id, hashedPassword);
        console.info("Noklusējuma parole atjaunināta!");
    }
}