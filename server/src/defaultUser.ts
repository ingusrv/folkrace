import { PASSWORD_SALT_ROUNDS, DEFAULT_USERNAME, DEFAULT_PASSWORD } from "./config.js";
import bcrypt from "bcrypt";
import { getDefaultUser, addUser, updateUserUsername, updateUserPassword } from "./databaseMethods.js";
import { MongoClient } from "mongodb";
import { Role } from "./types.js";

export default async function setupDefaultUser(mongoClient: MongoClient) {
    const defaultUser = await getDefaultUser(mongoClient);
    const username = DEFAULT_USERNAME;
    const password = DEFAULT_PASSWORD;

    if (defaultUser && !username && !password) {
        return;
    }

    if (!defaultUser && username && password) {
        console.info("Noklusējuma lietotājs netika atrasts! Tiks izveidots jauns lietotājs no .env");
        await addUser(mongoClient, {
            username: username,
            password: await bcrypt.hash(password, PASSWORD_SALT_ROUNDS),
            role: Role.default
        });
        console.info("Lietotājs izveidots");
        return;
    }

    if (!defaultUser && !username && !password) {
        console.info("Noklusējuma lietotājs netika atrasts! Tiks automātiski izveidots jauns lietotājs...");

        await addUser(mongoClient, {
            username: "admin",
            password: await bcrypt.hash("folkrace", PASSWORD_SALT_ROUNDS),
            role: Role.default
        });

        console.info("Noklusējuma lietotājvārds: admin");
        console.info("Noklusējuma Parole: folkrace");
        return;
    }

    if (defaultUser && username) {
        if (defaultUser.username !== username) {
            await updateUserUsername(mongoClient, defaultUser._id, username);
            console.info("Noklusējuma lietotājvārds atjaunināts");
        }
    }

    if (defaultUser && password) {
        bcrypt.compare(password, defaultUser.password, async (err, result) => {
            if (err) {
                console.error(err.message);
                return;
            }

            if (!result) {
                const hashedPassword = await bcrypt.hash(password, PASSWORD_SALT_ROUNDS);
                await updateUserPassword(mongoClient, defaultUser._id, hashedPassword);
                console.info("Noklusējuma parole atjaunināta");
            }
        });
    }
}
