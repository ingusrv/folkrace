import { MongoClient } from "mongodb";

async function listDatabases(client) {
    const databaseList = await client.db().admin().listDatabases();
    console.log("Databases:");
    databaseList.databases.forEach(element => {
        console.log(element.name);
    });
}

export async function addUser(client, user) {
    const res = await client.db("folkrace").collection("users").insertOne(user);
    console.log(`user ${user.username} added with id ${res.insertedId}`);
    return res.insertedId;
}

async function addMultipleUsers(client, users) {
    const res = await client.db("folkrace").collection("users").insertMany(users);
    console.log(`${res.insertedCount} users added with ids:`);
    console.log(res.insertedIds);
}

export async function getUser(client, username) {
    const res = await client.db("folkrace").collection("users").findOne({ username: username });
    if (res) {
        console.log(`user ${username} found`);
        return res;
    } else {
        console.log(`user ${username} not found`);
        return undefined;
    }
}

export async function getUsers(client) {
    const cursor = client.db("folkrace").collection("users").find({});
    const res = await cursor.toArray();
    console.log("All users: ");
    res.forEach(element => {
        console.log(element.username);
    });
    return res;
}

export async function removeUser(client, username) {
    const res = await client.db("folkrace").collection("users").deleteOne({ username: username });
    console.log(`user ${username} removed`);
    // TODO: change to something better
    return res.deletedCount;
}

async function removeMultipleUsers(client, usernames) {
    const res = await client.db("folkrace").collection("users").deleteMany({ username: { $in: usernames } });
    console.log(`${res.deletedCount} users removed`);
    return res.deletedCount;
}

export async function addDriveData(client, driveData) {
    const res = await client.db("folkrace").collection("driveData").insertOne(driveData);
    console.log(`data inserted with id: ${res.insertedId}`);
    return res.insertedId;
}

export async function getDriveDataByOwner(client, username) {
    const cursor = client.db("folkrace").collection("driveData").find({ owner: username });
    const res = await cursor.toArray();
    return res;
}

export async function getRobotsByOwner(client, username) {
    const cursor = client.db("folkrace").collection("robots").find({ owner: username });
    const res = await cursor.toArray();
    return res;
}

export async function addRobot(client, robot) {
    const res = await client.db("folkrace").collection("robots").insertOne(robot);
    console.log(`robot inserted with id: ${res.insertedId}`);
    return res.insertedId;
}

export async function getRobotById(client, robotId) {
    const res = client.db("folkrace").collection("robots").findOne({ robotId: robotId });
    if (!res) {
        return undefined;
    }

    return res;
}

export async function getRobotByKey(client, key) {
    const res = client.db("folkrace").collection("robots").findOne({ key: key });
    if (!res) {
        return undefined;
    }

    return res;
}

export async function removeRobot(client, robotId) {
    const res = await client.db("folkrace").collection("robots").deleteOne({ robotId: robotId });
    return res.deletedCount;
}

export async function updateRobotKey(client, robotId, key) {
    const res = await client.db("folkrace").collection("robots").updateOne({ robotId: robotId }, { $set: { key: key } });
    return res.modifiedCount;
}

export async function updateRobotStatus(client, robotId, status) {
    const res = await client.db("folkrace").collection("robots").updateOne({ robotId: robotId }, { $set: { status: status, lastUpdated: new Date().toLocaleString("lv") } });
    return res.modifiedCount;
}

async function main() {
    const uri = "mongodb://127.0.0.1:27017";
    const client = new MongoClient(uri);

    try {
        await client.connect();
        // await listDatabases(client);
        const root = {
            "username": "admin",
            "password": "admin",
            "root": true,
            "admin": true
        };
        // await addUser(client, root);

        const userlist = [
            {
                username: "foo",
                password: "foo",
                root: false,
                admin: false
            },
            {
                username: "bar",
                password: "bar",
                root: false,
                admin: false
            },
            {
                username: "baz",
                password: "baz",
                root: false,
                admin: false
            }
        ]

        await addMultipleUsers(client, userlist);

        await getUsers(client);
        await removeMultipleUsers(client, ["foo", "bar", "baz"]);
        await getUsers(client);
    } catch (e) {
        console.error(e);
    } finally {
        await client.close();
    }
}

// main().catch(console.error);
