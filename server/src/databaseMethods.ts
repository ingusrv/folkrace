import { MongoClient, ObjectId } from "mongodb";
import { DriveData, Robot, Role, User } from "./types.js";
export async function getUserByToken(client: MongoClient, token: string) {
    const res = await client.db("folkrace").collection("users").findOne<User>({ token: token });
    return res;
}

export async function setToken(client: MongoClient, userId: ObjectId, token: string | null) {
    const res = await client.db("folkrace").collection("users").updateOne(
        { _id: userId },
        { $set: { token: token } }
    );
    return res.acknowledged;
}

export async function getDefaultUser(client: MongoClient) {
    const res = await client.db("folkrace").collection("users").findOne<User>({ role: Role.default });
    return res;
}

export async function getUser(client: MongoClient, userId: ObjectId) {
    const res = await client.db("folkrace").collection("users").findOne<User>({ _id: userId });
    return res;
}

export async function addUser(client: MongoClient, user: { username: string; password: string; role: Role; }) {
    const u: User = {
        _id: new ObjectId(),
        username: user.username,
        password: user.password,
        role: user.role,
        token: null,
        createdAt: new Date()
    };
    const res = await client.db("folkrace").collection("users").insertOne(u);
    return res.acknowledged ? u : null;
}

export async function removeUser(client: MongoClient, userId: ObjectId) {
    const res = await client.db("folkrace").collection("users").deleteOne({ _id: userId });
    return res.acknowledged;
}

export async function updateUserUsername(client: MongoClient, userId: ObjectId, username: string) {
    const res = await client.db("folkrace").collection("users").updateOne({ _id: userId }, { $set: { username: username } });
    return res.acknowledged;
}

export async function updateUserPassword(client: MongoClient, userId: ObjectId, password: string) {
    const res = await client.db("folkrace").collection("users").updateOne({ _id: userId }, { $set: { password: password } });
    return res.acknowledged;
}

export async function getUserByUsername(client: MongoClient, username: string) {
    const res = await client.db("folkrace").collection("users").findOne<User>({ username: username });
    return res;
}

export async function getAllUsers(client: MongoClient) {
    const cursor = client.db("folkrace").collection("users").find<User>({});
    const res = await cursor.toArray();
    return res;
}

export async function getDriveDataByOwner(client: MongoClient, ownerId: ObjectId) {
    const cursor = client.db("folkrace").collection("driveData").find<DriveData>({ owner: ownerId });
    const res = await cursor.toArray();
    return res;
}

export async function addDriveData(client: MongoClient, driveData: DriveData) {
    const res = await client.db("folkrace").collection("driveData").insertOne(driveData);
    return res.acknowledged;
}

export async function removeDriveData(client: MongoClient, ownerId: ObjectId, dataId: ObjectId) {
    const res = await client.db("folkrace").collection("driveData").deleteOne({ _id: dataId, owner: ownerId });
    return res.acknowledged;
}

export async function getRobotsByOwner(client: MongoClient, ownerId: ObjectId) {
    const cursor = client.db("folkrace").collection("robots").find<Robot>({ owner: ownerId });
    const res = await cursor.toArray();
    return res;
}

export async function addRobot(client: MongoClient, robot: { ownerId: ObjectId, name: string, token: string }) {
    const r: Robot = {
        _id: new ObjectId(),
        owner: robot.ownerId,
        name: robot.name,
        token: robot.token,
        createdAt: new Date()
    };
    const res = await client.db("folkrace").collection("robots").insertOne(r);
    return res.acknowledged ? r : null;
}

export async function getRobotByToken(client: MongoClient, token: string) {
    const res = await client.db("folkrace").collection("robots").findOne<Robot>({ token: token });
    return res;
}

export async function removeRobot(client: MongoClient, ownerId: ObjectId, robotId: ObjectId) {
    const res = await client.db("folkrace").collection("robots").deleteOne(
        { robotId: robotId, owner: ownerId }
    );
    return res.acknowledged;
}

export async function setRobotToken(client: MongoClient, ownerId: ObjectId, robotId: ObjectId, token: string) {
    const res = await client.db("folkrace").collection("robots").updateOne(
        { _id: robotId, owner: ownerId },
        { $set: { token: token } }
    );
    return res.acknowledged;
}
