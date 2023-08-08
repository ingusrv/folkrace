import { MongoClient } from "mongodb";
export const mongoClient = new MongoClient(process.env.DATABASE_URI);
await mongoClient.connect();
