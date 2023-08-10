import { MongoClient } from "mongodb";

if (!process.env.DATABASE_URI) {
    console.error("Nav iestatīts 'DATABASE_URI'! Lūdzu izveidojiet .env failu un iestatiet šo vērtību!");
    process.exit(1);
}

export const mongoClient = new MongoClient(process.env.DATABASE_URI);
await mongoClient.connect();
