import { MongoClient } from "mongodb";
import assert from "assert";

let _mongoClient: MongoClient;

export function getMongoInstance(): MongoClient {
    assert.ok(_mongoClient, "Datubāze nav savienota!");
    return _mongoClient;
}

export function initMongoDb(connectionString: string): Promise<MongoClient> {
    return new Promise((resolve, reject) => {
        if (_mongoClient) {
            reject("Datubāze jau ir savienota!");
        }

        const mongoClient = new MongoClient(connectionString);

        mongoClient.connect().then(() => {
            _mongoClient = mongoClient;
            resolve(mongoClient);
        });
    });
}
