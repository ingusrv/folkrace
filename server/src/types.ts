import { ObjectId } from "mongodb";

export enum MessageType {
    status,
    start,
    stop,
    data,
    error,
}

export enum RobotState {
    disconnected,
    connected,
    running,
    error,
}

export enum Role {
    default,
    user,
    admin,
}

export type User = {
    _id: ObjectId;
    username: string;
    password: string;
    role: Role;
    token: string | null;
    createdAt: Date;
}

export type Robot = {
    _id: ObjectId;
    owner: ObjectId;
    name: string;
    token: string;
    createdAt: Date;
}

export type DriveData = {
    _id: ObjectId;
    owner: ObjectId;
    robot: ObjectId;
    name: string;
    algorithm: string;
    version: string;
    elapsedTime: number;
    data: string[];
    fps: number[];
    averageFps: number;
    createdAt: Date;
}
