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
    _id: string;
    username: string;
    role: Role;
    token: string;
    createdAt: Date;
}

export type Robot = {
    _id: string;
    owner: string;
    name: string;
    token: string;
    createdAt: Date;
    delay: number;
    state: RobotState;
}

export type DriveData = {
    _id: string;
    owner: string;
    robot: string;
    name: string;
    algorithm: string;
    version: string;
    elapsedTime: number;
    data: string[];
    fps: number[];
    averageFps: number;
    createdAt: Date;
}
