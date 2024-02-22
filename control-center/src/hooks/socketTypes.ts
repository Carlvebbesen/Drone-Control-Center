import { Socket } from "socket.io-client";

export interface ServerToClientEvents {
  noArg: () => void;
  basicEmit: (a: number, b: string, c: Buffer) => void;
  withAck: (d: string, callback: (e: number) => void) => void;
}

export interface SocketDataType {
  time: string;
  type: "connected" | "error" | "disconnected" | "message" | "command";
  msg: string;
}

export interface ClientToServerEvents {
  hello: () => void;
}

export interface InterServerEvents {
  ping: () => void;
}

export interface SocketData {
  name: string;
  age: number;
}

export interface SocketContextInterface {
  sentData: SocketDataType[];
  receivedData: SocketDataType[];
  testConnection: () => void;
  sendControlCommand: (controlCommand: string) => void;
  getSocketConnection: Socket<any, any> | null;
}
