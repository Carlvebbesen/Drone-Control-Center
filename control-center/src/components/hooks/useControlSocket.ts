"use client";

import { useEffect, useState } from "react";
import { Socket, io } from "socket.io-client";
import { SocketDataType } from "./socketTypes";

export const useControlSocket = () => {
  const [sentData, setSentData] = useState<SocketDataType[]>([]);
  const [socketData, setSocketData] = useState<SocketDataType[]>([]);
  const [socket, setSocket] = useState<Socket<any, any> | null>(null);

  useEffect(() => {
    const socketId = io("0.0.0.0:5683");
    setSocket(socketId);
  }, []);

  const updateData = ({
    msg,
    type,
  }: {
    msg: string;
    type: SocketDataType["type"];
  }) => {
    const now = new Date().toLocaleTimeString();
    setSocketData((prev) => [{ time: now, type: type, msg: msg }, ...prev]);
  };
  const updateSentData = ({
    msg,
    type,
  }: {
    msg: string;
    type: SocketDataType["type"];
  }) => {
    const now = new Date().toLocaleTimeString();
    setSentData((prev) => [{ time: now, type: type, msg: msg }, ...prev]);
  };
  useEffect(() => {
    if (socket) {
      socket.on("connect", () => {
        updateData({ type: "connected", msg: `id: ${socket.id}` });
        const engine = socket.io.engine;
        engine.on("close", (reason) =>
          updateData({ msg: reason, type: "disconnected" })
        );
      });
      socket.on("test", (msg: string) => {
        updateData({ type: "command", msg: msg });
      });
      socket.on("connect_error", () => {
        updateData({
          msg: "socket closed, Reconnecting ... ",
          type: "error",
        });
        setTimeout(() => {
          socket.connect();
        }, 3000);
      });
      socket.on("disconnect", (reason, details) => {
        if (reason === "io server disconnect") {
          // the disconnection was initiated by the server, you need to reconnect manually
          socket.connect();
        }
        updateData({
          msg: details["description"],
          type: "disconnected",
        });
      });

      socket.on("video", (video: string) => {
        console.log(typeof video);
        updateData({
          msg: "video",
          type: "command",
        });
      });
    }
  }, [socket]);

  const sendControlEvent = (controlCommand: string) => {
    //socket.volatile.emit("command", "if we need to only send latest event, and dont buffer up");
    if (socket) {
      socket.emit("control", controlCommand, (response: any) => {
        updateSentData({ type: "command", msg: response });
      });
    }
  };
  const testConnection = () => {
    if (socket) {
      socket.emit("test", "test successful");
      updateSentData({ type: "command", msg: "testing ..." });
    }
  };
  return {
    socketData,
    sentData,
    sendControlEvent,
    testConnection,
  };
};
