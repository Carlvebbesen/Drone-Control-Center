"use client";

import { useEffect, useState } from "react";
import { Socket, io } from "socket.io-client";
import { SocketDataType } from "./socketTypes";

export const useVideoLogger = () => {
  const [videoLog, setVideoLog] = useState<SocketDataType[]>([]);
  const [socket, setSocket] = useState<Socket<any, any> | null>(null);

  useEffect(() => {
    const socketId = io("127.0.0.1:5000");
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
    setVideoLog((prev) => [{ time: now, type: type, msg: msg }, ...prev]);
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
      socket.on("error", (errorMsg: string) => {
        updateData({
          msg: errorMsg,
          type: "error",
        });
      });
      socket.on("image_data", (data: any) => {
        updateData({ msg: data, type: "command" });
      });
    }
  }, [socket]);
  return {
    videoLog,
    socket,
  };
};
