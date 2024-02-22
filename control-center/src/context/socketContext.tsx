import { SocketContextInterface, SocketDataType } from "@/hooks/socketTypes";
import { ReactNode, createContext, useEffect, useState } from "react";
import { Socket, io } from "socket.io-client";

export const SocketContext = createContext<SocketContextInterface | null>(null);
export const SocketContextProvider = ({
  children,
}: {
  children: React.ReactNode;
}) => {
  const [sentData, setSentData] = useState<SocketDataType[]>([]);
  const [receivedData, setReceivedData] = useState<SocketDataType[]>([]);
  const [socket, setSocket] = useState<Socket<any, any> | null>(null);

  useEffect(() => {
    const socketId = io("127.0.0.1:5000");
    setSocket(socketId);
  }, []);

  const updateReceivedData = ({
    msg,
    type,
  }: {
    msg: string;
    type: SocketDataType["type"];
  }) => {
    const now = new Date().toLocaleTimeString();
    setReceivedData((prev) => [{ time: now, type: type, msg: msg }, ...prev]);
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
        updateReceivedData({ type: "connected", msg: `id: ${socket.id}` });
        const engine = socket.io.engine;
        engine.on("close", (reason) =>
          updateReceivedData({ msg: reason, type: "disconnected" })
        );
      });
      socket.on("test", (msg: string) => {
        updateReceivedData({ type: "command", msg: msg });
      });
      socket.on("connect_error", () => {
        updateReceivedData({
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
        updateReceivedData({
          msg: details["description"],
          type: "disconnected",
        });
      });
      socket.on("error", (errorMsg: string) => {
        updateReceivedData({
          msg: errorMsg,
          type: "error",
        });
      });
    }
  }, [socket]);
  const sendControlEvent = (controlCommand: string) => {
    if (socket) {
      updateSentData({ type: "command", msg: controlCommand });
      socket.volatile.emit(
        "manual_control",
        controlCommand,
        (response: any) => {
          updateReceivedData({ type: "command", msg: response });
        }
      );
    }
  };
  const testConnection = () => {
    if (socket) {
      socket.emit("test", "test ack return", (response: any) => {
        updateReceivedData({ type: "command", msg: response });
        updateSentData({ type: "command", msg: "Init testing ..." });
      });
    }
  };

  return (
    <SocketContext.Provider
      value={{
        sentData: sentData,
        receivedData: receivedData,
        testConnection: testConnection,
        sendControlCommand: sendControlEvent,
        getSocketConnection: socket,
      }}
    >
      {children}
    </SocketContext.Provider>
  );
};
