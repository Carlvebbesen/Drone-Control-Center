"use client";
import { SocketDataType } from "../../hooks/socketTypes";
import { useControlSocket } from "../../hooks/useControlSocket";
import { useArrowKeyPress } from "../../hooks/useKeyboardListener";
import { Button } from "../ui/button";
import { SocketLogger } from "./socketLogger";

export const ManualControl = () => {
  const socket = useControlSocket();
  useArrowKeyPress((command) => {
    socket.sendControlEvent(command);
  });
  return (
    <div className="flex flex-col my-8 justify-center items-center h-full">
      <Button onClick={() => socket.testConnection()}>Test Connection</Button>
      <div className="grid grid-cols-2 w-full gap-10 mt-20 h-full">
        <SocketLogger
          className="mx-4 px-4 h-96 w-full"
          data={socket.receivedData}
          title="Received Socket Data"
        />
        <SocketLogger
          className="mx-4 px-4 min-h-96 h-96 w-full"
          data={socket.sentData}
          title="Sent Socket Data"
        />
      </div>
    </div>
  );
};
