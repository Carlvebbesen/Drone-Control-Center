"use client";
import { SocketDataType } from "../hooks/socketTypes";
import { useControlSocket } from "../hooks/useControlSocket";
import { useArrowKeyPress } from "../hooks/useKeyboardListener";
import { Button } from "../ui/button";

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
          className="mx-4 px-4 overflow-scroll h-96 w-full"
          data={socket.socketData}
          title="Socket Data"
        />
        <SocketLogger
          className="mx-4 px-4 min-h-96 overflow-scroll h-96 w-full"
          data={socket.sentData}
          title="Sent Data"
        />
      </div>
    </div>
  );
};

interface SocketLoggerProps {
  title: string;
  data: SocketDataType[];
  className: string;
}
const SocketLogger = (props: SocketLoggerProps) => {
  const getTypeColor = (type: SocketDataType["type"]) => {
    if (["disconnected", "error"].includes(type)) {
      return "red";
    }
    if (type === "connected") {
      return "lightGreen";
    } else {
      return "lightGray";
    }
  };

  return (
    <div className={props.className}>
      <h1>{props.title}</h1>
      <div className="border border-red-400">
        {props.data.map((data, index) => (
          <div key={`${index} ${data.time}`} className="flex gap-2">
            <p
              style={{
                backgroundColor: getTypeColor(data.type),
              }}
            >
              {data.type}
            </p>
            <p>{data.time}</p>
            <p>{data.msg}</p>
          </div>
        ))}
      </div>
    </div>
  );
};
