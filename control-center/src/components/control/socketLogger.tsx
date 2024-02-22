import { SocketDataType } from "@/hooks/socketTypes";

interface SocketLoggerProps {
  title: string;
  data: SocketDataType[];
  className: string;
}
export const SocketLogger = (props: SocketLoggerProps) => {
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
