import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import Image from "next/image";
import { format } from "date-fns";
import { TelloDrone } from "@/components/logo/telloDrone";

const Droner = () => {
  const drones = [
    {
      id: "d40d6d9a-46fc-4c6f-badc-907e931a3fd3",
      name: "Drone9",
      isConnected: true,
      lastUsed: "1999-11-12T09:49:21.263Z",
      building: "NTNU",
      buildingArea: "Dewitt Avenue",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "03ed6349-2abc-46cb-aad5-4b310c07680c",
      name: "Drone5",
      isConnected: true,
      lastUsed: "2019-04-12T01:27:55.198Z",
      building: "Rockheim",
      buildingArea: "Lake Street",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "e39e70ce-b06c-4575-ac28-b2cc6c28b863",
      name: "Drone8",
      isConnected: true,
      lastUsed: "2004-04-20T18:50:46.144Z",
      building: "Trondheim Spektrum",
      buildingArea: "Grant Avenue",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "2f8c9776-64aa-4091-80d8-a1cd5599301e",
      name: "Drone4",
      isConnected: true,
      lastUsed: "2022-05-06T14:56:19.171Z",
      building: "Trondheim Spektrum",
      buildingArea: "Bergen Street",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "0e8be7f8-5833-47da-96bf-ee3ee977a8e7",
      name: "Drone1",
      isConnected: true,
      lastUsed: "2006-08-30T13:52:16.017Z",
      building: "NTNU",
      buildingArea: "Independence Avenue",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "24ed807d-4260-4c24-85d3-e38b67668f70",
      name: "Drone5",
      isConnected: true,
      lastUsed: "2000-09-03T09:17:58.372Z",
      building: "Rockheim",
      buildingArea: "Coyle Street",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "ebe1d767-cec0-49a5-8017-0fdbc89caf43",
      name: "Drone1",
      isConnected: true,
      lastUsed: "2002-11-05T19:20:42.204Z",
      building: "Rockheim",
      buildingArea: "Colonial Road",
      batteryPercentage: 80,
      temperature: 37,
    },
    {
      id: "840e74a1-e34d-4f30-9b1c-885cf942ed5a",
      name: "Drone0",
      isConnected: false,
      lastUsed: "1970-09-14T04:34:38.074Z",
      building: "Rockheim",
      buildingArea: "Varanda Place",
      batteryPercentage: 80,
      temperature: 37,
    },
  ];

  return (
    <div className="flex justify-center items-center flex-col max-w-screen-2xl">
      <h1 className="text-center text-6xl font-semibold mb-11">Dine Droner</h1>
      <div className="justify-between items-center flex gap-10 flex-wrap">
        {drones.map((drone) => (
          <Card key={drone.id} className="min-w-72 w-72 min-h-72">
            <CardHeader>
              <CardTitle>{drone.name}</CardTitle>
              <CardDescription>
                Denne dronen hører til {drone.building}, Område:{" "}
                <p className="font-bold inline">{drone.buildingArea}</p>
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="flex justify-center items-center flex-col gap-2">
                <TelloDrone size={100} />
                <div className="flex items-center gap-2">
                  <p>Drone status:</p>
                  <span
                    style={{ background: drone.isConnected ? "green" : "red" }}
                    className="rounded-full w-5 h-5"
                  ></span>
                </div>
                {drone.isConnected && (
                  <div className="flex items-center gap-2">
                    <div className="flex items-center gap-2">
                      <p>Temp:</p>
                      <p
                        style={{
                          color: drone.temperature < 60 ? "green" : "red",
                        }}
                      >
                        {drone.temperature} C
                      </p>
                    </div>
                    <div className="flex items-center gap-2">
                      <p>Batteri:</p>
                      <p
                        style={{
                          color: drone.batteryPercentage > 50 ? "green" : "red",
                        }}
                      >
                        {drone.batteryPercentage} %
                      </p>
                    </div>
                  </div>
                )}
                <p className="text-sm text-gray-500">
                  Last inspection: {format(drone.lastUsed, "EE/MM/yy k:m")}
                </p>
              </div>
            </CardContent>
          </Card>
        ))}
      </div>
    </div>
  );
};

export default Droner;
