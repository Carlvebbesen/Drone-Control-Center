import type { ColumnDef } from "@tanstack/react-table";
import { TelloDrone } from "../logo/telloDrone";
import { format } from "date-fns";

// This type is used to define the shape of our data.
// You can use a Zod schema here if you want.
export type InspectionColumns = {
  name: number;
  detensionCount: number;
  status: string;
  launchType: string;
  scheduleType: string;
  time: string;
  elapsedTime: string;
  droneId: string;
};

const getColor = (state: string) => {
  if (state === "error") return "red";
  if (state === "onGoing") return "gray";
  if (state === "success") return "green";
  return "";
};

export const columns: ColumnDef<InspectionColumns>[] = [
  {
    accessorKey: "name",
    header: "InspeksjonsNavn",
    cell: ({ row }) => {
      return <div>{`#${row.getValue("name")} Inspeksjon`}</div>;
    },
  },
  {
    accessorKey: "droneId",
    header: "Dronen",
    cell: ({ row }) => {
      return (
        <div>
          <p>{row.getValue("droneId")}</p>
          <TelloDrone size={40} />
        </div>
      );
    },
  },
  {
    accessorKey: "status",
    header: "Status",
    cell: ({ row }) => {
      const value = row.getValue("status") as string;
      return (
        <div className={"font-semibold"} style={{ color: getColor(value) }}>
          {value}
        </div>
      );
    },
  },
  // {
  //   accessorKey: "launchType",
  //   header: "Trigger",
  //   cell: ({ row }) => {
  //     return <div>{row.getValue("launchType")}</div>;
  //   },
  // },
  {
    accessorKey: "time",
    header: "StartTid",
    cell: ({ row }) => {
      return (
        <div>{format(new Date(row.getValue("time")), "EE/MM/yy k:m")}</div>
      );
    },
  },
  // {
  //   accessorKey: "elapsedTime",
  //   header: "Varighet",
  //   cell: ({ row }) => {
  //     return <div>{row.getValue("elapsedTime")}</div>;
  //   },
  // },
  {
    accessorKey: "detensionCount",
    header: "Avvik",
    cell: ({ row }) => {
      return <div>{row.getValue("detensionCount")}</div>;
    },
  },
];
