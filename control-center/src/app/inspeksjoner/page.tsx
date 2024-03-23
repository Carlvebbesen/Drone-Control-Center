"use client";

import { InspectionMap } from "@/components/maps/inspectionMap";
import InspectionTable from "@/components/tables/inspectionTable";

const Inspections = () => {
  return (
    <div className="flex w-full h-full justify-center items-center">
      <InspectionTable />
      <InspectionMap />
    </div>
  );
};

export default Inspections;
