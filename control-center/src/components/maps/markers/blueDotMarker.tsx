import React from "react";
import { GenericMarker } from "./genericMarker";

export const MapMarkerBlueDot = () => {
  return (
    <GenericMarker>
      <svg height="100" width="100">
        <circle
          cx="19"
          cy="20"
          r="15"
          stroke="white"
          strokeWidth="3"
          fill="#1FAFFC"
          fillOpacity="0.5"
        />
        <circle
          cx="19"
          cy="20"
          r="10"
          stroke="white"
          strokeWidth="3"
          fill="#1FAFFC"
        />
      </svg>
    </GenericMarker>
  );
};
