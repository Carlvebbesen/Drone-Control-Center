import React, { ReactElement } from "react";

export const GenericMarker = ({ children }: { children: ReactElement }) => {
  return (
    <div
      // className="marker mapboxgl-marker mapboxgl-marker-anchor-center zlevel-marker"
      style={{
        position: "relative",
        width: "36px",
        height: "44.5714px",
        opacity: 1,
        // transform:
        //     'translate(-50%, -50%) translate(234px, 245px) rotateX(0deg) rotateZ(0deg)',
      }}
    >
      {children}
    </div>
  );
};
