import React from "react";
import style from "./markers.module.scss";

export const MapMarkerA: React.FC = () => {
  return (
    <div
      // className="marker mapboxgl-marker mapboxgl-marker-anchor-center zlevel-marker"
      className={style.marker}
      style={{
        width: "36px",
        height: "44.5714px",
        opacity: 1,
        // transform:
        //     'translate(-50%, -50%) translate(234px, 245px) rotateX(0deg) rotateZ(0deg)',
      }}
    >
      <span
        style={{
          color: "#FFF",
          display: "inline-block",
          position: "absolute",
          top: "4.628571428571429px",
          height: "29.142857142857142px",
          lineHeight: "29.142857142857142px",
          left: 0,
          right: "0px",
          textAlign: "center",
          fontSize: "23.314285714285717px",
        }}
      >
        A
      </span>
      <svg
        width="100%"
        height="100%"
        viewBox="0 0 42 52"
        version="1.1"
        xmlns="http://www.w3.org/2000/svg"
        xmlnsXlink="http://www.w3.org/1999/xlink"
      >
        <ellipse
          fill="#000000"
          fillOpacity="0.112715127"
          id="Oval-7"
          cx="21"
          cy="50"
          rx="8"
          ry="2"
        />
        <path
          d="M.93 21.044c0 6.357 2.983 12.215 7.961 16.002l.445.353c3.663 2.681 6.486 5.332 8.597 7.848a26.548 26.548 0 0 1 1.783 2.359c.27.405.432.682.5.812a1 1 0 0 0 1.778-.01c.066-.13.225-.405.488-.81a25.688 25.688 0 0 1 1.757-2.355c2.087-2.515 4.895-5.164 8.525-7.82.093-.068.2-.151.32-.247.25-.205.437-.363.524-.44 4.738-3.797 7.55-9.513 7.55-15.692C41.158 9.935 32.152.93 21.044.93 9.935.93.93 9.935.93 21.044z"
          fill="#FFF"
          stroke="#d6d6d6"
        />
        <path
          d="M19.465 43.961c-2.213-2.638-5.154-5.4-8.916-8.152l-.43-.342c-4.504-3.426-7.189-8.7-7.189-14.423C2.93 11.04 11.04 2.93 21.044 2.93c10.004 0 18.114 8.11 18.114 18.114 0 5.565-2.531 10.71-6.8 14.132-.143.123-.309.263-.48.403-.147.118-.23.183-.295.23-3.764 2.754-6.692 5.516-8.883 8.157a30.172 30.172 0 0 0-1.61 2.104 29.99 29.99 0 0 0-1.625-2.109z"
          fill="#1FAFFC"
        />
      </svg>
    </div>
  );
};
