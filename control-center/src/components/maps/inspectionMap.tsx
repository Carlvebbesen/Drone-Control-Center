"use client";

import React, { useEffect, useRef, useState } from "react";
import useScript from "react-script-hook";

import {
  LngLat,
  Pos,
  StringServiceRequest,
  StringServiceResponse,
  transformGeoToRos,
  transformRosToGeoGraphic,
} from "./mapUtils";
declare global {
  let Mazemap: any;
}

export type BlueDot = {
  setLngLat: (lngLat: LngLat) => BlueDot;
  setZLevel: (level: number) => BlueDot;
  setAccuracy: (level: number) => BlueDot;
  show: () => void;
  destroy: () => void;
};

export type MazeMarker = {
  setLngLat: (lngLat: LngLat) => MazeMarker;
  setZLevel: (level: number) => MazeMarker;
  setAccuracy: (level: number) => MazeMarker;
  show: () => void;
  addTo: (map: any) => MazeMarker;
  remove: () => void;
  options: any;
  _lngLat: LngLat;
  on: (event: string, callback: () => void) => void;
};

type MazeMapViewProps = {
  mapOriginGeo: LngLat;
  mapAngleTransform: number;
  onMapPointClick?: (
    e: any,
    rosPoint: Pos,
    Mazemap: any,
    mapRef: React.MutableRefObject<any>
  ) => void;
  onMapReady?: (Mazemap: any, mapRef: React.MutableRefObject<any>) => void;
  containerClassName?: string;
  zoom?: number;
  jumpTo?: LngLat;
  zLevel?: number;
  showMapOrigin?: boolean;
  legend?: JSX.Element;
};

export const InspectionMap = () => {
  const myMapRef = useRef<any>();
  const [mapReady, setMapReady] = useState<boolean>(false);
  const [loading, error] = useScript({
    src: "https://api.mazemap.com/js/v2.0.114/mazemap.min.js",
  });
  // 63.416778387237926, 10.402715401799476
  const jumpTo = { lng: 10.402715401799476, lat: 63.416708387237926 };
  const zoom = 19;
  const [lastDots, setLastDots] = useState<
    {
      location: { lngLat: { lng: number; lat: number }; zLevel: number };
      ref: BlueDot;
    }[]
  >([]);
  const routeControllerRef = useRef<any>();
  const onMapClick = (e: any) => {
    let zLevel = myMapRef.current.zLevel;
    const dotRef = createDot(e.lngLat, zLevel);
    setLastDots((prev) => [
      ...prev,
      { location: { lngLat: e.lngLat, zLevel: zLevel }, ref: dotRef },
    ]);
    Mazemap.Data.getPoiAt(e.lngLat, zLevel).then((poi) => {
      // Highlight the room
      highlightRoom({ poi: poi });
    });
  };

  const createDot = (lngLat: { lng: number; lat: number }, zLevel: number) => {
    return new Mazemap.BlueDot({
      map: myMapRef.current,
      color: "red",
    })
      .setLngLat(lngLat)
      .setZLevel(zLevel)
      .setAccuracy(1)
      .show();
  };
  const highlightRoom = ({ poi }: { poi: any }) => {
    // If the POI has a polygon, use the default 'highlight' function to draw a marked outline around the POI.
    myMapRef.current.highlighter.highlight(poi);
  };

  useEffect(() => {
    if (!loading) {
      myMapRef.current = new Mazemap.Map({
        campuses: 1,
        container: "mazemap-container",
        zlevel: 1,
      });

      myMapRef.current.jumpTo({
        center: jumpTo,
        zoom: zoom,
        // speed: 2.5,
      });

      myMapRef.current.on("click", onMapClick);

      const onRotate = (e: any): void => {
        console.log(e);
        console.log(myMapRef.current.transform);
      };

      // console.log(myMapRef.current)
      myMapRef.current.on("rotate", onRotate);

      const onLoad = (): void => {
        myMapRef.current.highlighter = new Mazemap.Highlighter(
          myMapRef.current,
          {
            showOutline: true, // optional
            showFill: true, // optional
            outlineColor: Mazemap.Util.Colors.MazeColors.MazeBlue, // optional
            fillColor: Mazemap.Util.Colors.MazeColors.MazeBlue, // optional
          }
        );
        routeControllerRef.current = new Mazemap.RouteController(
          myMapRef.current,
          {
            showDirectionArrows: true, // Turn off if you don't want direction arrows
          }
        );
        setMapReady(true);
      };

      myMapRef.current.on("load", onLoad);
    }

    return undefined;
  }, [loading]);

  useEffect(() => {
    if (lastDots.length > 1) {
      // routeControllerRef.current.clear();
      Mazemap.Data.getRouteJSON(
        lastDots[lastDots.length - 2].location,
        lastDots[lastDots.length - 1].location
      ).then(function (geojson) {
        routeControllerRef.current.setPath(geojson);
        const coordinates: number[][] =
          geojson.features[0].geometry.coordinates;
        const zLevel: number = geojson.features[0].properties.z;
        coordinates.forEach((element) => {
          createDot({ lat: element[1], lng: element[0] }, zLevel);
        });
        var bounds = Mazemap.Util.Turf.bbox(geojson);
        myMapRef.current.fitBounds(bounds, { padding: 100 });
      });
    }
  }, [lastDots]);
  return (
    <div className={"w-[500px] h-[500px]"}>
      <div id="mazemap-container" className={"w-full h-full"} />
      {/* {legend && legend} */}
    </div>
  );
};
