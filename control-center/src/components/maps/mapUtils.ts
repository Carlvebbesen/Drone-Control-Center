/**
 * The orientation of the map and the robot is given in quarternions,
 * this function converts to euler angles around the z-axis in a
 * right hand down coordinate system(tror jeg)
 */
export const ConvertFromQuaternionToEulerAngle = (
  orientation: Quaternion
): number => {
  const q = orientation;
  const roll = Math.atan2(
    2.0 * (q.x * q.y + q.w * q.z),
    q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z
  );
  return -roll - 0.5 * Math.PI;
};
type Coordinates = {
  x: number;
  y: number;
};
export const ConvertToROSCoordinates = (
  mapScale: number,
  mapOrigin: Pose,
  mapData: OccupancyGrid,
  x: number,
  y: number
): Coordinates => {
  return {
    x: (-(y - mapOrigin.y) * mapData.info.resolution) / mapScale,
    y: (-(x - mapOrigin.x) * mapData.info.resolution) / mapScale,
  };
};
export type LngLat = {
  lng: number;
  lat: number;
};
export type Pos = {
  x: number;
  y: number;
};
export const transformRosToGeoGraphic = (
  mapOriginGeo: LngLat,
  mapOrientation: number,
  poseRos: Pos
): LngLat => {
  const theta = mapOrientation;

  const transformedPose = {
    x: poseRos.x * Math.cos(theta) - poseRos.y * Math.sin(theta),
    y: poseRos.x * Math.sin(theta) + poseRos.y * Math.cos(theta),
  };

  // Position, decimal degrees
  const { lat, lng } = mapOriginGeo;

  // Earth’s radius, sphere
  const R = 6378137;

  // offsets in meters
  const de = transformedPose.x;
  const dn = transformedPose.y;

  // Coordinate offsets in radians
  const dLat = dn / R;
  const dLon = de / (R * Math.cos((Math.PI * lat) / 180));

  // OffsetPosition, decimal degrees
  const latO = lat + (dLat * 180) / Math.PI;
  const lonO = lng + (dLon * 180) / Math.PI;

  const offSetLngLat = {
    lng: lonO,
    lat: latO,
  };

  return offSetLngLat;
};
export const transformGeoToRos = (
  mapOriginGeo: LngLat,
  mapOrientation: number,
  poseGeo: LngLat
): Pos => {
  // Earth’s radius, sphere
  const R = 6378137;

  // Difference in latitude and longitude in radians
  const latD = ((poseGeo.lat - mapOriginGeo.lat) * Math.PI) / 180;
  const lngD = ((poseGeo.lng - mapOriginGeo.lng) * Math.PI) / 180;

  // Approximate difference north and east in meters
  const dn = latD * R;
  const de = lngD * R * Math.cos((Math.PI * poseGeo.lat) / 180); // We have to account for one of the circles being smaller the further north we are

  // Perform rotation transformation
  const theta = -mapOrientation;
  const transformedPose = {
    x: de * Math.cos(theta) - dn * Math.sin(theta),
    y: de * Math.sin(theta) + dn * Math.cos(theta),
  };

  return transformedPose;
};

export interface StringServiceRequest {
  data: string;
}

export interface StringServiceResponse {
  data: string;
}

/* eslint-disable */
// import { Vector3 } from "roslib";
// import { Pos } from "../utilities/MapUtils";
// import { MazeMarker } from "./MazeMapView";

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}
export interface Pose {
  x: number;
  y: number;
  rotation: number;
}
interface RosPose {
  position: Point;
  orientation: Quaternion;
}
interface Point {
  x: number;
  y: number;
  z?: number;
}
interface Stamp {
  sec: number;
  nsec: number;
}
interface Header {
  seq: number;
  stamp: Stamp;
  frame_id: string;
}
export interface OccupancyGrid {
  header: Header;
  info: {
    time_map: Stamp;
    resolution: number;
    width: number;
    height: number;
    origin: RosPose;
  };
  data: number[];
}
interface PoseWithCovariance {
  pose: RosPose;
  covariance: number[];
}
// export interface Twist {
//   linear: Vector3;
//   angular: Vector3;
// }
// interface TwistWithCovariance {
//   twist: Twist;
//   covariance: number[];
// }
// export interface OdomMessage {
//   header: Header;
//   child_frame_id: string;
//   pose: PoseWithCovariance;
//   twist: TwistWithCovariance;
// }

export interface Transform {
  translation: {
    x: number;
    y: number;
    z: number;
  };
  rotation: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
}

export interface TransformStamped {
  header: Header;
  child_frame_id: string;
  transform: Transform;
}

export type TfMessage = { transforms: TransformStamped[] };

export interface Message {
  data: string;
}

// export type PatrolPoint = {
//   pos: Pos;
//   marker: MazeMarker;
// };
