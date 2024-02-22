"use client";
import React, { useRef, useEffect } from "react";
import { SocketLogger } from "../control/socketLogger";
import { useVideoLogger } from "@/hooks/useVideoLogger";

export const VideoStream = () => {
  const videoRef = useRef(null);
  const videoStream = useVideoLogger();
  // useEffect(() => {
  //   if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
  //     navigator.mediaDevices
  //       .getUserMedia({ video: true })
  //       .then((stream) => {
  //         if (videoRef.current) {
  //           videoRef.current.srcObject = stream;
  //         }
  //       })
  //       .catch((err) => {
  //         console.log(err);
  //       });
  //   }
  // }, []);

  return (
    <div>
      <SocketLogger
        title={"Video Logg"}
        data={videoStream.videoLog}
        className="mx-4 px-4 h-96 w-full"
      />
      <video ref={videoRef} autoPlay />;
    </div>
  );
};
