import { ManualControl } from "@/components/control/manualControl";
import { VideoStream } from "@/components/video/videoStream";

const Debug = () => {
  return (
    <div>
      <VideoStream />
      <ManualControl />
    </div>
  );
};

export default Debug;
