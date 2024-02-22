import { useEffect } from "react";

export function useArrowKeyPress(callback: (cmd: string) => void) {
  useEffect(() => {
    const keyBoardMapping = {
      ArrowUp: "forward",
      ArrowDown: "backward",
      ArrowLeft: "left",
      ArrowRight: "right",
      w: "up",
      a: "rotate_left",
      s: "down",
      d: "rotate_right",
      l: "land",
      e: "e",
      t: "takeoff",
      f: "flip",
    };
    const handleKeyPress = (event: KeyboardEvent) => {
      if (
        [
          "ArrowUp",
          "ArrowDown",
          "ArrowLeft",
          "ArrowRight",
          "w",
          "a",
          "s",
          "d",
          "l",
          "t",
          "e",
          "f",
        ].includes(event.key)
      ) {
        callback(keyBoardMapping[event.key]);
      }
    };

    window.addEventListener("keydown", handleKeyPress);

    // Cleanup function to remove the event listener when the component unmounts
    return () => {
      window.removeEventListener("keydown", handleKeyPress);
    };
  }, [callback]); // Re-run the effect if the callback changes
}
