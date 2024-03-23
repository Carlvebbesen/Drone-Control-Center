import Image from "next/image";

export const TelloDrone = ({ size }: { size: number }) => {
  return (
    <Image
      src={"/telloDrone.webp"}
      height={size}
      width={size}
      alt="tello drone"
    />
  );
};
