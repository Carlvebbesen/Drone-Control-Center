import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "../ui/dropdown-menu";
import { Avatar, AvatarImage, AvatarFallback } from "@radix-ui/react-avatar";
import { useContext } from "react";
import { AuthContext } from "@/context/authContext";
import { signOut } from "@/lib/firebase/auth";

const ProfileAvatar = () => {
  const { user } = useContext(AuthContext);
  const getInitials = () => {
    const nameList = user?.displayName?.split(" ") ?? [""];
    if (nameList.length === 0) {
      return "Error";
    }
    if (nameList.length === 1) {
      if (nameList[0].length > 1) {
        return nameList[0].slice(0, 2).toUpperCase();
      }
      return nameList[0].toUpperCase();
    }
    return `${nameList[0][0].toUpperCase()}${nameList[nameList.length - 1][0].toUpperCase()}`;
  };
  return (
    <DropdownMenu>
      <DropdownMenuTrigger>
        <Avatar>
          <AvatarImage
            className="rounded-full w-16"
            src={user?.photoURL ?? undefined}
            alt={`Avatar of ${user?.displayName ?? "user"}`}
          />
          <AvatarFallback>{getInitials()}</AvatarFallback>
        </Avatar>
      </DropdownMenuTrigger>
      <DropdownMenuContent>
        {/* <Link href={`/ansatte/${authId}`}>
          <DropdownMenuItem>Min profil</DropdownMenuItem>
        </Link> */}
        <DropdownMenuSeparator />

        <DropdownMenuItem
          onClick={async () => {
            await signOut();
          }}
        >
          Logg ut
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );
};

export default ProfileAvatar;
