"use client";
import Link from "next/link";
import { usePathname } from "next/navigation";
import {
  NavigationMenu,
  NavigationMenuItem,
  NavigationMenuList,
} from "../ui/navigation-menu";
import { useContext } from "react";
import { AuthContext } from "@/context/authContext";
import navigation from "@/lib/navigation";
import { Button } from "../ui/button";
import ProfileAvatar from "./profileAvatar";
import { DroneLogo } from "../logo/droneLogo";
import { signInWithPopupCustom } from "@/lib/firebase/auth";

const DesktopNavigation = () => {
  const pathname = usePathname();
  const { isSignedIn } = useContext(AuthContext);
  return (
    <NavigationMenu className="mt-3  px-6 min-w-full h-20 flex justify-between">
      <div className="flex items-center justify-start">
        <Link href="/">
          <DroneLogo />
        </Link>
      </div>

      {isSignedIn ? (
        <NavigationMenuList className="w-full gap-x-20 grow">
          {navigation.map((item, i) => {
            const active = pathname === item.href;

            return (
              <NavigationMenuItem key={i}>
                <Link href={item.href}>
                  <Button variant={active ? "outline" : "link"}>
                    {item.label}
                  </Button>
                </Link>
              </NavigationMenuItem>
            );
          })}
          <div className="flex items-center justify-center">
            {isSignedIn && (
              <div className="flex items-center justify-center">
                <NavigationMenuItem className="mr-6">
                  <ProfileAvatar />
                </NavigationMenuItem>
              </div>
            )}
          </div>
        </NavigationMenuList>
      ) : (
        <NavigationMenuItem key={"loginLink"}>
          <Button onClick={()=>signInWithPopupCustom()} variant={"outline"}>{"LogIn"}</Button>
        </NavigationMenuItem>
      )}
    </NavigationMenu>
  );
};
export default DesktopNavigation;
