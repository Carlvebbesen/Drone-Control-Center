import LogoLink from "../LogoLink";
import navigation from "@/lib/navigation";
import Link from "next/link";
import { useRouter } from "next/router";
import { Bars3Icon } from "@heroicons/react/24/solid";
import { useEffect, useState } from "react";
import { MD_IN_PIXELS } from "@/utils/tailwind";
import { Sheet, SheetContent, SheetFooter, SheetTrigger } from "../ui/Sheet";
import { Button } from "../button";
import ProfileAvatar from "../ProfileAvatar";

const MobileNavigation = () => {
  const router = useRouter();
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    router.events.on("routeChangeComplete", () => setIsOpen(false));
  }, [isOpen, router]);

  const isSignedIn = true;

  useEffect(() => {
    // Close the sheet if viewport is resized to desktop
    const handleResize = () => {
      if (window.innerWidth > MD_IN_PIXELS) {
        setIsOpen(false);
      }
    };
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  });

  return (
    <div className="flex h-20 w-full items-center justify-between p-4 md:hidden">
      <div className="flex items-center justify-start">
        <LogoLink />
      </div>
      <Sheet
        open={isOpen}
        onOpenChange={() => {
          setIsOpen(!isOpen);
        }}
      >
        <SheetTrigger asChild>
          <Button variant="outline">
            <Bars3Icon className="w-4" />
          </Button>
        </SheetTrigger>
        <SheetContent className="flex w-10/12 flex-col items-center justify-center bg-slate-100">
          <div className="my-8 flex flex-col items-center gap-y-4">
            {navigation.map((item, i) => {
              const active = router.pathname === item.href;
              return (
                <Link href={item.href} key={i}>
                  <Button variant={active ? "subtle" : "link"}>
                    {item.label}
                  </Button>
                </Link>
              );
            })}
          </div>

          <SheetFooter className="flex-1">
            <div className="flex items-end justify-center">
              {isSignedIn && (
                <div className="flex items-center justify-center">
                  <ProfileAvatar />
                </div>
              )}
            </div>
          </SheetFooter>
        </SheetContent>
      </Sheet>
    </div>
  );
};

export default MobileNavigation;
