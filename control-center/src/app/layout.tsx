import { AuthContextProvider } from "@/context/authContext";
import "./globals.css";
import type { Metadata } from "next";
import Navigation from "@/components/navigation/Navigation";
import DesktopNavigation from "@/components/navigation/DesktopNavigation";
import Script from "next/script";

export const metadata: Metadata = {
  title: "Drone Control Center",
  description: "Drone control center for the tello inspection drones",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body>
        <AuthContextProvider>
          <DesktopNavigation />
          <main className="p-6 flex justify-center">{children}</main>
        </AuthContextProvider>
      </body>
    </html>
  );
}
