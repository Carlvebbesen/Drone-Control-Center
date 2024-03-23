"use client";
import { Button } from "@/components/ui/button";
import { AuthContext } from "@/context/authContext";
import { signInWithPopupCustom } from "@/lib/firebase/auth";
import { useContext } from "react";

export default function Home() {
  const user = useContext(AuthContext);
  return (
    <div>
      <p>Hei dett er en test</p>
      <Button onClick={() => signInWithPopupCustom()}>klikk meg!</Button>
      <h1>Brukeren: </h1>
      <div>{user?.user?.displayName}</div>
      <div>{user?.user?.email}</div>
      <div>{user?.user?.emailVerified}</div>
      <img
        width={300}
        height={300}
        alt="profilePic"
        src={user?.user?.photoURL ?? ""}
      />
    </div>
  );
}
