"use client";
import { authInstance } from "@/lib/firebase/config";
import { User, onAuthStateChanged } from "firebase/auth";
import { createContext, useEffect, useState } from "react";

export interface AuthContextType {
  user: User | null;
  logOut: () => void;
  isSignedIn: boolean;
}

export const AuthContext = createContext<AuthContextType>({
  user: null,
  logOut: () => {},
  isSignedIn: false,
});
export const AuthContextProvider = ({
  children,
}: {
  children: React.ReactNode;
}) => {
  const [user, setUser] = useState<User | null>(null);
  useEffect(() => {
    const unsubscribe = onAuthStateChanged(authInstance, (user) => {
      setUser(user);
    });
    return () => unsubscribe();
  }, []);

  return (
    <AuthContext.Provider
      value={{
        user: user,
        logOut: () => setUser(null),
        isSignedIn: !!user,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};
