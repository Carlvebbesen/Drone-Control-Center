import { GoogleAuthProvider, signInWithPopup } from "firebase/auth";
import { authInstance, firebaseInstance } from "./config";

export async function signInWithPopupCustom() {
  const provider = new GoogleAuthProvider();

  try {
    const result = await signInWithPopup(authInstance, provider);
    console.log(result);

    return result;
  } catch (error) {
    console.error("Error signing in with Google", error);
  }
}

export async function signOut() {
  try {
    return authInstance.signOut();
  } catch (error) {
    console.error("Error signing out with Google", error);
  }
}
