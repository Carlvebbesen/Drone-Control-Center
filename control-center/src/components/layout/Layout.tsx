import Head from "next/head";
import React from "react";
import Footer from "./Footer";
import Navigation from "../navigation/Navigation";

type Props = {
  children: React.ReactNode;
};

const Layout = ({ children }: Props) => {
  return (
    <>
      <Head>
        <title>Internarena</title>
        <meta name="description" content="Internarena" />
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <Navigation />

      <main className="flex h-full flex-col items-center">{children}</main>
      <Footer />
    </>
  );
};

export default Layout;
