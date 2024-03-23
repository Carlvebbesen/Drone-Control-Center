"use client";

import {
  useReactTable,
  getCoreRowModel,
  getPaginationRowModel,
  getSortedRowModel,
  getFilteredRowModel,
} from "@tanstack/react-table";
import { DataTable } from "./dataTable";
import { InspectionColumns, columns } from "./inspectionColumns";

// export interface Detensions {
//   id: string;
//   description: string;
//   location: string;
// }

// export interface Inspections {
//   id: string;
//   type: string;
//   detentions: Detensions[];
//   name: number;
//   status: "error" | "success" | "onGoing";
//   launchType: "planlagt" | "event" | "manuell";
//   scheduleType: "Daglig" | "ukentlig" | "månedlig" | "kvartalsvis" | undefined;
//   time: Date;
//   elapsedTime: string;
//   droneId: string;
// }

const InspectionTable = () => {
  const inspections = [
    {
      id: "eef1e3fb-a09a-41c9-9b25-47a3d16bec96",
      name: 0,
      status: "onGoing",
      launchType: "planlagt",
      scheduleType: "månedlig",
      type: "vegger",
      detentions: [
        {
          id: "816330d8-8fe7-4fbd-8f1d-4cbf6ea05177",
          description:
            "do reprehenderit id ex Lorem esse deserunt enim commodo in sit dolore excepteur amet cillum",
          location: "1519804409841",
        },
        {
          id: "2cd23e1b-a550-4298-9507-6aa5b7bb1be3",
          description:
            "in ad culpa irure quis reprehenderit sunt anim velit sit cupidatat amet aute eu voluptate",
          location: "939102560438",
        },
        {
          id: "57811764-ef86-474e-9260-5b065bd940db",
          description:
            "cupidatat laborum enim et nostrud non Lorem pariatur non excepteur do sunt minim elit duis",
          location: "426140949493",
        },
        {
          id: "48688a93-dbf6-4128-ad17-297542f6bd32",
          description:
            "ea et velit culpa deserunt duis sint aliqua et sit pariatur nostrud adipisicing deserunt magna",
          location: "567534512761",
        },
        {
          id: "b520db00-5985-443e-8a3b-5079f7e2fc1f",
          description:
            "ullamco dolore pariatur mollit laboris nisi duis voluptate esse occaecat incididunt occaecat ex mollit aliquip",
          location: "1069506510871",
        },
        {
          id: "87f695bd-f877-421f-a1f4-ac0c452fb40c",
          description:
            "eu enim mollit eiusmod quis dolor in velit ipsum cillum sit culpa occaecat culpa eu",
          location: "1336290191706",
        },
        {
          id: "96306f9e-85cd-4756-b430-596798e9bbc9",
          description:
            "qui eu veniam ad id nulla eiusmod sint fugiat consequat id commodo officia esse tempor",
          location: "1312131032950",
        },
        {
          id: "8f3faca1-afc9-4b15-b0dd-b45962a5e94e",
          description:
            "fugiat laboris ullamco dolore occaecat labore veniam consequat eu sint enim voluptate laborum amet cillum",
          location: "1246948179642",
        },
        {
          id: "ae152839-6d93-4993-af9b-e33e3b3e7718",
          description:
            "in eu aute aute ut quis et ad eiusmod pariatur culpa exercitation culpa ea velit",
          location: "58659606734",
        },
        {
          id: "7dc9316d-f3b8-4c38-a31d-03b595dbe43f",
          description:
            "consectetur duis pariatur quis eiusmod proident commodo proident qui exercitation commodo do occaecat sit incididunt",
          location: "287285341672",
        },
      ],
      time: "2008-09-07T21:51:59.705Z",
      droneId: "Drone10",
      elapsedTime: "17:40",
    },
    {
      id: "96d3932e-683c-4660-b3a4-4fa1b986308d",
      name: 1,
      status: "success",
      launchType: "manuell",
      scheduleType: "månedlig",
      type: "tak",
      detentions: [],
      time: "1985-11-10T20:43:26.141Z",
      droneId: "Drone10",
      elapsedTime: "25:29",
    },
    {
      id: "8b6fca21-dc9c-4b95-a1c7-1b8a037f2e47",
      name: 2,
      status: "onGoing",
      launchType: "manuell",
      scheduleType: "ukentlig",
      type: "rom",
      detentions: [],
      time: "2021-04-04T14:51:39.802Z",
      droneId: "Drone10",
      elapsedTime: "53:33",
    },
    {
      id: "d01358b9-af3d-4b10-ae38-4d264d659946",
      name: 3,
      status: "error",
      launchType: "planlagt",
      scheduleType: "månedlig",
      type: "rengjøring",
      detentions: [
        {
          id: "2c196354-8122-43dc-ab19-5522ba734dc4",
          description:
            "et proident mollit pariatur ad amet anim proident fugiat adipisicing occaecat tempor occaecat est in",
          location: "865498404842",
        },
        {
          id: "5ef1f4ae-14e0-40f2-9328-e5c5884c083e",
          description:
            "fugiat nisi culpa adipisicing nostrud amet velit cillum proident sint aliqua consequat consectetur sunt ut",
          location: "1232376333820",
        },
        {
          id: "cfba6f99-805c-43d5-abe0-a4b42766ca59",
          description:
            "ut nulla est deserunt consequat excepteur sunt proident aute incididunt ullamco ad nulla sint id",
          location: "991551575864",
        },
        {
          id: "46c1d2dd-fab6-456a-b475-28c54dd10fb8",
          description:
            "incididunt dolore adipisicing aliquip esse tempor cillum ut nulla duis eiusmod Lorem mollit occaecat ea",
          location: "177798513980",
        },
        {
          id: "22dbd846-77b6-4296-b4be-8a99d4d82400",
          description:
            "ex culpa est aliquip do officia in proident adipisicing exercitation commodo ipsum incididunt laborum ullamco",
          location: "1401631833304",
        },
        {
          id: "5564bbfb-6c3a-4f14-884e-3e2053fbba35",
          description:
            "proident Lorem do fugiat enim pariatur tempor tempor nulla elit voluptate ipsum aliquip qui incididunt",
          location: "466506107929",
        },
      ],
      time: "2020-11-20T23:58:01.647Z",
      droneId: "Drone1",
      elapsedTime: "21:42",
    },
    {
      id: "1a5951a9-8e69-4572-a451-6c5c91dca7f0",
      name: 4,
      status: "success",
      launchType: "event",
      scheduleType: "månedlig",
      type: "rengjøring",
      detentions: [
        {
          id: "0e034eca-f4f2-4b55-ae41-01c12e6a2c04",
          description:
            "est velit in laborum exercitation enim sunt consequat eiusmod irure qui dolore labore aute consectetur",
          location: "713254484486",
        },
        {
          id: "6ab71cec-7d3f-4614-bcc4-d13eac88caec",
          description:
            "minim voluptate consectetur sit labore nostrud sunt est labore esse anim qui eiusmod aliqua magna",
          location: "80729294062",
        },
        {
          id: "8eb64cb4-9a1c-4712-9cba-fba7f30d895a",
          description:
            "sint occaecat exercitation incididunt ex laborum eu dolor magna aliqua deserunt quis magna elit consectetur",
          location: "686288064341",
        },
        {
          id: "b8ab220d-72e4-4305-9001-be4bc8e1d9b3",
          description:
            "ea veniam veniam nulla mollit proident ut occaecat cupidatat quis aliqua pariatur elit sit nisi",
          location: "1636194311532",
        },
      ],
      time: "1988-08-23T19:28:12.622Z",
      droneId: "Drone1",
      elapsedTime: "33:44",
    },
    {
      id: "08e8a4fa-3da3-41ce-a951-ecd88b55e3ad",
      name: 5,
      status: "onGoing",
      launchType: "planlagt",
      scheduleType: "Daglig",
      type: "gulv",
      detentions: [
        {
          id: "5fe1de60-e64b-4701-92fd-a04f115454e1",
          description:
            "laboris dolore consequat excepteur magna non in non pariatur fugiat cillum enim ad tempor laborum",
          location: "93540558136",
        },
      ],
      time: "1987-08-25T15:34:27.271Z",
      droneId: "Drone1",
      elapsedTime: "48:31",
    },
    {
      id: "002893a0-83c3-41b8-b24e-12da73a12f38",
      name: 6,
      status: "success",
      launchType: "manuell",
      scheduleType: "månedlig",
      type: "gulv",
      detentions: [
        {
          id: "5b0cd708-2f4b-468a-8878-7fa873010caa",
          description:
            "excepteur magna cupidatat non nulla proident ex magna aute do cillum culpa esse sit quis",
          location: "806820677356",
        },
      ],
      time: "2023-05-30T12:43:40.236Z",
      droneId: "Drone1",
      elapsedTime: "15:0",
    },
    {
      id: "5d998357-5541-4df9-b934-0d35fef96753",
      name: 7,
      status: "onGoing",
      launchType: "planlagt",
      scheduleType: "kvartalsvis",
      type: "rom",
      detentions: [
        {
          id: "a3065632-d4b3-4c25-8ac6-fd3cb2028d53",
          description:
            "ut eiusmod amet consectetur dolore elit nulla deserunt veniam nostrud tempor culpa non eu esse",
          location: "1083493001982",
        },
        {
          id: "26ec0929-6fef-456c-aa2c-5f1cc9f021ed",
          description:
            "sit adipisicing est culpa veniam enim commodo adipisicing adipisicing est sint non enim culpa id",
          location: "1509478086946",
        },
        {
          id: "453959bd-415c-4952-ac81-fb82c4566f3c",
          description:
            "reprehenderit et proident tempor eiusmod voluptate consequat aute veniam adipisicing exercitation consequat nisi voluptate veniam",
          location: "1210175472204",
        },
        {
          id: "bae304c7-2db2-4a3e-bc1b-d6f34af539e1",
          description:
            "do do dolore in ipsum labore incididunt incididunt ex occaecat laboris velit minim cillum est",
          location: "872527264092",
        },
        {
          id: "d9c0f27c-1a4d-4fea-93be-f45512406088",
          description:
            "magna esse consequat ullamco esse enim quis labore culpa cillum do officia elit irure ut",
          location: "86647634486",
        },
        {
          id: "d0fb474c-7512-4b13-b548-21c43c2402c4",
          description:
            "eu dolor Lorem enim et Lorem mollit anim ut ex eiusmod id sint ea enim",
          location: "522878766841",
        },
        {
          id: "a4a57369-2ab1-4acf-bae9-47cbffc3981f",
          description:
            "excepteur elit non Lorem non magna et qui reprehenderit nostrud ut officia aliquip ea esse",
          location: "991015048205",
        },
      ],
      time: "2022-07-13T01:10:33.871Z",
      droneId: "Drone1",
      elapsedTime: "24:51",
    },
    {
      id: "2f03dbb1-699d-4647-bdcc-3a863fffae56",
      name: 8,
      status: "onGoing",
      launchType: "event",
      scheduleType: "Daglig",
      type: "inspeksjon",
      detentions: [
        {
          id: "2fdec04f-245a-4ebb-aa82-109b7bb6c571",
          description:
            "anim cupidatat ad eiusmod culpa irure magna cupidatat deserunt sint eu eu excepteur in do",
          location: "869529098784",
        },
        {
          id: "b98628ed-b455-4ca8-9c78-6aff0f6d456a",
          description:
            "nisi veniam aute dolore aliqua ipsum officia incididunt amet aliqua pariatur occaecat id velit irure",
          location: "1408091944250",
        },
        {
          id: "7e4691c0-5dcb-4b98-89b3-14779c75daec",
          description:
            "amet cupidatat nulla excepteur culpa in anim magna do voluptate sint incididunt qui consequat velit",
          location: "1267525280169",
        },
        {
          id: "ac9079f9-e628-4cdf-87a8-d7119bee1c9c",
          description:
            "sunt esse excepteur duis enim labore magna minim aliqua aute enim labore tempor nostrud ex",
          location: "81353042357",
        },
        {
          id: "416fe574-0010-4f78-9d5a-e95c660739fe",
          description:
            "eu adipisicing enim nisi velit amet enim officia quis non exercitation exercitation duis nisi quis",
          location: "553717319932",
        },
        {
          id: "82d72196-e3f7-4946-8e39-0432043a7be7",
          description:
            "voluptate anim deserunt id magna cillum voluptate occaecat anim consequat deserunt quis dolore dolor elit",
          location: "1354123613086",
        },
        {
          id: "706b95e0-cec3-40cf-93a0-297b9de32e56",
          description:
            "id duis nulla consectetur commodo nulla deserunt commodo ullamco nostrud deserunt cillum exercitation proident ex",
          location: "1258762479842",
        },
      ],
      time: "2020-03-24T16:43:27.349Z",
      droneId: "Drone10",
      elapsedTime: "15:39",
    },
    {
      id: "aa71f044-51e6-444e-b19f-701547c6a817",
      name: 9,
      status: "success",
      launchType: "manuell",
      scheduleType: "månedlig",
      type: "nødutganger",
      detentions: [
        {
          id: "fc50a860-d34c-40fc-9267-e1cd2b4aa5bd",
          description:
            "deserunt esse amet tempor in ullamco nostrud ullamco duis adipisicing exercitation nostrud consequat ullamco commodo",
          location: "1124401108164",
        },
        {
          id: "beb9c1fc-a698-4089-acc0-6d55b81f01ac",
          description:
            "aute nisi ex exercitation veniam exercitation nulla cillum non elit amet minim irure ipsum veniam",
          location: "1046674644367",
        },
        {
          id: "b7095d0b-b866-459f-8050-4ae8d69149ec",
          description:
            "mollit labore mollit aliqua ex magna ex duis et dolor sint minim velit ipsum veniam",
          location: "863898308481",
        },
        {
          id: "f447521d-770b-488d-8bdd-f76f8f981458",
          description:
            "voluptate non mollit magna sunt adipisicing ut do ipsum ipsum sunt elit nostrud eu eu",
          location: "585996260183",
        },
        {
          id: "d6ba5c69-903c-410a-b2fa-e496dd61b01b",
          description:
            "duis consectetur velit consequat velit duis proident consectetur enim veniam sint ullamco elit proident laboris",
          location: "288316409029",
        },
        {
          id: "5c91c28a-9f45-4c69-b453-86c91a95204f",
          description:
            "ullamco consectetur adipisicing pariatur aliqua dolor occaecat nostrud nulla ex aliqua amet est exercitation enim",
          location: "1173748825690",
        },
        {
          id: "fda9be8b-f97b-43f6-b822-ee0db8555802",
          description:
            "ut do ad do exercitation id velit enim amet consectetur dolore ipsum quis non quis",
          location: "1384451255808",
        },
        {
          id: "372bf2a5-3552-4977-9fd1-f7dde31bc145",
          description:
            "sunt ex laborum commodo adipisicing dolor sunt in excepteur officia voluptate aliquip anim aliqua exercitation",
          location: "1306418761666",
        },
        {
          id: "5caede22-a5cb-48be-9899-4a0e1f084832",
          description:
            "nostrud eiusmod do laboris enim qui voluptate veniam quis dolore aliquip nostrud voluptate in voluptate",
          location: "1020843820062",
        },
        {
          id: "476840c4-866e-4ba3-b937-1aa26f6867b4",
          description:
            "dolor sint commodo ullamco deserunt dolore quis aliqua do excepteur laborum ea tempor nostrud cillum",
          location: "1027472075464",
        },
      ],
      time: "2011-05-26T05:17:57.145Z",
      droneId: "Drone1",
      elapsedTime: "40:13",
    },
    {
      id: "809feb05-8921-447e-9f74-020b7dd2c54a",
      name: 10,
      status: "error",
      launchType: "event",
      scheduleType: "Daglig",
      type: "rengjøring",
      detentions: [
        {
          id: "e92cd95e-c8f6-4c7a-b400-9ada1c22341b",
          description:
            "magna occaecat reprehenderit elit exercitation cupidatat mollit nisi consequat esse aliquip nisi enim non sint",
          location: "1497775807974",
        },
        {
          id: "120b02ad-221f-40c6-98c6-f8530803539f",
          description:
            "id mollit deserunt ex aute ad ullamco ullamco ad est nulla aliqua aute labore culpa",
          location: "663403002323",
        },
        {
          id: "3a6717cc-1ddd-4088-9462-91fb72bb82ce",
          description:
            "adipisicing aliqua reprehenderit nulla commodo qui sunt qui reprehenderit Lorem nulla esse culpa voluptate ad",
          location: "996047833863",
        },
        {
          id: "7ce7347b-a829-411e-99fa-951b66409f53",
          description:
            "laborum reprehenderit deserunt aute esse occaecat culpa sint ad veniam amet reprehenderit adipisicing officia amet",
          location: "178814291301",
        },
        {
          id: "c0fdd0d6-9259-4d1f-925f-9e6e95e6cfca",
          description:
            "nisi minim cupidatat Lorem deserunt velit laborum id commodo adipisicing quis culpa esse aute sit",
          location: "1637366140197",
        },
        {
          id: "578bf35c-47f3-448c-baf7-10a2cd0fbea3",
          description:
            "laboris laboris eu id do amet nisi sit excepteur deserunt et Lorem tempor deserunt eu",
          location: "466196382399",
        },
      ],
      time: "2007-07-17T15:44:02.046Z",
      droneId: "Drone1",
      elapsedTime: "22:14",
    },
    {
      id: "8649ea83-f2a3-428c-b631-079dbf100b16",
      name: 11,
      status: "error",
      launchType: "event",
      scheduleType: "Daglig",
      type: "nødutganger",
      detentions: [
        {
          id: "601e6f92-09f9-4823-a011-5d9b142b604f",
          description:
            "ullamco exercitation exercitation consequat magna duis et amet ipsum aliquip occaecat aute adipisicing sint sit",
          location: "1356776262839",
        },
      ],
      time: "2016-05-18T15:48:54.813Z",
      droneId: "Drone1",
      elapsedTime: "14:31",
    },
    {
      id: "18a34b50-2842-4cbb-aa5a-6ce1103c0e0f",
      name: 12,
      status: "onGoing",
      launchType: "manuell",
      scheduleType: "månedlig",
      type: "inspeksjon",
      detentions: [
        {
          id: "0a826712-7f50-497c-9272-e26764326111",
          description:
            "dolore proident reprehenderit ad exercitation voluptate esse laboris tempor adipisicing proident occaecat do ut adipisicing",
          location: "91263860628",
        },
      ],
      time: "1985-08-30T22:02:53.866Z",
      droneId: "Drone10",
      elapsedTime: "44:29",
    },
    {
      id: "8de2928c-1e26-4247-974a-bd9d9d12cdaf",
      name: 13,
      status: "error",
      launchType: "event",
      scheduleType: "Daglig",
      type: "inspeksjon",
      detentions: [
        {
          id: "afbccc69-c7ef-4bb3-bd5e-9d0469249eff",
          description:
            "reprehenderit quis in deserunt aute do incididunt reprehenderit occaecat in id laborum et exercitation ut",
          location: "1290340543255",
        },
        {
          id: "68e3f2a5-2a54-4ff3-9ea0-334d27a082f4",
          description:
            "enim anim dolore pariatur incididunt aliquip aliquip proident sint nostrud deserunt aute mollit commodo nulla",
          location: "148768625939",
        },
        {
          id: "6675a75c-abde-41e6-9bb7-f581ef813c99",
          description:
            "duis officia aliqua ut duis labore Lorem aute cupidatat tempor voluptate qui ad cillum dolor",
          location: "1363747582275",
        },
        {
          id: "16ef56c5-f603-4b88-be5a-3bb89381fb68",
          description:
            "culpa dolor consequat tempor consectetur occaecat sint aute ex laboris est voluptate dolor sint do",
          location: "167114809386",
        },
        {
          id: "d5dfb3a7-3be5-4623-84e1-1feb373b727e",
          description:
            "ullamco duis nulla aliqua laboris non aute fugiat mollit ullamco nulla qui labore ad ea",
          location: "116950028758",
        },
        {
          id: "f6ed6fb8-eb24-413c-a717-97bee81e5ba5",
          description:
            "sit cupidatat elit laborum ad sint laboris nulla minim officia proident id irure in consequat",
          location: "848296778914",
        },
        {
          id: "72e6e91f-e14a-43c6-9ca5-527f1586e9a9",
          description:
            "enim nostrud id commodo mollit ex esse dolor ad officia amet est eiusmod velit enim",
          location: "383420514456",
        },
        {
          id: "9d496a0d-4d70-43b8-b5f9-4bb14f698484",
          description:
            "labore cupidatat enim esse cupidatat nulla minim commodo laboris eu laborum do incididunt cupidatat sit",
          location: "549092846602",
        },
      ],
      time: "1976-10-15T23:22:58.965Z",
      droneId: "Drone10",
      elapsedTime: "32:6",
    },
    {
      id: "dd1fd07b-4ae6-4bf1-92f0-564a7f9ce2a1",
      name: 14,
      status: "onGoing",
      launchType: "manuell",
      scheduleType: "kvartalsvis",
      type: "vegger",
      detentions: [
        {
          id: "39197bd3-34c9-48ba-9139-c193ecfcab5b",
          description:
            "qui ipsum elit mollit irure deserunt cillum sint ipsum minim nisi et mollit adipisicing nisi",
          location: "1663870762317",
        },
        {
          id: "6ac29185-2b49-4042-a8a6-d0679a0fe329",
          description:
            "consectetur amet mollit ex nulla ipsum esse dolore non voluptate adipisicing amet Lorem reprehenderit dolore",
          location: "232524967084",
        },
        {
          id: "84d2cf60-3926-409c-b26b-5e5a47a3569b",
          description:
            "aute irure magna excepteur aute eu sint Lorem pariatur do in amet laboris ex amet",
          location: "655789950182",
        },
        {
          id: "06e03ce1-b888-408b-9d46-1c27588e7e5e",
          description:
            "voluptate reprehenderit qui est tempor excepteur amet ea voluptate id excepteur id deserunt non fugiat",
          location: "509720754824",
        },
        {
          id: "0988a73b-bd43-4615-9abc-1bc7b600a56c",
          description:
            "eu aute minim fugiat ut ullamco ex consequat deserunt ea exercitation pariatur ullamco officia qui",
          location: "310942413250",
        },
        {
          id: "7d36d72d-cba0-4d8f-a290-e0364966ff1f",
          description:
            "incididunt ad in eu exercitation laborum culpa exercitation laborum excepteur commodo reprehenderit consequat eiusmod sunt",
          location: "1267456319078",
        },
      ],
      time: "1980-03-19T03:13:15.934Z",
      droneId: "Drone10",
      elapsedTime: "40:36",
    },
    {
      id: "ea7138b5-06f6-4ea9-b59f-be31da51c22c",
      name: 15,
      status: "success",
      launchType: "event",
      scheduleType: "kvartalsvis",
      type: "inspeksjon",
      detentions: [
        {
          id: "06ef9c54-e31e-4285-9759-8a044709f7da",
          description:
            "in veniam id aliquip in pariatur sint exercitation eiusmod magna labore incididunt elit non et",
          location: "1252649453915",
        },
        {
          id: "f46f5640-2474-4619-9afd-601f54de49fd",
          description:
            "ut magna culpa voluptate eu nisi deserunt laboris incididunt proident pariatur tempor quis aliqua ex",
          location: "1527944997309",
        },
        {
          id: "eabd1508-384b-42ba-bd31-d5e1beb0677c",
          description:
            "est voluptate reprehenderit Lorem anim culpa nisi pariatur deserunt nisi elit cillum ut culpa ipsum",
          location: "616249293539",
        },
      ],
      time: "2014-10-10T12:52:55.821Z",
      droneId: "Drone10",
      elapsedTime: "45:51",
    },
    {
      id: "96ee3ae1-77f5-48ef-a382-ae9f351b1ef2",
      name: 16,
      status: "error",
      launchType: "event",
      scheduleType: "ukentlig",
      type: "nødutganger",
      detentions: [
        {
          id: "426f0c0a-e4cb-4a38-af46-b90f64327702",
          description:
            "officia quis mollit officia incididunt qui velit ipsum nostrud pariatur aute nisi duis laboris voluptate",
          location: "769576348342",
        },
        {
          id: "fe39225b-9142-4df8-baa2-294e54df2ef1",
          description:
            "esse dolore voluptate ullamco exercitation exercitation est nulla ea Lorem aliqua esse culpa proident voluptate",
          location: "1437840890429",
        },
        {
          id: "19058341-356b-4cec-a620-4fabf411b1ac",
          description:
            "veniam irure sunt id aute eiusmod consectetur quis officia minim anim esse qui do consectetur",
          location: "1136740061628",
        },
        {
          id: "8b3d52fd-2a28-4dd2-b364-59c0b988a709",
          description:
            "pariatur incididunt dolore ipsum et laborum ipsum deserunt aliqua tempor id sit deserunt est proident",
          location: "481162488217",
        },
      ],
      time: "1990-09-10T03:07:38.870Z",
      droneId: "Drone10",
      elapsedTime: "28:59",
    },
    {
      id: "b6ae635f-718e-4177-a95f-479ae3fab6da",
      name: 17,
      status: "error",
      launchType: "event",
      scheduleType: "Daglig",
      type: "inspeksjon",
      detentions: [],
      time: "2014-07-07T00:24:53.370Z",
      droneId: "Drone10",
      elapsedTime: "33:16",
    },
    {
      id: "fba0d103-b133-432f-9054-6c5419481fb5",
      name: 18,
      status: "onGoing",
      launchType: "event",
      scheduleType: "kvartalsvis",
      type: "rengjøring",
      detentions: [
        {
          id: "36f966c2-edbe-4547-8fe3-e02e1c535405",
          description:
            "occaecat culpa aliqua deserunt qui tempor ut nulla culpa aliquip ea dolor reprehenderit in proident",
          location: "796868080211",
        },
        {
          id: "eca78d99-769f-4dad-ace1-af4f0c9e4e7f",
          description:
            "labore in nostrud ex exercitation enim Lorem incididunt nisi veniam sit culpa in mollit consectetur",
          location: "590246759289",
        },
        {
          id: "5f3c2b64-ad1b-4c9f-adc0-459226e93675",
          description:
            "dolor in pariatur adipisicing nostrud labore officia tempor tempor Lorem laboris sint deserunt sunt esse",
          location: "1030620373933",
        },
        {
          id: "b2a02363-fc51-474a-835e-2075926d4545",
          description:
            "Lorem ullamco adipisicing fugiat culpa esse consectetur id dolor enim nulla fugiat nisi aliquip do",
          location: "162727071364",
        },
        {
          id: "862f4725-35c1-4fa5-b162-7af2706c2f54",
          description:
            "proident esse exercitation do ut sit velit nulla adipisicing enim deserunt magna elit eu culpa",
          location: "1563063792009",
        },
      ],
      time: "2001-05-23T07:01:00.209Z",
      droneId: "Drone10",
      elapsedTime: "1:45",
    },
    {
      id: "68210dbd-7de8-4926-8440-5faa82222b8f",
      name: 19,
      status: "onGoing",
      launchType: "planlagt",
      scheduleType: "kvartalsvis",
      type: "rengjøring",
      detentions: [
        {
          id: "14daa3fe-59f7-4cc1-a907-cf9b2ca8593b",
          description:
            "magna dolore pariatur ex occaecat ut cupidatat do incididunt laborum dolore excepteur ullamco eu do",
          location: "1564237183372",
        },
        {
          id: "8677f1e6-d3c3-4f69-a5ef-376f904853dd",
          description:
            "culpa ut elit adipisicing nulla labore in ea veniam dolor Lorem elit dolor anim non",
          location: "963540504509",
        },
        {
          id: "261c268f-b702-4f9e-908e-c11f5d3e4241",
          description:
            "cupidatat ad voluptate irure incididunt eu esse irure elit ullamco in id irure tempor excepteur",
          location: "1477327599292",
        },
      ],
      time: "2003-11-19T04:18:30.962Z",
      droneId: "Drone10",
      elapsedTime: "60:51",
    },
    {
      id: "0f3ea4e0-f60d-4dc2-a7e4-2c9245896229",
      name: 20,
      status: "onGoing",
      launchType: "event",
      scheduleType: "månedlig",
      type: "rengjøring",
      detentions: [
        {
          id: "39943883-7f87-4788-81e3-95b681a53368",
          description:
            "consectetur velit mollit pariatur in pariatur reprehenderit mollit quis irure veniam consectetur excepteur esse veniam",
          location: "830455309271",
        },
        {
          id: "8f0101b4-2774-4df5-a20c-3d78d0a81732",
          description:
            "nulla magna nisi adipisicing ipsum et ex nostrud exercitation irure mollit ut cillum sunt sit",
          location: "434944632815",
        },
        {
          id: "274b4435-cd03-43bf-b3ac-b5e25e803dd4",
          description:
            "duis dolor officia id consequat est nostrud ipsum aute anim id duis minim nisi Lorem",
          location: "1665613115755",
        },
        {
          id: "99c35da9-4578-4bbb-965e-e55166c50a15",
          description:
            "nulla sunt exercitation laborum tempor aliquip consectetur irure officia ea reprehenderit esse proident enim Lorem",
          location: "1633242413093",
        },
        {
          id: "43e8330b-4ff6-43fc-9365-c444aa97dfd6",
          description:
            "minim anim culpa esse veniam ut duis consectetur sit anim deserunt ea consequat irure labore",
          location: "1415999441751",
        },
        {
          id: "90102ba3-cf54-4d68-a8f6-c63ed63992e9",
          description:
            "ex officia commodo pariatur amet incididunt est ad sunt commodo magna sit laboris eiusmod dolore",
          location: "33861195962",
        },
        {
          id: "519e1065-b7e3-4655-9a37-54f5e67dafe3",
          description:
            "qui ea cillum fugiat anim consequat minim qui ullamco Lorem veniam dolore excepteur ex id",
          location: "171358644397",
        },
        {
          id: "d11630a1-6d8b-4d5c-9c0b-29109059df14",
          description:
            "adipisicing ullamco duis excepteur proident occaecat laboris ad minim ut in magna sunt eu Lorem",
          location: "983413035838",
        },
        {
          id: "af99ffad-1275-410a-8d7b-69c1316c7f68",
          description:
            "reprehenderit deserunt ut qui cupidatat aliqua enim et mollit culpa culpa cillum enim ex aute",
          location: "133136936353",
        },
      ],
      time: "1995-10-16T01:38:19.806Z",
      droneId: "Drone10",
      elapsedTime: "2:29",
    },
    {
      id: "48148bbc-c46a-45de-a649-28610411620d",
      name: 21,
      status: "onGoing",
      launchType: "event",
      scheduleType: "kvartalsvis",
      type: "rom",
      detentions: [
        {
          id: "aa107d1d-6b22-4bf4-848f-c8fbfd6a6e17",
          description:
            "sit nulla dolore id exercitation incididunt ut voluptate ullamco proident ex fugiat minim laborum magna",
          location: "976680582619",
        },
        {
          id: "5b141e45-d3de-4f42-b8a0-424a38d9fcd8",
          description:
            "do proident ad voluptate reprehenderit veniam ut incididunt ullamco aute consequat deserunt elit est excepteur",
          location: "650507967158",
        },
        {
          id: "1ee4fb74-7727-41b0-a354-59584036f33e",
          description:
            "aliqua aliqua laborum tempor irure cupidatat eiusmod cupidatat voluptate elit cillum pariatur elit ipsum nisi",
          location: "199455255466",
        },
        {
          id: "3458a0db-af60-4344-b46f-568246103ecb",
          description:
            "ipsum cillum tempor in incididunt ut excepteur et officia proident aliquip velit ea et nostrud",
          location: "1017379110991",
        },
        {
          id: "e6982a60-1c66-476d-8b2b-1b912b96d762",
          description:
            "officia veniam eiusmod sit adipisicing eu sit voluptate magna id laboris veniam ad aliqua nulla",
          location: "361460744569",
        },
        {
          id: "da696e63-c2cd-43a2-a1fe-1dd6adced12f",
          description:
            "non deserunt nulla sint incididunt sit tempor Lorem commodo est consectetur duis eiusmod enim proident",
          location: "1049272696682",
        },
        {
          id: "368d428e-d618-4146-8b8a-4c77875e1bc2",
          description:
            "magna est ex ex occaecat nisi reprehenderit excepteur do laborum voluptate occaecat quis mollit ipsum",
          location: "700382637419",
        },
        {
          id: "200a2f35-5845-4a14-8d78-5f97c6208454",
          description:
            "consectetur excepteur sunt occaecat nostrud incididunt qui aliquip commodo enim ullamco reprehenderit amet et excepteur",
          location: "313493588821",
        },
      ],
      time: "2022-04-08T22:00:06.292Z",
      droneId: "Drone1",
      elapsedTime: "37:15",
    },
  ];
  const data: InspectionColumns[] = inspections.map((item) => {
    return { ...item, detensionCount: item.detentions.length };
  });

  const table = useReactTable({
    data: data || [],
    columns,
    getCoreRowModel: getCoreRowModel(),
  });
  return (
    <DataTable
      className="overflow-auto max-h-[500px]"
      columns={columns}
      table={table}
    />
  );
};

export default InspectionTable;
