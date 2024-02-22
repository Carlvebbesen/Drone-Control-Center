import socket
import websockets
import asyncio

async def websocket_test():
    uri = "ws://localhost:5683"
    async with websockets.connect(uri) as websocket:
        await websocket.send("Hello World!")
        greeting = await websocket.recv()
        print(greeting)

asyncio.run(websocket_test())
