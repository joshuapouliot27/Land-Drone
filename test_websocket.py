import time

import websocket, json


def on_message(ws, message):
    print(message)


ws = websocket.WebSocket()
ws.connect("ws://raspberrypi.local:8181", on_message=on_message)
while True:
    ws.send("return")
    time.sleep(2.5)
