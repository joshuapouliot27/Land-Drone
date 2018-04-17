import websocket, json

def on_message(ws, message):
    print(message)

ws = websocket.WebSocket()
ws.connect("ws://raspberrypi.local:8081", on_message=on_message)
ws.run_forever()
