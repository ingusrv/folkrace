#!/usr/bin/env python3

from argparse import ArgumentParser
from multiprocessing import Process
from websockets.client import connect
from time import sleep, time
from enum import Enum
import asyncio
import json

# robota programmas kods
def robot_main():
    while True:
        print("running")
        sleep(1)



# pārējais kods kas atļaus robotam darboties
parser = ArgumentParser()
parser.add_argument("-a", "--address")
parser.add_argument("-p", "--port")
parser.add_argument("-t", "--token")
args = parser.parse_args()

class MessageType(Enum):
    status = 0
    start = 1
    stop = 2
    data = 3
    error = 4

ADDRESS = args.address
PORT = args.port
TOKEN = args.token
ALGORITHM_NAME = "test"
ALGORITHM_VERSION = "0.0.1"
robot_process = None
ws = None
program_start_time = 0
program_stop_time = 0

async def ws_connection():
        global ws
        ws = await connect(f"ws://{ADDRESS}:{PORT}/robot?token={TOKEN}")
        print("Robots savienots")
        try:
            while True:
                data = await ws.recv()
                msg = json.loads(data)
                print(f"Saņēmām ziņu: {msg}")

                # ws message apstrāde
                if msg["type"] == MessageType.start.value:
                    start_delay = msg["delay"]
                    print(f"Startējam robota programmu, starta delay: {start_delay} sekundes")
                    sleep(start_delay)
                    res = start_robot()
                    if res == True:
                        await ws.send(json.dumps({
                            "type": MessageType.start.value,
                            "message": "Programma startēta"
                        }))
                    else:
                        await ws.send(json.dumps({
                            "type": MessageType.error.value,
                            "message": "Programma jau startēta"
                        }))
                        continue

                elif msg["type"] == MessageType.stop.value:
                    print("Apstādinām robota programmu")
                    res = stop_robot()
                    if res == True:
                        await ws.send(json.dumps({
                            "type": MessageType.stop.value,
                            "message": "Programma apstādināta"
                        }))
                        await send_drive_data(ws)
                    else:
                        await ws.send(json.dumps({
                            "type": MessageType.error.value,
                            "message": "Programma jau apstādināta"
                        }))
                        continue
                elif msg["type"] == MessageType.data.value:
                    print("Serveris saņēma nosūtītos datus")
                elif msg["type"] == MessageType.error.value:
                    print("Notika kļūda sazinoties ar serveri")
                    print(msg["message"])
                else:
                    print((f"Nezināms ziņas tips: {msg['type']}"))
        except:
            print("ws closed")
            ws = None

async def send_drive_data(ws):
    elapsed_time = round(program_stop_time - program_start_time, 2)
    fps = [10, 15, 13, 12, 14, 15, 16]
    drive_data = [[0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]
    await ws.send(json.dumps({
        "type": MessageType.data.value,
        "algorithm": ALGORITHM_NAME,
        "version": ALGORITHM_VERSION,
        "fps": fps,
        "elapsedTime": elapsed_time,
        "data": drive_data,
    }))


def start_robot():
    print("starting")
    global robot_process
    if not robot_process == None:
        return False
    robot_process = Process(target=robot_main)
    robot_process.start()
    global program_start_time
    program_start_time = time()
    return True


def stop_robot():
    print("stopping")
    global robot_process
    if robot_process == None:
        return False
    robot_process.terminate()
    global program_stop_time
    program_stop_time = time()
    return True

# šo vajag testēt ar gpiozero
async def on_button_pressed():
    print("button pressed")
    global robot_process
    if robot_process == None:
        start_robot()
        if not ws == None:
            await ws.send(json.dumps({
                "type": MessageType.start.value,
                "message": "Programma startēta"
            }))
    else:
        stop_robot()
        if not ws == None:
            await ws.send(json.dumps({
                "type": MessageType.stop.value,
                "message": "Programma apstādināta"
            }))
        await send_drive_data(ws)


async def main():
    while True:
        try:
            ws_task = asyncio.create_task(ws_connection())
            await ws_task
        except:
            print("Savienojums ar serveri neizdevās")
            print("Mēģinām vēlreiz pēc 5 sekundēm...")
        await asyncio.sleep(5)


# sākums
asyncio.run(main())

