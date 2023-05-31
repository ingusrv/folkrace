#!/usr/bin/env python
from argparse import ArgumentParser
import asyncio
from websockets.sync.client import connect
import json
from time import sleep, time

parser = ArgumentParser()
parser.add_argument("-a", "--address")
parser.add_argument("-p", "--port")
parser.add_argument("-k", "--key")
args = parser.parse_args()

ADDRESS = args.address
PORT = args.port
KEY = args.key
ALGORITHM_NAME = None
ALGORITHM_VERSION = "0.0.1"
robotId = None
program_started = False

# šis notiks vietā kur palaiž algoritmu
start_time = None
stop_time = None


def main():
    with connect(f"ws://{ADDRESS}:{PORT}/api/v1/robot") as ws:
        ws.send(json.dumps({"type": "connect", "key": KEY}))
        message = json.loads(ws.recv())
        if message["type"] == "connect":
            if message["code"] == 200:
                robotId = message["robotId"]
                print(f"Robots autorizēts! Nosaukums: {robotId}")
            else:
                raise Exception(message["message"])
        else:
            raise Exception(
                f"Serveris atbildēja ar neatbilstošu ziņas tipu: {message['type']}")

        # robots tagad ir autorizēts un gatavs darbam
        print("Robots gatavs darbam")
        while True:
            global program_started
            message = json.loads(ws.recv())
            print(f"DEBUG: {message}")

            # ws message apstrāde
            if message["type"] == "start":
                if program_started == True:
                    ws.send(json.dumps(
                        {"code": 500, "type": "start", "message": "Programma jau startēta", "key": KEY}))
                    continue

                start_delay = message["delay"]
                print(
                    f"Startējam robota programmu, starta delay: {start_delay} sekundes")
                # šis notiks vietā kur palaiž algoritmu
                sleep(start_delay)
                start_time = time()

                program_started = True
                ws.send(json.dumps(
                        {"code": 200, "type": "start", "message": "Programma veiksmīgi startēta", "key": KEY}))
            elif message["type"] == "stop":
                if program_started == False:
                    ws.send(json.dumps(
                        {"code": 500, "type": "stop", "message": "Programma jau apstādināta", "key": KEY}))
                    continue

                print("Apstādinam robota programmu")

                # šo mēs dabūjam no algoritma daļas
                stop_time = time()
                elapsed_time = stop_time - start_time
                fps = [10, 15, 13, 12, 14, 15, 16]
                drive_data = [[0, 1, 0], [0, 1, 0], [0, 1, 0],
                              [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]]

                program_started == False
                ws.send(json.dumps(
                        {"code": 200, "type": "stop", "message": "Programma veiksmīgi apstādināta", "key": KEY}))

                sleep(2)

                print("Sūtam brauciena datus...")
                ws.send(json.dumps({
                    "type": "data",
                    "algorithm": ALGORITHM_NAME,
                    "version": ALGORITHM_VERSION,
                    "fps": fps,
                    "elapsedTime": elapsed_time,
                    "data": drive_data,
                    "key": KEY
                }))
            elif message["type"] == "data":
                if message["code"] == 200:
                    print(f"Robota dati veiksmīgi nosūtīti")
                else:
                    raise Exception(message["message"])
            else:
                print((
                    f"Nezināms ziņas tips: {message['type']}"))


main()
