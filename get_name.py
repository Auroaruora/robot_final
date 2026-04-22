import asyncio
from bleak import BleakScanner

async def scan():
    ROOT_ID = "48c5d828-ac2a-442d-97a3-0c9822b04979"
    devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
    for device, adv in devices.values():
        if ROOT_ID in adv.service_uuids:
            print(f"Found Root/Create3 -> name: {device.name!r}, address: {device.address}")

asyncio.run(scan())