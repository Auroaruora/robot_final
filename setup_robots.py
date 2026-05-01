"""
setup_robots.py  —  one-shot pairing of Robot A and Robot B.

macOS rotates a Create 3's BLE UUID across reboots, so the Bluetooth
addresses baked into run_config_<RID>.json go stale. Run this script
before driving the robots:

  1. Scans for nearby Create 3 robots.
  2. Connects to all of them, then walks them one at a time: lights the
     robot up white and plays a short tune so you can see/hear which
     physical robot is talking right now.
  3. For each robot it asks:
        A   -> assign as Robot A (lights turn green)
        B   -> assign as Robot B (lights turn blue)
        anything else (n / skip / enter) -> leave it alone and move on
  4. Writes the chosen addresses into robot_info/run_config_<RID>.json
     so run_robot.py / run_both.py pick them up next launch.

Usage:  python setup_robots.py
"""

import asyncio, json, os, sys
from bleak import BleakScanner

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Create3

ROOT_SERVICE_UUID = "48c5d828-ac2a-442d-97a3-0c9822b04979"
SCAN_TIMEOUT = 10.0

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_INFO_DIR = os.path.join(HERE, "robot_info")


def load_run_config(rid):
    path = os.path.join(ROBOT_INFO_DIR, "run_config_" + rid + ".json")
    if not os.path.exists(path):
        sys.exit("Missing " + path + " — run `python compute_path.py` first.")
    with open(path) as f:
        return json.load(f)


def save_bluetooth_address(rid, address):
    path = os.path.join(ROBOT_INFO_DIR, "run_config_" + rid + ".json")
    with open(path) as f:
        cfg = json.load(f)
    cfg["bluetooth_address"] = address
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)


async def scan():
    print("\nScanning for Create 3 robots (" + str(int(SCAN_TIMEOUT)) + "s)...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT, return_adv=True)
    found = []
    for device, adv in devices.values():
        if ROOT_SERVICE_UUID in adv.service_uuids:
            found.append((device.name or "<unknown>", device.address))
    return found


def main():
    # Loaded only for the human-readable color names in console output;
    # the identification light is always green so "the green one" always
    # means "the one being prompted right now".
    cfg_a = load_run_config("A")
    cfg_b = load_run_config("B")
    name_a = cfg_a["color_name"]
    name_b = cfg_b["color_name"]

    # The SDK's Robot.__init__ calls asyncio.get_event_loop() and Robot.play()
    # later does loop.run_forever() on it, so we need one loop that lives
    # across the scan and the play. asyncio.run() would create+close a loop
    # for the scan and leave the main thread without one (Python 3.10+).
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    found = loop.run_until_complete(scan())
    if not found:
        sys.exit("No Create 3 robots found. Make sure they are on and in range.")

    print("\nFound " + str(len(found)) + " Create 3 robot(s):")
    for i, (name, addr) in enumerate(found):
        print("  " + str(i + 1) + ". " + name + "  " + addr)

    # One asyncio.Event per robot — robot N waits for gate N to be set
    # before lighting up. Robot N then sets gate N+1, so interaction is
    # strictly sequential even though all robots connect in parallel.
    n = len(found)
    gates = [asyncio.Event() for _ in range(n)]
    assignments = {"A": None, "B": None}
    robots = []

    for i, (name, addr) in enumerate(found):
        robot = Create3(Bluetooth(address=addr))
        robots.append(robot)

        @event(robot.when_play)
        async def play(robot, i=i, addr=addr, name=name):
            if i == 0:
                gates[0].set()
            await gates[i].wait()

            print("\n--- Robot " + str(i + 1) + "/" + str(n) +
                  ":  " + name + "  " + addr + " ---")
            # Light up green and play the tune so the human can tell which
            # physical robot we're talking to right now.
            await robot.set_lights_on_rgb(0, 255, 0)
            await robot.play_note(440, 0.25)
            await robot.play_note(660, 0.25)
            await robot.play_note(880, 0.5)
            await robot.stop_sound()

            loop = asyncio.get_event_loop()
            choice = await loop.run_in_executor(
                None, input, "Assign this robot? [A / B / anything else = skip]: "
            )
            choice = choice.strip().upper()

            if choice == "A":
                if assignments["A"] is not None:
                    print("  (overwriting previous A assignment)")
                assignments["A"] = addr
                print("  -> Robot A (" + name_a + ")")
            elif choice == "B":
                if assignments["B"] is not None:
                    print("  (overwriting previous B assignment)")
                assignments["B"] = addr
                print("  -> Robot B (" + name_b + ")")
            else:
                print("  -> skipped")

            # Turn the green light off before the next robot is prompted, so
            # only one robot is ever lit green at a time.
            await robot.set_lights_on_rgb(0, 0, 0)

            if i + 1 < n:
                gates[i + 1].set()
            else:
                # All robots done — break out of SDK's run_forever().
                loop.call_later(0.3, loop.stop)

    # Single .play() drives every registered robot (same pattern as
    # test_2_connection.py / multidrive.py). Blocks until loop.stop().
    robots[0].play()

    print("\nFinal assignment:")
    print("  A -> " + (assignments["A"] or "(unchanged)"))
    print("  B -> " + (assignments["B"] or "(unchanged)"))

    if assignments["A"]:
        save_bluetooth_address("A", assignments["A"])
    if assignments["B"]:
        save_bluetooth_address("B", assignments["B"])

    if assignments["A"] and assignments["B"]:
        print("\nSaved. You can now run:  python run_both.py")
    elif assignments["A"] or assignments["B"]:
        print("\nOnly one slot was updated; the other still has its previous address.")
    else:
        print("\nNothing was changed.")


if __name__ == "__main__":
    main()
