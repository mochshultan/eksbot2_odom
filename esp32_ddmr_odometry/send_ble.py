# send_ble.py
import asyncio
import sys
from bleak import BleakScanner, BleakClient

# Ganti dengan UUID karakteristik yang benar di perangkatmu
CHAR_UUID = "00002a52-0000-1000-8000-00805f9b34fb"  # ⚠️ SESUAIKAN!

async def main(message):
    print("Mencari perangkat BLE...")
    device = await BleakScanner.find_device_by_address("88:57:21:B6:50:56", timeout=10.0)
    if not device:
        print("Perangkat tidak ditemukan!")
        return

    print(f"Terhubung ke {device.name or device.address}")
    async with BleakClient(device) as client:
        await client.write_gatt_char(CHAR_UUID, message.encode(), response=True)
    print("Pesan terkirim!")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python send_ble.py <message>")
        sys.exit(1)
    asyncio.run(main(sys.argv[1]))
