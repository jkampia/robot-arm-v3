# Read sensor data through a TCA9548A I2C multiplexer 

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover - depends on target hardware image
    from smbus import SMBus


TCA9548A_ADDRESS = 0x70
AS5600_ADDRESS = 0x36
AS5600_ANGLE_REGISTER = 0x0E
AS5600_RESOLUTION = 4096
DEFAULT_ENCODER_CHANNELS = (0, 1, 2, 3, 4)


@dataclass
class AS5600Reading:
    raw_angle: int
    degrees: float
    read_time_seconds: float


@dataclass
class EncoderReadings:
    step_counts: list[int | None]
    angles_degrees: list[float | None]
    read_time_seconds: float
    errors: list[str | None]


class TCA9548A:
    def __init__(self, bus: SMBus, address: int = TCA9548A_ADDRESS):
        self.bus = bus
        self.address = address

    def select_channel(self, channel: int) -> None:
        if not 0 <= channel <= 7:
            raise ValueError("TCA9548A channel must be between 0 and 7")

        self.bus.write_byte(self.address, 1 << channel)

    def disable_all_channels(self) -> None:
        self.bus.write_byte(self.address, 0x00)


class AS5600:
    def __init__(self, bus: SMBus, address: int = AS5600_ADDRESS):
        self.bus = bus
        self.address = address

    def read_angle(self) -> AS5600Reading:
        start_time = time.perf_counter()
        high_byte = self.bus.read_byte_data(self.address, AS5600_ANGLE_REGISTER)
        low_byte = self.bus.read_byte_data(self.address, AS5600_ANGLE_REGISTER + 1)
        read_time_seconds = time.perf_counter() - start_time

        raw_angle = ((high_byte & 0x0F) << 8) | low_byte
        degrees = raw_angle * 360.0 / AS5600_RESOLUTION

        return AS5600Reading(
            raw_angle=raw_angle,
            degrees=degrees,
            read_time_seconds=read_time_seconds,
        )


def read_encoder_angle(
    bus_number: int = 1,
    mux_channel: int = 0,
    mux_address: int = TCA9548A_ADDRESS,
    encoder_address: int = AS5600_ADDRESS,
    mux_settle_seconds: float = 0.001,
) -> AS5600Reading:
    with SMBus(bus_number) as bus:
        mux = TCA9548A(bus, mux_address)
        mux.select_channel(mux_channel)
        time.sleep(mux_settle_seconds)

        encoder = AS5600(bus, encoder_address)
        return encoder.read_angle()


def read_encoder_angles(
    bus_number: int = 1,
    mux_channels: tuple[int, ...] = DEFAULT_ENCODER_CHANNELS,
    mux_address: int = TCA9548A_ADDRESS,
    encoder_address: int = AS5600_ADDRESS,
    mux_settle_seconds: float = 0.001,
    retries: int = 0,
) -> EncoderReadings:
    with SMBus(bus_number) as bus:
        mux = TCA9548A(bus, mux_address)
        encoder = AS5600(bus, encoder_address)
        return read_encoder_angles_on_bus(
            mux,
            encoder,
            mux_channels,
            mux_settle_seconds=mux_settle_seconds,
            retries=retries,
        )


def read_encoder_angles_on_bus(
    mux: TCA9548A,
    encoder: AS5600,
    mux_channels: tuple[int, ...] = DEFAULT_ENCODER_CHANNELS,
    mux_settle_seconds: float = 0.001,
    retries: int = 0,
) -> EncoderReadings:
    start_time = time.perf_counter()
    step_counts = []
    angles_degrees = []
    errors = []

    for channel in mux_channels:
        reading = None
        last_error = None

        for attempt in range(retries + 1):
            try:
                mux.select_channel(channel)
                time.sleep(mux_settle_seconds)
                reading = encoder.read_angle()
                break
            except OSError as error:
                last_error = error
                if attempt < retries:
                    time.sleep(mux_settle_seconds)

        if reading is None:
            step_counts.append(None)
            angles_degrees.append(None)
            errors.append(f"channel {channel}: {last_error}")
        else:
            step_counts.append(reading.raw_angle)
            angles_degrees.append(reading.degrees)
            errors.append(None)

    return EncoderReadings(
        step_counts=step_counts,
        angles_degrees=angles_degrees,
        read_time_seconds=time.perf_counter() - start_time,
        errors=errors,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read five AS5600 magnetic encoders on TCA9548A channels 0-4."
    )
    parser.add_argument("--bus", type=int, default=1, help="Linux I2C bus number")
    parser.add_argument(
        "--channels",
        default="0,1,2,3,4",
        help="Comma-separated TCA9548A channels to read in order",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Seconds between reads when streaming",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read once and exit instead of streaming continuously",
    )
    parser.add_argument(
        "--mux-settle",
        type=float,
        default=0.001,
        help="Seconds to wait after switching mux channels",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=0,
        help="Retries per encoder channel after an I2C error",
    )
    args = parser.parse_args()
    channels = tuple(int(channel.strip()) for channel in args.channels.split(","))

    with SMBus(args.bus) as bus:
        mux = TCA9548A(bus)
        encoder = AS5600(bus)
        is_first_read = True

        while True:
            readings = read_encoder_angles_on_bus(
                mux,
                encoder,
                channels,
                mux_settle_seconds=args.mux_settle,
                retries=args.retries,
            )
            angles = [
                None if angle is None else round(angle, 3)
                for angle in readings.angles_degrees
            ]
            errors = [error for error in readings.errors if error]
            if is_first_read and errors:
                print(f"startup_errors={errors}", flush=True)
            is_first_read = False

            print(
                f"steps={readings.step_counts} "
                f"angles={angles} "
                f"total_read_time={readings.read_time_seconds * 1_000_000.0:9.1f} us",
                flush=True,
            )

            if args.once:
                break

            time.sleep(max(args.interval, 0.0))


if __name__ == "__main__":
    main()
