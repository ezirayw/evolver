#!/usr/bin/env python3
import argparse
import struct
from enum import Enum


class SerialAddressMap(Enum):
    """Enum mapping names to serial addresses based on evolver_SAConfig.h"""

    ARDUINO_0 = 0  # reserved for Motherboard, used to sync arduino on current loop phase
    ARDUINO_1 = 1  # reserved, used to sync arduino on current loop phase
    ARDUINO_2 = 2  # reserved, used to sync arduino on current loop phase
    ARDUINO_3 = 3  # reserved, used to sync arduino on current loop phase
    ARDUINO_4 = 4
    OD_LED_LEFT = 5
    OD_LED_RIGHT = 6
    OD_90_LEFT = 7
    OD_90_RIGHT = 8
    TEMP = 9
    STIR = 10
    OVERFLOW_LEFT = 11
    OVERFLOW_RIGHT = 12
    EFFLUX_PUMPS = 13
    EFFLUX_PUMPS_CONFIG = 14


class CommandTags(Enum):
    """Enum mapping command types to values"""

    REQUEST = 0
    ACKNOWLEDGE = 1
    SENSOR = 2
    ECHO = 3
    CONFIG = 4


def cobs_encode(data):
    """
    Encodes data with Consistent Overhead Byte Stuffing (COBS).

    This is adapted from evolver_namespace_server.py's implementation.

    Args:
        data (bytes or bytearray): The data to encode.

    Returns:
        bytearray: The COBS-encoded data.
    """
    if not data:
        return bytearray(b"\x01\x00")

    # Start with an extra byte for the code
    result = bytearray()

    # Iterate through the data to find all zeros and encode
    code_index = 0
    code = 1

    # Add placeholder for first code byte
    result.append(0)

    for byte in data:
        if byte == 0:
            # Found a zero, write the code byte and reset
            result[code_index] = code
            code = 1
            code_index = len(result)
            result.append(0)  # Placeholder for next code byte
        else:
            # Non-zero byte, append it
            result.append(byte)
            code += 1
            # If the code reaches its maximum value, write it and start a new block
            if code == 0xFF:
                result[code_index] = code
                code = 1
                code_index = len(result)
                result.append(0)  # Placeholder for next code byte

    # Write the final code byte
    result[code_index] = code

    # Add the frame delimiter zero byte
    result.append(0)

    return result


def cobs_decode(data):
    """
    Decodes data encoded with Consistent Overhead Byte Stuffing (COBS).

    This is adapted from evolver_namespace_server.py's implementation.

    Args:
        data (bytes or bytearray): The COBS-encoded data to decode.

    Returns:
        bytearray: The decoded data.
    """
    if not data or data[-1] != 0:
        raise ValueError("Invalid COBS encoded data: missing zero delimiter")

    result = bytearray()
    i = 0

    while i < len(data) - 1:  # Skip the final zero delimiter
        code = data[i]
        i += 1

        if code == 0:
            raise ValueError("Invalid COBS encoded data: unexpected zero")

        for j in range(1, code):
            if i < len(data) - 1:  # Ensure we're not at the final delimiter
                result.append(data[i])
                i += 1

        if code < 0xFF and i < len(data) - 1:
            result.append(0)

    return result


def calculate_checksum(packet):
    """Calculate the checksum for a packet"""
    # Sum all bytes in the packet
    checksum = sum(packet)

    # Add carry bits back
    while checksum > 0xFF:
        checksum = (checksum & 0xFF) + (checksum >> 8)

    # Return the complement
    return 0xFF - checksum


def get_address():
    """Prompt the user for the address"""
    print("\nEnter the address (integer or name from SerialAddressMap):")
    for name, member in SerialAddressMap.__members__.items():
        print(f"  {name} = {member.value}")

    address_input = input("> ").strip().upper()

    try:
        # Try to parse as integer
        address = int(address_input)
        # Check if valid address
        if 0 <= address <= 255:
            return address
        else:
            print("Error: Address must be between 0 and 255")
            return get_address()
    except ValueError:
        # Try to parse as enum name
        try:
            address = SerialAddressMap[address_input].value
            return address
        except KeyError:
            print(f"Error: '{address_input}' is not a valid address name")
            return get_address()


def get_data_length():
    """Prompt the user for the data length"""
    print("\nEnter the data length (number of integers in the payload):")

    try:
        data_length = int(input("> ").strip())
        if data_length > 0:
            return data_length
        else:
            print("Error: Data length must be greater than 0")
            return get_data_length()
    except ValueError:
        print("Error: Please enter a valid integer")
        return get_data_length()


def get_command_type():
    """Prompt the user for the command type"""
    print("\nEnter the command type (integer or name from CommandTags):")
    for name, member in CommandTags.__members__.items():
        print(f"  {name} = {member.value}")

    type_input = input("> ").strip().upper()

    try:
        # Try to parse as integer
        cmd_type = int(type_input)
        # Check if valid command type
        if 0 <= cmd_type <= 4:
            return cmd_type
        else:
            print("Error: Command type must be between 0 and 4")
            return get_command_type()
    except ValueError:
        # Try to parse as enum name
        try:
            cmd_type = CommandTags[type_input].value
            return cmd_type
        except KeyError:
            print(f"Error: '{type_input}' is not a valid command type name")
            return get_command_type()


def get_data_payload(data_length):
    """Prompt the user for the data payload values"""
    print(f"\nEnter {data_length} integer values for the data payload (one per line):")

    data_payload = []
    for i in range(data_length):
        try:
            value = int(input(f"Value {i + 1}> ").strip())
            data_payload.append(value)
        except ValueError:
            print("Error: Please enter a valid integer")
            # Retry this value
            i -= 1

    return data_payload


def build_packet():
    """Build a packet based on user input"""
    # Get packet components from user
    address = get_address()
    data_length = get_data_length()
    cmd_type = get_command_type()
    data_payload = get_data_payload(data_length)

    # Build raw packet
    packet = bytearray()

    # Add address
    packet.append(address)

    # Add data length
    packet.append(data_length)

    # Add command type
    packet.append(cmd_type)

    # Add data payload (4 bytes per value, little endian)
    for value in data_payload:
        packet.extend(struct.pack("<I", value))

    # Calculate and add checksum
    checksum = calculate_checksum(packet)
    packet.append(checksum)

    # Encode with COBS
    encoded_packet = cobs_encode(packet)

    # Output the packet in multiple formats
    print("\n--- BUILD OUTPUT ---")
    print("\nRaw Packet (hex):")
    raw_hex = " ".join([f"0x{byte:02X}" for byte in packet])
    print(f"[{raw_hex}]")

    print("\nCOBS-encoded Packet (hex):")
    encoded_hex = " ".join([f"0x{byte:02X}" for byte in encoded_packet])
    print(f"[{encoded_hex}]")

    print("\nCopy-Paste Format (COBS-encoded):")
    copy_paste = "".join([f"{byte:02X}" for byte in encoded_packet])
    print(copy_paste)

    # If this was a request packet, also generate the corresponding acknowledge packet
    if cmd_type == CommandTags.REQUEST.value:
        print("\n=== Corresponding Acknowledgment Packet ===")
        # Create a copy of the packet but change the type to ACKNOWLEDGE
        ack_packet = bytearray(packet)
        ack_packet[2] = CommandTags.ACKNOWLEDGE.value  # Change type to ACKNOWLEDGE

        # Recalculate checksum
        # First, remove the old checksum
        ack_packet.pop()
        checksum = calculate_checksum(ack_packet)
        ack_packet.append(checksum)

        # Encode with COBS
        encoded_ack_packet = cobs_encode(ack_packet)

        # Output the acknowledgment packet in multiple formats
        print("\nRaw Acknowledgment Packet (hex):")
        raw_ack_hex = " ".join([f"0x{byte:02X}" for byte in ack_packet])
        print(f"[{raw_ack_hex}]")

        print("\nCOBS-encoded Acknowledgment Packet (hex):")
        encoded_ack_hex = " ".join([f"0x{byte:02X}" for byte in encoded_ack_packet])
        print(f"[{encoded_ack_hex}]")

        print("\nCopy-Paste Format (COBS-encoded Acknowledgment):")
        copy_paste_ack = "".join([f"{byte:02X}" for byte in encoded_ack_packet])
        print(copy_paste_ack)

    # Ask if user wants to build another packet
    print("\nBuild another packet? (y/n)")
    response = input("> ").strip().lower()
    if response.startswith("y"):
        return True
    else:
        return False


def main():
    parser = argparse.ArgumentParser(description="Build COBS-encoded packets for eVOLVER communication")
    parser.parse_args()

    print("===== eVOLVER COBS Packet Builder =====")

    while True:
        if not build_packet():
            break

    print("Goodbye!")


if __name__ == "__main__":
    main()
