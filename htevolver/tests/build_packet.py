import argparse

from htevolver.dependencies import EvolverPacket


def get_address() -> str:
    """Prompt the user for the address"""
    address_input: str = ""
    while True:
        print("\nEnter the address (integer or name from SerialAddressMap):")
        for address_name, address_value in EvolverPacket.serial_addresses.items():
            print(f"  {address_name} = {address_value}")
        address_input: str = input("> ").strip().lower()
        if address_input not in EvolverPacket.serial_addresses.keys():
            print("\nTry again, use a valid address input...")
        else:
            break
    return address_input


def get_payload_length() -> int:
    """Prompt the user for the payload length"""
    payload_length: int = 0
    while True:
        print("\nEnter the number of values in the command payload:")
        try:
            payload_length = int(input("> ").strip())
            if payload_length > 0:
                break
            else:
                print("\nTry again, data length must be greater than 0")
        except ValueError:
            print("\nTry again, use a valid integer")
    return payload_length


def get_command_type() -> str:
    """Prompt the user for the command type"""
    command_tag: str = ""
    while True:
        print("\nEnter the command type (integer or name from CommandTags):")
        for tag_name, tag_value in EvolverPacket.command_tags.items():
            print(f"  {tag_name} = {tag_value}")
        command_tag = input("> ").strip().lower()
        if command_tag not in EvolverPacket.command_tags.keys():
            print("\nTry again, enter a valid command tag...")
        else:
            break
    return command_tag


def get_data_payload(data_length) -> list[int]:
    """Prompt the user for the data payload values"""
    print(f"\nEnter {data_length} integer values for the data payload (one per line):")
    data_payload = []

    for i in range(data_length):
        while True:
            try:
                value = int(input(f"Value_{i + 1}> ").strip())
                if value < 0:
                    print("\nTry again, value must be positive...")
                else:
                    data_payload.append(value)
                    break
            except ValueError:
                print("\nTry again, enter a valid integer...")
                i -= 1

    return data_payload


def main():
    parser = argparse.ArgumentParser(description="Build COBS-encoded packets for eVOLVER communication")
    parser.parse_args()
    print("\n===== eVOLVER COBS Packet Builder =====")

    while True:
        """Build a packet based on user input"""
        # Get packet components from user
        address = get_address()
        data_length = get_payload_length()
        command_type = get_command_type()
        data_payload = get_data_payload(data_length)

        packet: EvolverPacket = EvolverPacket(address, data_length, command_type, data_payload, generate_ack=True)

        # Output the packet in multiple formats
        print("\n--- BUILD OUTPUT ---")
        print("\nRaw Packet (hex):")
        print(packet.get_decode_bytes(fmt=" "))

        print("\nCOBS-encoded Packet (hex):")
        print(packet.get_encode_bytes(fmt=" "))

        print("\nCopy-Paste Format (COBS-encoded):")
        print(packet.get_encode_bytes())

        # If this was a request packet, also generate the corresponding acknowledge packet
        if command_type == "request":
            print("\n=== Corresponding Acknowledgment Packet ===")

            # Output the acknowledgment packet in multiple formats
            print("\nRaw Acknowledgment Packet (hex):")
            print(packet.get_decode_bytes(use_ack=True))

            print("\nCOBS-encoded Acknowledgment Packet (hex):")
            print(packet.get_encode_bytes(fmt=" ", use_ack=True))

            print("\nCopy-Paste Format (COBS-encoded Acknowledgment):")
            print(packet.get_encode_bytes(use_ack=True))

        # Ask if user wants to build another packet
        print("\nBuild another packet? (y/n)")
        response = input("> ").strip().lower()
        if response.startswith("y"):
            return True
        else:
            return False


if __name__ == "__main__":
    continue_build: bool = True
    while continue_build:
        continue_build = main()
