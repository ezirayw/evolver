from htevolver_client.client import HTEvolverClient

# Create client and connect
client = HTEvolverClient(ip="192.168.1.2", directory="")
client.connect()

# Access the namespace objects directly
# Get temperature data from the evolver namespace
station = client.evolver.stations[0]  # Access first station
latest_temp = station.temp[-1]  # Get latest temperature reading

# Send commands through the evolver namespace
client.evolver.send_command("temp_config", [30, 30, 30, 30], immediate=True, recurring=True)
client.evolver.run_ipps({0: 10, 1: -5})  # Run IPPs on stations 0 and 1

# Use the robotics namespace for robotics operations
client.robotics.prime_syringe_pumps({0: ("MEDIA", 500), 1: ("DRUG", 500)})
client.robotics.fill_vials({0: ("MEDIA", 1000)})

# Disconnect when done
client.disconnect()
