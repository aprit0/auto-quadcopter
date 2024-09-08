from yamspy import MSPy


serial_port = "/dev/ttyUSB0"

with MSPy(device=serial_port, loglevel='DEBUG', baudrate=115200) as board:
    # Read info from the FC
    # Please, pay attention to the way it works:
    # 1. Message is sent: MSP_ALTITUDE without any payload (data=[])
    print("loop")
    if board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'], data=[]):
        # 2. Response msg from the flight controller is received
        dataHandler = board.receive_msg()
        # 3. The msg is parsed
        board.process_recv_data(dataHandler)
        # 4. After the parser, the instance is populated.
        # In this example, SENSOR_DATA has its altitude value updated.
        print(board.SENSOR_DATA['altitude'])