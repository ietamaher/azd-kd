#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from azd_kd.msg import Query, Response, State
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder
from rclpy.exceptions import ParameterNotDeclaredException 

class ModbusNode(Node):
    def __init__(self):
        super().__init__('azd_kd_modbus_node')

        # Load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('method', 'rtu'),
                ('port', 'com1'),
                ('baudrate', 9600),
                ('timeout', 1),
                ('stopbits', 1),
                ('parity', 'N')
            ])      
 
        # Retrieve parameter values
        self.method = self.get_parameter('method').value
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.stopbits = self.get_parameter('stopbits').value
        self.parity = self.get_parameter('parity').value

        # Log parameter values for debugging
        self.get_logger().info('Modbus configuration:')
        self.get_logger().info(f'Method: {self.method}')
        self.get_logger().info(f'Port: {self.port}')
        self.get_logger().info(f'Baudrate: {self.baudrate}')
        self.get_logger().info(f'Timeout: {self.timeout}')
        self.get_logger().info(f'Stopbits: {self.stopbits}')
        self.get_logger().info(f'Parity: {self.parity}')
 
        # Create subscribers and publishers
        self.query_subscriber = self.create_subscription(Query, 'azd_kd_query', self.query_callback, 10)
        self.response_publisher = self.create_publisher(Response, 'azd_kd_response', 10)
        self.state_publisher = self.create_publisher(State, 'azd_kd_state', 10)

    def query_callback(self, query_msg):
        # Upon receiving a query
        slave_id = query_msg.slave_id
        func_code = query_msg.func_code
        write_addr = query_msg.write_addr
        read_addr = query_msg.read_addr
        write_num = query_msg.write_num
        read_num = query_msg.read_num
        data = query_msg.data

        try:
            if func_code == 1:  # Write function code
                self.write_register(slave_id, write_addr, write_num, data)
            elif func_code == 2:  # Read function code
                self.read_holding_registers(slave_id, read_addr, read_num)
            else:
                # Handle unsupported function code
                pass
        except Exception as e:
            # Handle Modbus communication error
            self.handle_modbus_error(e)

    def write_register(self, slave_id, write_addr, write_num, data):
        client = self.get_modbus_client()

        try:
            if client.connect():
                # Construct the payload
                datasplit_into_int16 = [((value >> 16) & 0xFFFF, value & 0xFFFF) for value in data]

                # Flatten the list of tuples into a single list
                flattened_data = [val for tup in datasplit_into_int16 for val in tup]

                # Write the flattened data to the specified address
                self.get_logger().info(f"Writing to Modbus: slave_id={slave_id}, write_addr={write_addr}, write_num={write_num}, data={flattened_data}")

                selected_data = flattened_data[:64]

                query = client.write_registers(write_addr, selected_data, unit=slave_id)
  
                # Publish the response
                response_msg = Response()
                response_msg.data[0] = slave_id
                response_msg.data[1] = 16
                response_msg.data[2] = (write_addr >> 8) & 0xFF
                response_msg.data[3] = write_addr & 0xFF
                response_msg.data[4] = (write_num >> 8) & 0xFF
                response_msg.data[5] = write_num & 0xFF

                response_frame = [
                    response_msg.data[0],
                    response_msg.data[1],
                    response_msg.data[2],
                    response_msg.data[3],
                    response_msg.data[4],
                    response_msg.data[5],
                ]                

                response_msg.data[6],response_msg.data[7] = self.calculate_crc(response_frame)                
                response_msg.slave_id = slave_id
                response_msg.func_code = 16 # Write function code
                self.response_publisher.publish(response_msg)

                # Publish the state indicating success
                self.publish_state(0, 1, 0)

        except Exception as e:
            # Handle Modbus communication error
            self.handle_modbus_error(e)
            # Publish the state indicating an error
            self.publish_state(1, 0, 1)
        finally:
            client.close()

    def read_holding_registers(self, slave_id, read_addr, read_num):
        client = self.get_modbus_client()

        try:
            if client.connect():
                # Read the holding registers
                response = client.read_holding_registers(address=read_addr, count=read_num * 2, unit=slave_id)

                if response.isError():
                    # Handle Modbus error (no response, exception response, etc.)
                    self.handle_modbus_error(response)
                    # Publish the state indicating an error
                    self.publish_state(1, 0, 1)
                else:
                    # Transform every 2 values into a single int32 value
                    decoded_data = []
                    for i in range(0, len(response.registers), 2):
                        high_word = response.registers[i]
                        low_word = response.registers[i + 1]
                        int32_value = (high_word << 16) | low_word
                        decoded_data.append(int32_value)

                     # Ensure decoded_data has the correct length (64) by padding with zeros if needed
                    decoded_data += [0] * (64 - len(decoded_data))
                    
                    # Publish the response
                    response_msg = Response()
                    response_msg.data = decoded_data
                    response_msg.slave_id = slave_id
                    response_msg.func_code = 2  # Read function code
                    self.response_publisher.publish(response_msg)

                    # Publish the state indicating success
                    self.publish_state(0, 1, 0)
        except Exception as e:
            # Handle Modbus communication error
            self.handle_modbus_error(e)
            # Publish the state indicating an error
            self.publish_state(1, 0, 1)
        finally:
            client.close()
 
 

    def get_modbus_client(self):
        return ModbusSerialClient(
            method=self.method,
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

    def handle_modbus_error(self, exception):
        # Handle Modbus communication error
        self.get_logger().error('Modbus communication error: %s' % exception)


    def publish_state(self, state_driver, state_mes, state_error):
        # Publish the state
        state_msg = State()
        state_msg.state_driver = state_driver
        state_msg.state_mes = state_mes
        state_msg.state_error = state_error

        # Assuming these are attributes in the State message.
        # Adjust the assignments based on your actual State message structure.
    
        # Publish the state message
        self.state_publisher.publish(state_msg)

    # Rest of the code...
    def calculate_crc(self, data):
        """
        Calculate CRC-16 for Modbus frame.
        :param data: List of integers representing the Modbus frame.
        :return: Calculated CRC-16 value.
        """
        crc = 0xFFFF

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1

         # Swap bytes
        crc_high_byte = (crc & 0xFF00) >> 8
        crc_low_byte = crc & 0xFF

        return crc_high_byte, crc_low_byte

def main(args=None):
    rclpy.init(args=args)
    try:
        modbus_node = ModbusNode()
        rclpy.spin(modbus_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'modbus_node' in locals():
            modbus_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
