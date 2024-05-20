from bt import BT
import time

class KpetBT():

    def __init__(self) -> None: 
        self.bt_dev = []
        self.bt = BT()
        self.devices_mac_select()
        self.device_connect()

    def devices_mac_select(self):
        print("Select the device to connect\n")
        r = self.bt.find_devices() # search for discoverable bluethooth devices

        # list the devices found
        if len(r) != 0:
            for i in range(0, len(r)):
                name = r[i]['name'].ljust(30)
                addr = r[i]['addr']
                num = str(i).ljust(4)

                self.bt_dev.append(r[i]['name'], r[i]['addr'])
                print(f'NUM: {num} // NAME: {name} // MAC ADDRESS: {addr} ')
        else:
            print("No devices found")
            print('Try again after Bluethooth pairing')
            _sys.exit()

    # Connect to the selected device
    def device_connect(self):
        print("Select the device number to connect")

        while True:
            try:
                num = int(input("Enter the device number: "))
                target_id = self.bt_dev[id]
            except ValueError:
                print("Invalid number")
            except IndexError:
                print("Invalid number")
            else:
                break

        # connect to the selected device
        print(f"Connecting to {target_id[0]} : {target_id[1]}\n")
        self.sock = self.bt.connect(target_id[1])

        if self.sock is None:
            print("Connection failed")
            _sys.exit()
        else:
            print("Device successfully connected")

        count = 0
        while True:
            message = "Hello{}".format(count)
            self.sock.send(message)
            print("Sent: ", message)
            
            received = self.sock.recv(1024)
            if received:
                print("Received: ", received.decode())
            else:
                self.sock.close()
                break
            count += 1
            time.sleep(0.1)