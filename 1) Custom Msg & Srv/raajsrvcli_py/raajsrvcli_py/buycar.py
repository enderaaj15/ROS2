import rclpy
from rclpy.node import Node
from raaj_interfaces.srv import BuyCar

class BuyCarClient(Node):
    def __init__(self):
        super().__init__('buycar_client')
        self.client = self.create_client(BuyCar, 'buy_car_service')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service...')

        self.run_conversation()

    def run_conversation(self):
        car_make = ""
        budget = 0.0
        car_model = ""

        # Step 1: Initial handshake to trigger server prompt
        req = BuyCar.Request()
        req.car_make = "INIT"  # special handshake string
        req.budget = 0.0
        req.car_model = ""

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res = future.result()
            print(f"Server: {res.message}")
        else:
            print("Failed to receive initial message.")
            return

        # 🛠️ Ask for input BEFORE loop starts to prevent second empty request
        car_make = input("You: ").strip()

        # Main interaction loop
        while True:
            req = BuyCar.Request()
            req.car_make = car_make
            req.budget = budget
            req.car_model = car_model

            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                res = future.result()
                print(f"Server: {res.message}")

                # Handle input based on server message
                if "what car do you want" in res.message.lower() or "please enter the car make" in res.message.lower():
                    car_make = input("You: ").strip()
                    budget = 0.0
                    car_model = ""
                elif "do you want to buy a bmw" in res.message.lower():
                    car_make = input("You (yes/no): ").strip()
                elif "budget" in res.message.lower():
                    try:
                        budget = float(input("Your Budget (RM): ").strip())
                    except ValueError:
                        print("Invalid number. Try again.")
                        budget = 0.0
                elif "which one would you like" in res.message.lower():
                    car_model = input("Model: ").strip()
                elif "thank you" in res.message.lower() or "goodbye" in res.message.lower():
                    break
                else:
                    car_make = input("You: ").strip()
            else:
                print("Service call failed.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = BuyCarClient()
    rclpy.shutdown()
