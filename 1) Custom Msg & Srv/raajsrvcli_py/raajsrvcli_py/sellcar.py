import rclpy
from rclpy.node import Node
from raaj_interfaces.srv import BuyCar

class BuyCarServer(Node):
    def __init__(self):
        super().__init__('buycar_server')
        self.srv = self.create_service(BuyCar, 'buy_car_service', self.handle_request)
        self.get_logger().info('BuyCar server is ready.')
        self.client_state = 'ask_make'

    def handle_request(self, request, response):
        car_make = request.car_make.strip().lower()
        budget = request.budget
        car_model = request.car_model.strip().lower()

        # Initial handshake
        if car_make == "init":
            self.client_state = 'ask_make'
            response.message = "Hi, what car do you want to buy?"
            return response

        # Step 1: Ask for car make
        if self.client_state == 'ask_make':
            if car_make == 'bmw':
                self.client_state = 'ask_budget'
                response.message = "Great choice! What's your budget in RM?"
            elif car_make in ['ferrari', 'toyota', 'honda', 'mercedes']:
                self.client_state = 'confirm_bmw'
                response.message = "You came to the wrong place, we only sell BMWs here.\nDo you want to buy a BMW or not?"
            else:
                response.message = "Sorry, we only sell BMWs. Please enter the car make again:"

        # Step 1b: Confirm BMW
        elif self.client_state == 'confirm_bmw':
            if car_make in ['yes', 'y']:
                self.client_state = 'ask_budget'
                response.message = "Great! What's your budget in RM?"
            else:
                self.client_state = 'ask_make'
                response.message = "Okay, goodbye!"

        # Step 2: Ask budget
        elif self.client_state == 'ask_budget':
            if budget <= 0:
                response.message = "Please enter a valid budget (in RM, e.g. 300000):"
            elif budget > 250000.0:
                self.client_state = 'choose_model_high'
                response.message = "Models for > RM250,000: BMW M5, BMW X6, BMW i8. Which one would you like?"
            else:
                self.client_state = 'choose_model_low'
                response.message = "Models for ≤ RM250,000: BMW 320i, BMW X1, BMW 118i. Which one would you like?"

        # Step 3: Choose model
        elif self.client_state in ['choose_model_high', 'choose_model_low']:
            if car_model:
                response.car_make = 'BMW'
                response.budget = budget
                response.car_model = car_model
                response.message = f"Thank you for your purchase of the {car_model.title()}! Enjoy your new ride!"
                self.client_state = 'done'
            else:
                response.message = "Please enter the model name you want to buy."

        # Step 4: Restart
        elif self.client_state == 'done':
            self.client_state = 'ask_make'
            response.message = "Hi, what car do you want to buy?"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = BuyCarServer()
    rclpy.spin(node)
    rclpy.shutdown()

