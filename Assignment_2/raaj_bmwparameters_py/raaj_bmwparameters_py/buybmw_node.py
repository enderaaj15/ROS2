import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class BuyCarParamNode(Node):
    def __init__(self):
        super().__init__('buycar_param_node')

        self.get_logger().info('BMW is ready to be bought. What is your budget?')

        # Full BMW car catalog with prices (RM)
        self.car_catalog = {
            'BMW 118i':                    150000,
            'BMW 116i':                    120000,
            'BMW 218i Active Tourer':      180000,
            'BMW X1 sDrive18i':            190000,
            'BMW 218i Gran Coupe Sport':   241000,
            'BMW 320i Sport':              265800,
            'BMW X3 20 xDrive M Sport':    320800,
            'BMW 530i M Sport':            399800,
            'BMW 330i M Sport':            340200,
            'BMW 2 Series Gran Coupe':     250000,
            'BMW X1':                      260000,
            'BMW 3 Series Sedan':          300000,
            'BMW X4':                      380000,
            'BMW Z4':                      400000,
            'BMW i4':                      430000,
            'BMW i5':                      460000,
            'BMW X5':                      480000,
            'BMW X6':                      550000,
            'BMW M340i xDrive':            580000,
            'BMW 420i Coupe':              255000,
            'BMW X2':                      280000,
            'BMW 330Li M Sport':           320000
        }

        # Allowed colors
        self.allowed_colors = ['black', 'white', 'blue', 'silver']

        # Allowed banks and their interest rates (annual)
        self.bank_interest = {
            'maybank': 0.035,     
            'cimb': 0.037,        
            'public_bank': 0.036, 
            'rhb': 0.038          
        }

        # Allowed loan years (5 to 9 inclusive)
        self.allowed_loan_years = range(5, 10)  

        # Declare parameters
        self.declare_parameter('downpayment', 0)
        self.declare_parameter('budget', 200000)  
        self.declare_parameter('car_model', 'BMW 116i')
        self.declare_parameter('car_color', 'white')
        self.declare_parameter('loan_years', 5)
        self.declare_parameter('bank', 'rhb')
        self.over_budget_count = 0   

        # Setup parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('Waiting for parameters to be set.')
        self.purchase_timer = self.create_timer(4.0, self.evaluate_purchase)        

    def parameter_callback(self, params):
        for param in params:
            name = param.name
            value = param.value

            if name == 'budget':
                if not isinstance(value, int):
                    self.get_logger().error(f'Budget must be an integer. Rejecting parameter.')
                    return SetParametersResult(successful=False)

                if not (value >= 120_000):
                    self.get_logger().error(
                        f'Budget RM{value:,} is too low (<RM120,000). Rejecting parameter.'
                    )
                    return SetParametersResult(successful=False)
                else:
                    self.get_logger().info(f'Budget set to RM{value:,}')
                    models = [m for m, p in self.car_catalog.items() if p <= value]
                    self.get_logger().info(f'Choose car model. Models available within budget: {models}')


            elif name == 'car_model':
                if value not in self.car_catalog:
                    self.get_logger().error(
                        f'Model "{value}" is not in catalog. Rejecting parameter.'
                    )
                    return SetParametersResult(successful=False)
                else:
                    self.get_logger().info(f'{value} chosen. Sweet choice! Now choose the color of your ride.')

            elif name == 'car_color':
                color = value.lower()
                if color not in self.allowed_colors:
                    self.get_logger().error(
                        f'Color "{color}" not allowed. Choose from {self.allowed_colors}. Rejecting parameter.'
                    )
                    return SetParametersResult(successful=False)
                else:
                    self.get_logger().info(f'Color set to {color}. Now choose your preferred loan repayments years')

            elif name == 'loan_years':
                if not isinstance(value, int):
                    self.get_logger().error(f'Loan years must be an integer. Rejecting parameter.')
                    return SetParametersResult(successful=False)

                if value not in self.allowed_loan_years:
                    self.get_logger().error(
                        f'Loan years must be between {min(self.allowed_loan_years)} and {max(self.allowed_loan_years)} inclusive. Rejecting parameter.'
                    )
                    return SetParametersResult(successful=False)
                else:
                    self.get_logger().info(f'Loan repayment years set to {value}. Which bank are you applying loan from?')

            elif name == 'bank':
                bank = value.lower()
                if bank not in self.bank_interest:
                    self.get_logger().error(
                        f'Bank "{bank}" not recognized. Choose from {list(self.bank_interest.keys())}. Rejecting parameter.'
                    )
                    return SetParametersResult(successful=False)
                else:
                    self.get_logger().info(f'Bank set to {bank}. How much downpayment?')
                    
            elif name == 'downpayment':
                if not isinstance(value, int) or value < 0:
                    self.get_logger().error(f'Downpayment must be a positive integer. Rejecting parameter.')
                    return SetParametersResult(successful=False)
                else:
                    self.get_logger().info(f'Downpayment set to RM{value:,}.')

        return SetParametersResult(successful=True)
                    

    # Automatically reevaluate after any valid parameter change
    def evaluate_purchase(self):
        param_vals = self.get_parameters(['budget', 'car_model', 'car_color', 'loan_years', 'bank', 'downpayment'])
        vals = {p.name: p.value for p in param_vals}

        # Validate all parameters
        if (
            isinstance(vals['budget'], int) and vals['budget'] >= 120_000 and
            vals['car_model'] in self.car_catalog and
            vals['car_color'].lower() in self.allowed_colors and
            isinstance(vals['loan_years'], int) and vals['loan_years'] in self.allowed_loan_years and
            vals['bank'].lower() in self.bank_interest and
            isinstance(vals['downpayment'], int) and vals['downpayment'] >= 0
        ):
            price = self.car_catalog[vals['car_model']]
            downpayment = vals['downpayment']
            models = [m for m, p in self.car_catalog.items() if p <= vals['budget']]

            if downpayment >= price:
                self.get_logger().error(
                    f'Downpayment RM{downpayment:,} must be less than car price RM{price:,}. Rejecting calculation.'
                )
                return SetParametersResult(successful=False)

            if price > vals['budget']:
                self.get_logger().warn(
                    f'Selected model "{vals["car_model"]}" costs RM{price:,}, which exceeds your budget of RM{vals["budget"]:,}.'
                )
                self.get_logger().warn(
                    f'Available models within your budget: {models}'
                )

                self.over_budget_count += 1

                #After 3 warnings
                if 3<= self.over_budget_count <= 6:
                    self.get_logger().info(
                        f'Bro... either increase your budget or choose a model within your budget and stop dreaming of "{vals["car_model"]}" lah 🤣'
                    )
                elif self.over_budget_count > 6:
                    self.get_logger().info(
                        f'Still want "{vals["car_model"]}"? Maybe try lottery first bro 😭'
                    )                    
                return SetParametersResult(successful=False)
                
            loan_amount = price - downpayment
            years = vals['loan_years']
            bank = vals['bank'].lower()
            rate = self.bank_interest[bank]

            monthly_rate = rate / 12
            months = years * 12

            monthly_payment = (loan_amount * monthly_rate) / (1 - (1 + monthly_rate) ** -months)
      
            color = vals['car_color'].lower() 
            
            self.get_logger().info(
                f'Monthly repayment for your {color} {vals["car_model"]} over {years} years via {bank}: RM{monthly_payment:,.2f}'
            )
            self.get_logger().info(
                f'🎉 Congratulations! You are now the proud owner of a {color} {vals["car_model"]}!'
            )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = BuyCarParamNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

