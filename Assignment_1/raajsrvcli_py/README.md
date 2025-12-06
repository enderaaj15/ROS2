#In this example, the server is to sell car and the client chooses the car make, budget, and car model
#Note I made this seller to only sell BMW xD

#In one terminal, run: 
ros2 run raajsrvcli_py carseller

#In a second terminal, run: 
ros2 run raajsrvcli_py carbuyer

#Cont in the second terminal. 
#For the first prompt 'Server: Hi, what car do you want to buy?'
first try inputting 'Perodua' or 'Proton'
Then input 'Honda'
Next type 'No'
#seller will stop here

Again, run: ros2 run raajsrvcli_py carseller
This time type 'BMW'

#Next prompt will be 'Server: Great choice! What's your budget in RM?'
input 200000 or 300000 or any budget (There's choices for car models below and above RM 250,000 in the next prompt)

#Then the prompt will ask you to select a BMW model u want, choose 1 and hit enter!
