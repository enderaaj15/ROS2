#Everytime the publisher ask if that is the amount of car the subscriber has, the subs will answer the same amount (example: 3 cars? answer: yes 3 cars, all 3 are BMWs)
#In terminal 1, run:
ros2 run raajpubsub_py askcar

Output:
[INFO] [1758466809.154320162] [minimal_publisher]: How many cars do you have? "43 cars"?
[INFO] [1758466809.655400358] [minimal_publisher]: How many cars do you have? "44 cars"?
[INFO] [1758466810.155566431] [minimal_publisher]: How many cars do you have? "45 cars"?

#In terminal 2, run:
ros2 run raajpubsub_py tellcar

Output:
[INFO] [1758466809.180725378] [minimal_subscriber]: I have "43 cars". All 43 are BMWs
[INFO] [1758466809.656066380] [minimal_subscriber]: I have "44 cars". All 44 are BMWs
[INFO] [1758466810.156207862] [minimal_subscriber]: I have "45 cars". All 45 are BMWs
